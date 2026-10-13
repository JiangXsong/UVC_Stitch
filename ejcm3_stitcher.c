#include <linux/module.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/kallsyms.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/scatterlist.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

/* ------------------------------------------------------------------ */
/* Kernel version compatibility shims                                 */
/* ------------------------------------------------------------------ */
/* iosys_map was renamed from dma_buf_map in v5.18 (commit 7938f4218168). */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
#define iosys_map		dma_buf_map
#define iosys_map_set_vaddr	dma_buf_map_set_vaddr
#endif

/*
 * v6.4 made struct class pointers const and dropped the class_interface*
 * argument from the add_dev/remove_dev callbacks (Greg KH constify series).
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
#define STITCHER_CLASS_CONST
#else
#define STITCHER_CLASS_CONST const
#endif

/* vb2_get_num_buffers() was added in v6.8; provide it for older kernels. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
static inline unsigned int vb2_get_num_buffers(struct vb2_queue *q)
{
	return q->num_buffers;
}
#endif

#define STITCHER_NUM_SOURCES 2
#define STITCHER_SRC_NUM_BUFS 4
#define STITCHER_OUT_NUM_BUFS 4
#define STITCHER_MAX_OUT_BUFS 8
#define STITCHER_SYNC_DRAIN_FRAMES 4
#define STITCHER_SYNC_MAX_RETRIES 3
#define STITCHER_SEQ_MISMATCH_THRESHOLD 2

#define STITCHER_DEFAULT_VID 0x1E4E
#define STITCHER_DEFAULT_PID_P 0x7301
#define STITCHER_DEFAULT_PID_S 0x7302

#define STITCHER_DEFAULT_OUT_WIDTH 3840
#define STITCHER_DEFAULT_OUT_HEIGHT 2160
#define STITCHER_DEFAULT_FPS_NUM 1
#define STITCHER_DEFAULT_FPS_DEN 60

/**
 * UVC_CALL_OP - call a UVC ioctl op directly, holding the vdev lock.
 *
 * NOTE: This bypasses V4L2 core's capability checks and v4l2_fh state
 * validation. It is intentional for this kernel-space stitching design, but
 * callers must ensure the source filp/vdev remain valid (protected by
 * src_disc_lock or uvc_ctrl_lock as appropriate).
 */
#define UVC_CALL_OP(src, op, ...)                                     \
	({                                                            \
		int __ret = -ENOTTY;                                  \
		struct video_device *__vd = (src)->vdev;              \
		if (__vd && __vd->ioctl_ops && __vd->ioctl_ops->op) { \
			if (__vd->lock)                               \
				mutex_lock(__vd->lock);               \
			__ret = __vd->ioctl_ops->op(__VA_ARGS__);     \
			if (__vd->lock)                               \
				mutex_unlock(__vd->lock);             \
		}                                                     \
		__ret;                                                \
	})

/* ------------------------------------------------------------------ */
struct stitcher_slot {
	struct file *filp;
	struct video_device *vdev;
	struct v4l2_fh *fh;
	struct vb2_queue *vbq;
	struct v4l2_format fmt;

	struct vb2_buffer *cur_vb;
	ktime_t cur_ts;
	bool buf_ready;

	struct usb_interface *intf;
	struct usb_device *udev;
};

/* dma_buf exporter private data */
struct stitcher_dmabuf_priv {
	struct page **pages;
	unsigned int num_pages;
	unsigned int offset;
	size_t size;
};

/**
 * Per-attachment cached SG table.
 * Created once on first map_dma_buf, reused on subsequent calls,
 * freed on detach.
 */
struct stitcher_dmabuf_attach {
	struct sg_table *sgt;
	enum dma_data_direction dir;
	bool mapped;
};

/**
 * Per-output-buffer DMABUF tracking.
 * Each output buffer has one dma_buf per source (covering its half).
 * cached_fd: persistent fd in init_files for kthread re-QBUF.
 */
struct stitcher_dmabuf_slot {
	struct dma_buf *dbuf[STITCHER_NUM_SOURCES];
	int cached_fd[STITCHER_NUM_SOURCES];
	struct page **pages;
	unsigned int num_pages;
	bool created;
};

struct stitcher_video_buffer {
	struct vb2_v4l2_buffer buf;
	struct list_head queue;
};

struct stitcher_disc_work {
	struct work_struct work;
	struct uvc_stitcher *stitcher;
	struct video_device *vdev;
	bool is_add;
};

struct stitcher_video_queue {
	struct vb2_queue vbq;
	struct mutex mutex;

	spinlock_t irqlock;
	struct list_head irqqueue;
	wait_queue_head_t buf_wq;
};

struct uvc_stitcher {
	struct v4l2_device v4l2_dev;
	struct video_device *vdev;
	struct mutex lock;

	struct class_interface class_intf;
	int src_ready_count;
	struct mutex src_disc_lock;

	struct workqueue_struct *disc_wq;
	struct stitcher_slot src[STITCHER_NUM_SOURCES];

	u32 out_width;
	u32 out_height;
	u32 out_pixfmt;
	u32 src_width;
	u32 src_height;
	u32 fps_num;
	u32 fps_den;

	struct stitcher_video_queue out_q;
	bool out_q_initialized;

	struct mutex uvc_ctrl_lock;

	struct task_struct *stitcher_task;
	atomic_t streaming;

	bool use_dmabuf;
	unsigned int num_dmabuf_slots;
	atomic_t bufs_in_flight;
	struct stitcher_dmabuf_slot dma_slots[STITCHER_MAX_OUT_BUFS];
	struct vb2_buffer *out_vb_map[STITCHER_MAX_OUT_BUFS];
};

/* ------------------------------------------------------------------ */
/* Forward declarations                                               */
/* ------------------------------------------------------------------ */
static int stitcher_uvc_start(struct uvc_stitcher *stitcher);
static void stitcher_uvc_stop(struct uvc_stitcher *stitcher);
static int stitcher_register_class_intf(struct uvc_stitcher *stitcher,
					STITCHER_CLASS_CONST struct class *vcls);
static void stitcher_unregister_class_intf(struct uvc_stitcher *stitcher);
static void stitcher_unregister(struct uvc_stitcher *stitcher);
static int init_queue(struct stitcher_video_queue *q);
static void stitcher_queue_release(struct stitcher_video_queue *q);
static int stitcher_thread_fn(void *data);

static struct uvc_stitcher *g_stitcher;

/* ------------------------------------------------------------------ */
/* Module parameters                                                  */
/* ------------------------------------------------------------------ */
static bool zerocopy = true;
module_param(zerocopy, bool, 0444);
MODULE_PARM_DESC(zerocopy,
		 "Enable DMABUF feed-through zero-copy (default: true)");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Syu-Song Chiang");
MODULE_DESCRIPTION("UVC Stitcher: Stitching two UVC sources in kernel space");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
MODULE_IMPORT_NS("DMA_BUF");
#else
MODULE_IMPORT_NS(DMA_BUF);
#endif

/* ------------------------------------------------------------------ */
/* Inline helpers                                                     */
/* ------------------------------------------------------------------ */
static inline bool stitcher_src_valid(const struct stitcher_slot *src)
{
	return !IS_ERR_OR_NULL(src->filp) && src->vdev;
}

static inline bool stitcher_should_stop(const struct uvc_stitcher *stitcher)
{
	return kthread_should_stop() || !atomic_read(&stitcher->streaming);
}

static inline bool stitcher_is_fatal_err(int err)
{
	return err == -ENODEV || err == -EIO;
}

/* ------------------------------------------------------------------ */
/* Pixel-format helpers                                               */
/* ------------------------------------------------------------------ */
/**
 * stitcher_image_size - return total bytes for one frame at given w/h/pixfmt.
 *   YUYV : w * h * 2
 *   NV12 : w * h * 3 / 2
 */
static inline unsigned int stitcher_image_size(u32 w, u32 h, u32 pixfmt)
{
	if (pixfmt == V4L2_PIX_FMT_NV12)
		return w * h * 3 / 2;
	return w * h * 2;
}

/**
 * stitcher_bytesperline - return bytes-per-line for the given format.
 *   YUYV : w * 2   (packed, single plane)
 *   NV12 : w       (Y stride; UV stride is the same for interleaved UV)
 */
static inline unsigned int stitcher_bytesperline(u32 w, u32 pixfmt)
{
	if (pixfmt == V4L2_PIX_FMT_NV12)
		return w;
	return w * 2;
}

static inline bool stitcher_pixfmt_valid(u32 pixfmt)
{
	return pixfmt == V4L2_PIX_FMT_YUYV || pixfmt == V4L2_PIX_FMT_NV12;
}

/* ------------------------------------------------------------------ */
/* dma_buf exporter ops                                               */
/* ------------------------------------------------------------------ */
static int stitcher_dmabuf_attach(struct dma_buf *dbuf,
				  struct dma_buf_attachment *attach)
{
	struct stitcher_dmabuf_attach *a;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	attach->priv = a;
	return 0;
}

static void stitcher_dmabuf_detach(struct dma_buf *dbuf,
				   struct dma_buf_attachment *attach)
{
	struct stitcher_dmabuf_attach *a = attach->priv;

	if (!a)
		return;

	if (a->sgt) {
		if (a->mapped)
			dma_unmap_sgtable(attach->dev, a->sgt, a->dir, 0);
		sg_free_table(a->sgt);
		kfree(a->sgt);
	}
	kfree(a);
	attach->priv = NULL;
}

/**
 * map_dma_buf — called by vb2 on each QBUF (after DQBUF's unmap).
 *
 * The SG table + DMA mapping is created ONCE on the first call and
 * cached in the attachment.  Subsequent calls return the cached table.
 * This eliminates per-frame:
 *   - sg_alloc_table_from_pages (kmalloc + page iteration)
 *   - dma_map_sgtable (IOMMU page table update if VT-d enabled)
 *
 * Pages are static (vmalloc, pinned for stream lifetime) so the
 * cached DMA addresses remain valid across frames.
 */
static struct sg_table *stitcher_dmabuf_map(struct dma_buf_attachment *attach,
					    enum dma_data_direction dir)
{
	struct stitcher_dmabuf_attach *a = attach->priv;
	struct stitcher_dmabuf_priv *priv = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	/* Return cached SG table if available */
	if (a->sgt)
		return a->sgt;

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table_from_pages(sgt, priv->pages, priv->num_pages,
					priv->offset, priv->size, GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}

	ret = dma_map_sgtable(attach->dev, sgt, dir, 0);
	if (ret) {
		sg_free_table(sgt);
		kfree(sgt);
		return ERR_PTR(ret);
	}

	/* Cache for reuse */
	a->sgt = sgt;
	a->dir = dir;
	a->mapped = true;

	return sgt;
}

/**
 * unmap_dma_buf — called by vb2 on each DQBUF.
 *
 * Intentional NO-OP: SG table is cached in attachment and freed
 * on detach.  This is the standard pattern for exporters with
 * static (pinned) backing pages.
 */
static void stitcher_dmabuf_unmap(struct dma_buf_attachment *attach,
				  struct sg_table *sgt,
				  enum dma_data_direction dir)
{
	/* Cached — freed on detach */
}

static int stitcher_dmabuf_pin(struct dma_buf_attachment *attach)
{
	/* Pages are vmalloc — always pinned, never move */
	return 0;
}

static void stitcher_dmabuf_unpin(struct dma_buf_attachment *attach)
{
}

static int stitcher_dmabuf_begin_cpu_access(struct dma_buf *dbuf,
					    enum dma_data_direction dir)
{
	/* x86: cache-coherent, no action needed */
	return 0;
}

static int stitcher_dmabuf_end_cpu_access(struct dma_buf *dbuf,
					  enum dma_data_direction dir)
{
	return 0;
}

static void stitcher_dmabuf_release(struct dma_buf *dbuf)
{
	kfree(dbuf->priv);
}

static int stitcher_dmabuf_vmap(struct dma_buf *dbuf, struct iosys_map *map)
{
	struct stitcher_dmabuf_priv *priv = dbuf->priv;
	void *va;

	va = vmap(priv->pages, priv->num_pages, VM_MAP, PAGE_KERNEL);
	if (!va)
		return -ENOMEM;

	iosys_map_set_vaddr(map, va + priv->offset);
	return 0;
}

static void stitcher_dmabuf_vunmap(struct dma_buf *dbuf, struct iosys_map *map)
{
	struct stitcher_dmabuf_priv *priv;
	void *va = map->vaddr;

	if (va) {
		priv = dbuf->priv;
		vunmap(va - priv->offset);
	}
}

static const struct dma_buf_ops stitcher_dmabuf_ops = {
	// clang-format off
	.attach           = stitcher_dmabuf_attach,
	.detach           = stitcher_dmabuf_detach,
	.pin              = stitcher_dmabuf_pin,
	.unpin            = stitcher_dmabuf_unpin,
	.map_dma_buf      = stitcher_dmabuf_map,
	.unmap_dma_buf    = stitcher_dmabuf_unmap,
	.begin_cpu_access = stitcher_dmabuf_begin_cpu_access,
	.end_cpu_access   = stitcher_dmabuf_end_cpu_access,
	.release          = stitcher_dmabuf_release,
	.vmap             = stitcher_dmabuf_vmap,
	.vunmap           = stitcher_dmabuf_vunmap,
	// clang-format on
};

/* ------------------------------------------------------------------ */
/* DMABUF management helpers                                          */
/* ------------------------------------------------------------------ */
/**
 * stitcher_create_dmabuf - export a byte range of a vmalloc page array as dma_buf.
 *
 * @pages:     array of struct page pointers (from vmalloc_to_page)
 * @num_pages: number of pages in array
 * @offset:    byte offset within first page
 * @size:      total byte size of the range
 *
 * The dma_buf holds a reference to priv, not to the pages themselves.
 * Pages must outlive the dma_buf (output buffer freed after dma_buf).
 */
static struct dma_buf *stitcher_create_dmabuf(struct page **pages,
					      unsigned int num_pages,
					      unsigned int offset, size_t size)
{
	struct stitcher_dmabuf_priv *priv;
	struct dma_buf *dbuf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->pages = pages;
	priv->num_pages = num_pages;
	priv->offset = offset;
	priv->size = size;

	exp_info.ops = &stitcher_dmabuf_ops;
	exp_info.size = size;
	exp_info.flags = O_RDWR;
	exp_info.priv = priv;

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf))
		kfree(priv);

	return dbuf;
}

static int stitcher_dqbuf_src_dmabuf(struct uvc_stitcher *stitcher, int slot,
				     unsigned int *out_index, u32 *out_sequence,
				     ktime_t *out_ts)
{
	struct stitcher_slot *src = &stitcher->src[slot];
	struct v4l2_buffer buf = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	int ret;

	if (!stitcher_src_valid(src)) {
		pr_info("uvc_stitcher: src%d dqbuf skipped (invalid state)\n",
			slot);
		return -ENODEV;
	}

	ret = UVC_CALL_OP(src, vidioc_dqbuf, src->filp, src->fh, &buf);
	if (ret == -EINVAL && !atomic_read(&stitcher->streaming))
		return ret;
	if (ret) {
		pr_err("uvc_stitcher: src%d DQBUF failed: %d\n", slot, ret);
		return ret;
	}

	*out_index = buf.index;
	if (out_sequence)
		*out_sequence = buf.sequence;
	if (out_ts)
		*out_ts =
			ns_to_ktime((u64)buf.timestamp.tv_sec * NSEC_PER_SEC +
				    (u64)buf.timestamp.tv_usec * NSEC_PER_USEC);
	return 0;
}

/**
 * stitcher_qbuf_src_dmabuf - QBUF a dma_buf to UVC source via fd.
 *
 * @fd: file descriptor pointing to the dma_buf.
 */
static int stitcher_qbuf_src_dmabuf(struct uvc_stitcher *stitcher, int slot,
				    unsigned int buf_index, int fd)
{
	struct stitcher_slot *src = &stitcher->src[slot];
	struct v4l2_buffer v4l2_buf = {
		.index = buf_index,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	int ret;

	if (!stitcher_src_valid(src))
		return -ENODEV;

	v4l2_buf.m.fd = fd;
	ret = UVC_CALL_OP(src, vidioc_qbuf, src->filp, src->fh, &v4l2_buf);

	if (ret)
		pr_err("uvc_stitcher: dmabuf QBUF src%d idx%u fd%d failed: %d\n",
		       slot, buf_index, fd, ret);

	return ret;
}

/**
 * stitcher_dmabuf_qbuf_with_tmpfd - QBUF using a temporary fd.
 *
 * Creates a temporary fd in current->files, calls QBUF, closes fd.
 */
static int stitcher_dmabuf_qbuf_with_tmpfd(struct uvc_stitcher *stitcher,
					   int slot, unsigned int buf_index,
					   struct dma_buf *dbuf)
{
	int fd, ret;

	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0)
		return fd;

	get_file(dbuf->file);
	fd_install(fd, dbuf->file);

	ret = stitcher_qbuf_src_dmabuf(stitcher, slot, buf_index, fd);

	close_fd(fd);
	return ret;
}

/**
 * stitcher_dmabuf_create_cached_fds - create persistent fds for re-QBUF.
 */
static int stitcher_dmabuf_create_cached_fds(struct uvc_stitcher *stitcher)
{
	struct stitcher_dmabuf_slot *ds;
	unsigned int i;
	int s, fd;

	for (i = 0; i < stitcher->num_dmabuf_slots; i++) {
		ds = &stitcher->dma_slots[i];

		for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
			if (!ds->dbuf[s])
				continue;

			fd = get_unused_fd_flags(O_CLOEXEC);
			if (fd < 0) {
				pr_err("uvc_stitcher: get_unused_fd failed: %d\n",
				       fd);
				return fd;
			}

			get_file(ds->dbuf[s]->file);
			fd_install(fd, ds->dbuf[s]->file);
			ds->cached_fd[s] = fd;

			pr_debug(
				"uvc_stitcher: cached fd %d for slot%u src%d\n",
				fd, i, s);
		}
	}
	return 0;
}

/**
 * stitcher_dmabuf_close_cached_fds - close all cached fds.
 */
static void stitcher_dmabuf_close_cached_fds(struct uvc_stitcher *stitcher)
{
	struct stitcher_dmabuf_slot *ds;
	unsigned int i, s;

	for (i = 0; i < stitcher->num_dmabuf_slots; i++) {
		ds = &stitcher->dma_slots[i];

		for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
			if (ds->cached_fd[s] >= 0) {
				close_fd(ds->cached_fd[s]);
				ds->cached_fd[s] = -1;
			}
		}
	}
}

/**
 * stitcher_dmabuf_setup_slot - create dma_bufs for one output buffer.
 *
 * Splits the output buffer's pages into two halves (one per source).
 * sg_alloc_table_from_pages with offset handles non-page-aligned splits.
 */
static int stitcher_dmabuf_setup_slot(struct uvc_stitcher *stitcher,
				      struct stitcher_dmabuf_slot *ds,
				      struct vb2_buffer *out_vb,
				      unsigned int half_size)
{
	void *va;
	unsigned int frame_size = half_size * 2;
	unsigned int total_pages = PAGE_ALIGN(frame_size) >> PAGE_SHIFT;
	unsigned int a_num_pages, b_first_page, b_offset, b_num_pages;
	unsigned int i;
	int ret = 0;

	va = vb2_plane_vaddr(out_vb, 0);
	if (!va) {
		pr_err("uvc_stitcher: vb2_plane_vaddr NULL for out buf %d\n",
		       out_vb->index);
		return -EFAULT;
	}

	/* Collect vmalloc pages */
	ds->pages = kcalloc(total_pages, sizeof(struct page *), GFP_KERNEL);
	if (!ds->pages)
		return -ENOMEM;

	ds->num_pages = total_pages;

	for (i = 0; i < total_pages; i++) {
		ds->pages[i] = vmalloc_to_page((u8 *)va + (u64)i * PAGE_SIZE);
		if (!ds->pages[i]) {
			pr_err("uvc_stitcher: vmalloc_to_page NULL at page %u\n",
			       i);
			ret = -EFAULT;
			goto err_pages;
		}
	}

	/**
	 * Half A: bytes [0 .. half_size-1] → feed to src0
	 * Half B: bytes [half_size .. frame_size-1] → feed to src1
	 *
	 * When half_size is NOT page-aligned, page at the split point is
	 * shared between A and B.
     * sg_alloc_table_from_pages handles this via the offset parameter 
     * — DMA hardware operates at byte level.
	 */
	a_num_pages = (half_size + PAGE_SIZE - 1) >> PAGE_SHIFT;
	b_first_page = half_size >> PAGE_SHIFT;
	b_offset = half_size & (PAGE_SIZE - 1);
	b_num_pages = total_pages - b_first_page;

	/* Verify cache-line alignment of split point within shared page */
	if (b_offset && (b_offset % L1_CACHE_BYTES))
		pr_warn("uvc_stitcher: split offset %u not cache-line aligned (line=%u)\n",
			b_offset, L1_CACHE_BYTES);

	/* dma_buf for src0 (first half) */
	ds->dbuf[0] =
		stitcher_create_dmabuf(ds->pages, a_num_pages, 0, half_size);
	if (IS_ERR(ds->dbuf[0])) {
		ret = PTR_ERR(ds->dbuf[0]);
		ds->dbuf[0] = NULL;
		goto err_pages;
	}

	/* dma_buf for src1 (second half) */
	ds->dbuf[1] = stitcher_create_dmabuf(&ds->pages[b_first_page],
					     b_num_pages, b_offset,
					     frame_size - half_size);
	if (IS_ERR(ds->dbuf[1])) {
		ret = PTR_ERR(ds->dbuf[1]);
		ds->dbuf[1] = NULL;
		goto err_dbuf0;
	}

	ds->created = true;

	/* Initialize cached fds as invalid (created later in kthread) */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		ds->cached_fd[i] = -1;

	pr_debug("uvc_stitcher: dmabuf slot buf%d: A=%u pages off=0 len=%u, "
		 "B=%u pages off=%u len=%u\n",
		 out_vb->index, a_num_pages, half_size, b_num_pages, b_offset,
		 frame_size - half_size);

	return 0;

err_dbuf0:
	dma_buf_put(ds->dbuf[0]);
	ds->dbuf[0] = NULL;
err_pages:
	kfree(ds->pages);
	ds->pages = NULL;
	return ret;
}

static void stitcher_dmabuf_teardown_slot(struct stitcher_dmabuf_slot *ds)
{
	int i;

	if (!ds->created)
		return;

	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		if (ds->dbuf[i]) {
			dma_buf_put(ds->dbuf[i]);
			ds->dbuf[i] = NULL;
		}
	}

	kfree(ds->pages);
	ds->pages = NULL;
	ds->created = false;
}

static void stitcher_dmabuf_teardown_all(struct uvc_stitcher *stitcher)
{
	unsigned int i;

	for (i = 0; i < stitcher->num_dmabuf_slots; i++)
		stitcher_dmabuf_teardown_slot(&stitcher->dma_slots[i]);
	stitcher->num_dmabuf_slots = 0;
}

/**
 * stitcher_dmabuf_check_support - check if both UVC sources support DMABUF import.
 */
static bool stitcher_dmabuf_check_support(struct uvc_stitcher *stitcher)
{
	struct vb2_queue *vbq;
	int i;

	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		vbq = stitcher->src[i].vbq;
		if (!vbq) {
			pr_info("uvc_stitcher: src%d vbq NULL, DMABUF not available\n",
				i);
			return false;
		}
		if (!(vbq->io_modes & VB2_DMABUF)) {
			pr_info("uvc_stitcher: src%d lacks VB2_DMABUF, falling back to memcpy\n",
				i);
			return false;
		}
	}
	return true;
}

/* ------------------------------------------------------------------ */
/* Accessor vb2 helper                                                */
/* ------------------------------------------------------------------ */
/**
 * UVC vb2_queue accessor — see previous commit for full rationale.
 * Scans the opaque uvc_streaming allocation for a vb2_queue whose
 * drv_priv == &self (invariant set by uvc_queue_init).
 */
#define UVC_STREAM_SCAN_BYTES 8192UL

static struct vb2_queue *stitcher_scan_for_uvc_vbq(void *stream_base)
{
	char *p = (char *)stream_base;
	char *end = p + UVC_STREAM_SCAN_BYTES - sizeof(struct vb2_queue);
	struct vb2_queue *candidate;

	for (; p <= end; p += sizeof(void *)) {
		candidate = (struct vb2_queue *)p;
		if (candidate->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			continue;
		if (candidate->drv_priv != (void *)candidate)
			continue;
		if (!candidate->ops || !candidate->mem_ops)
			continue;
		pr_info("uvc_stitcher: found uvc vb2_queue at offset +%td\n",
			p - (char *)stream_base);
		return candidate;
	}
	return NULL;
}

static struct vb2_queue *stitcher_get_uvc_vbq(struct stitcher_slot *src)
{
	void *stream;
	struct vb2_queue *vbq;

	if (!src->vdev)
		return NULL;

	/* Fast path: vdev->queue populated by newer UVC / standard V4L2 */
	vbq = src->vdev->queue;
	if (vbq)
		return vbq;

	/* Slow path: scan uvc_streaming via video_drvdata */
	stream = video_get_drvdata(src->vdev);
	if (!stream) {
		pr_err("uvc_stitcher: video_get_drvdata NULL\n");
		return NULL;
	}
	vbq = stitcher_scan_for_uvc_vbq(stream);
	if (!vbq)
		pr_err("uvc_stitcher: vb2_queue not found in uvc_streaming\n");
	return vbq;
}

/**
 * stitcher_disc_work_fn - workqueue function for device discovery/removal.
 * 
 * Workqueue function for handling device discovery/removal asynchronously,
 * to avoid blocking the caller (e.g., uvcvideo) and to ensure proper locking.
 */
static void stitcher_disc_work_fn(struct work_struct *w)
{
	struct stitcher_disc_work *dw =
		container_of(w, struct stitcher_disc_work, work);
	struct uvc_stitcher *stitcher = dw->stitcher;
	struct video_device *vdev = dw->vdev;
	struct device *intf_dev;
	struct usb_interface *intf;
	struct usb_device *udev;
	u16 pid;
	char devpath[32];
	struct file *filp;
	int slot = -1;
	int i;

	if (dw->is_add) {
		intf_dev = vdev->v4l2_dev->dev;
		intf = to_usb_interface(intf_dev);
		udev = interface_to_usbdev(intf);
		pid = le16_to_cpu(udev->descriptor.idProduct);

		mutex_lock(&stitcher->src_disc_lock);
		if (stitcher->src_ready_count >= STITCHER_NUM_SOURCES) {
			mutex_unlock(&stitcher->src_disc_lock);
			goto out;
		}

		if (pid == STITCHER_DEFAULT_PID_P) {
			if (!stitcher->src[0].filp)
				slot = 0;
		} else if (pid == STITCHER_DEFAULT_PID_S) {
			if (!stitcher->src[1].filp)
				slot = 1;
		}

		if (slot < 0) {
			mutex_unlock(&stitcher->src_disc_lock);
			goto out;
		}

		/* Placeholder to mark slot as taken */
		stitcher->src[slot].filp = ERR_PTR(-EBUSY);
		stitcher->src_ready_count++;
		mutex_unlock(&stitcher->src_disc_lock);

		/* filp_open - Open without O_NONBLOCK. */
		snprintf(devpath, sizeof(devpath), "/dev/video%d", vdev->num);
		filp = filp_open(devpath, O_RDWR, 0);
		if (IS_ERR(filp)) {
			pr_err("uvc_stitcher: filp_open(%s): %ld\n", devpath,
			       PTR_ERR(filp));
			mutex_lock(&stitcher->src_disc_lock);
			stitcher->src[slot].filp = NULL;
			stitcher->src_ready_count--;
			mutex_unlock(&stitcher->src_disc_lock);
			goto out;
		}

		mutex_lock(&stitcher->src_disc_lock);
		stitcher->src[slot].filp = filp;
		stitcher->src[slot].vdev = video_devdata(filp);
		stitcher->src[slot].fh = filp->private_data;
		stitcher->src[slot].vbq =
			stitcher_get_uvc_vbq(&stitcher->src[slot]);
		if (!stitcher->src[slot].vbq) {
			pr_err("uvc_stitcher: slot%d: cannot locate vb2_queue\n",
			       slot);
			filp_close(filp, NULL);
			stitcher->src[slot].filp = NULL;
			stitcher->src[slot].vdev = NULL;
			stitcher->src[slot].fh = NULL;
			stitcher->src_ready_count--;
			mutex_unlock(&stitcher->src_disc_lock);
			goto out;
		}
		stitcher->src[slot].intf =
			to_usb_interface(vdev->v4l2_dev->dev);
		stitcher->src[slot].udev =
			interface_to_usbdev(stitcher->src[slot].intf);

		pr_info("uvc_stitcher: slot%d filled (%s), ready=%d/%d\n", slot,
			devpath, stitcher->src_ready_count,
			STITCHER_NUM_SOURCES);
		mutex_unlock(&stitcher->src_disc_lock);
	} else {
		if (atomic_read(&stitcher->streaming))
			stitcher_uvc_stop(stitcher);

		mutex_lock(&stitcher->src_disc_lock);
		for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
			if (stitcher->src[i].vdev == vdev) {
				filp_close(stitcher->src[i].filp, NULL);
				memset(&stitcher->src[i], 0,
				       sizeof(stitcher->src[i]));
				stitcher->src_ready_count--;
				pr_info("uvc_stitcher: slot%d removed, ready=%d/%d\n",
					i, stitcher->src_ready_count,
					STITCHER_NUM_SOURCES);
				break;
			}
		}
		mutex_unlock(&stitcher->src_disc_lock);
	}

out:
	put_device(&vdev->dev);
	kfree(dw);
}

/**
 * stitcher_match_device -  return true if vdev is one of target devices.
 * 
 * Discover UVC devices matching the specified VID/PID,
 * and add them as sources to the stitcher.
 */
static bool stitcher_match_device(struct video_device *vdev)
{
	struct device *intf_dev;
	struct usb_interface *intf;
	struct usb_device *udev;
	u16 pid;

	if (!(vdev->device_caps & V4L2_CAP_VIDEO_CAPTURE))
		return false;

	if (!vdev->v4l2_dev || !vdev->v4l2_dev->dev)
		return false;

	intf_dev = vdev->v4l2_dev->dev;

	if (!intf_dev->bus || strcmp(intf_dev->bus->name, "usb") != 0)
		return false;
	if (!intf_dev->driver ||
	    strcmp(intf_dev->driver->name, "uvcvideo") != 0)
		return false;

	intf = to_usb_interface(intf_dev);
	udev = interface_to_usbdev(intf);

	if (le16_to_cpu(udev->descriptor.idVendor) != STITCHER_DEFAULT_VID) {
		pr_info("uvc_stitcher: ignoring USB device with VID=0x%04X\n",
			le16_to_cpu(udev->descriptor.idVendor));
		return false;
	}

	pid = le16_to_cpu(udev->descriptor.idProduct);
	if (pid != STITCHER_DEFAULT_PID_P && pid != STITCHER_DEFAULT_PID_S) {
		pr_info("uvc_stitcher: ignoring USB device with PID=0x%04X\n",
			pid);
		return false;
	}

	return true;
}

static int stitcher_schedule_disc_work(struct uvc_stitcher *stitcher,
				       struct video_device *vdev, bool is_add)
{
	struct stitcher_disc_work *dw;

	dw = kzalloc(sizeof(*dw), GFP_ATOMIC);
	if (!dw)
		return -ENOMEM;

	get_device(&vdev->dev);
	INIT_WORK(&dw->work, stitcher_disc_work_fn);
	dw->stitcher = stitcher;
	dw->vdev = vdev;
	dw->is_add = is_add;

	queue_work(stitcher->disc_wq, &dw->work);
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
static int stitcher_class_add_dev(struct device *dev,
				  struct class_interface *ci)
#else
static int stitcher_class_add_dev(struct device *dev)
#endif
{
	struct uvc_stitcher *stitcher = g_stitcher;
	struct video_device *vdev = to_video_device(dev);

	if (!vdev || vdev == stitcher->vdev)
		return 0;

	if (!stitcher_match_device(vdev))
		return 0;

	return stitcher_schedule_disc_work(stitcher, vdev, true);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
static void stitcher_class_remove_dev(struct device *dev,
				      struct class_interface *ci)
#else
static void stitcher_class_remove_dev(struct device *dev)
#endif
{
	struct uvc_stitcher *stitcher = g_stitcher;
	struct video_device *vdev = to_video_device(dev);
	bool is_ours = false;
	int i;

	if (!vdev)
		return;

	mutex_lock(&stitcher->src_disc_lock);
	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		if (stitcher->src[i].vdev == vdev) {
			is_ours = true;
			break;
		}
	}
	mutex_unlock(&stitcher->src_disc_lock);

	if (!is_ours)
		return;

	stitcher_schedule_disc_work(stitcher, vdev, false);
}

static int stitcher_register_class_intf(struct uvc_stitcher *stitcher,
					STITCHER_CLASS_CONST struct class *vcls)
{
	stitcher->class_intf.class = vcls;
	stitcher->class_intf.add_dev = stitcher_class_add_dev;
	stitcher->class_intf.remove_dev = stitcher_class_remove_dev;

	return class_interface_register(&stitcher->class_intf);
}

static void stitcher_unregister_class_intf(struct uvc_stitcher *stitcher)
{
	class_interface_unregister(&stitcher->class_intf);
}

/* ------------------------------------------------------------------ */
/* SRC UVC streaming control                                          */
/* ------------------------------------------------------------------ */
static int stitcher_uvc_check_slots(struct uvc_stitcher *stitcher)
{
	pr_info("uvc_stitcher: checking source slots...\n");
	mutex_lock(&stitcher->src_disc_lock);
	if (stitcher->src_ready_count < STITCHER_NUM_SOURCES) {
		pr_err("uvc_stitcher: not all sources ready: %d/%d\n",
		       stitcher->src_ready_count, STITCHER_NUM_SOURCES);
		mutex_unlock(&stitcher->src_disc_lock);
		return -ENODEV;
	}
	mutex_unlock(&stitcher->src_disc_lock);
	pr_info("uvc_stitcher: all source slots are filled, ready to start streaming\n");
	return 0;
}

static int stitcher_uvc_negotiate_src_fmt(struct uvc_stitcher *stitcher,
					  int slot)
{
	struct stitcher_slot *src = &stitcher->src[slot];
	int ret;

	src->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	src->fmt.fmt.pix.width = stitcher->src_width;
	src->fmt.fmt.pix.height = stitcher->src_height;
	src->fmt.fmt.pix.pixelformat = stitcher->out_pixfmt;
	src->fmt.fmt.pix.field = V4L2_FIELD_NONE;

	ret = UVC_CALL_OP(src, vidioc_s_fmt_vid_cap, src->filp, src->fh,
			  &src->fmt);
	if (ret) {
		pr_err("uvc_stitcher: vidioc_s_fmt_vid_cap failed for slot%d: %d\n",
		       slot, ret);
		return ret;
	}

	ret = UVC_CALL_OP(src, vidioc_g_fmt_vid_cap, src->filp, src->fh,
			  &src->fmt);
	if (ret) {
		pr_err("uvc_stitcher: vidioc_g_fmt_vid_cap failed for slot%d: %d\n",
		       slot, ret);
		return ret;
	}

	pr_info("uvc_stitcher: src%d format negotiated: %ux%u, fmt=0x%X\n",
		slot, src->fmt.fmt.pix.width, src->fmt.fmt.pix.height,
		src->fmt.fmt.pix.pixelformat);

	return 0;
}

/**
 * stitcher_uvc_negotiate_src_fps - set the frame interval on a UVC source.
 *
 * Called after s_fmt so the UVC driver knows the active resolution.
 * Uses stitcher->fps_num / fps_den chosen by userspace via s_parm.
 */
static int stitcher_uvc_negotiate_src_fps(struct uvc_stitcher *stitcher,
					  int slot)
{
	struct stitcher_slot *src = &stitcher->src[slot];
	struct v4l2_streamparm parm = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	};
	int ret;

	parm.parm.capture.timeperframe.numerator = stitcher->fps_num;
	parm.parm.capture.timeperframe.denominator = stitcher->fps_den;

	ret = UVC_CALL_OP(src, vidioc_s_parm, src->filp, src->fh, &parm);
	if (ret) {
		pr_info("uvc_stitcher: src%d s_parm failed: %d (non-fatal)\n",
			slot, ret);
		return 0;
	}

	pr_info("uvc_stitcher: src%d fps negotiated: %u/%u\n", slot,
		parm.parm.capture.timeperframe.denominator,
		parm.parm.capture.timeperframe.numerator);
	return 0;
}

static int stitcher_uvc_reqbufs_mmap(struct uvc_stitcher *stitcher, int idx,
				     unsigned int count)
{
	struct stitcher_slot *src = &stitcher->src[idx];
	struct v4l2_requestbuffers req = {
		.count = count,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
	};
	unsigned int i;
	int ret;

	ret = UVC_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &req);
	if (ret) {
		pr_err("uvc_stitcher: vidioc_reqbufs failed for slot%d: %d\n",
		       idx, ret);
		return ret;
	}
	if (req.count < 2) {
		pr_err("uvc_stitcher: slot%d only has %d buffers\n", idx,
		       req.count);
		return -ENOMEM;
	}

	for (i = 0; i < req.count; i++) {
		struct v4l2_buffer buf = {
			.index = i,
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.memory = V4L2_MEMORY_MMAP,
		};
		ret = UVC_CALL_OP(src, vidioc_qbuf, src->filp, src->fh, &buf);
		if (ret) {
			pr_err("uvc_stitcher: src%d QBUF[%u] failed: %d\n", idx,
			       i, ret);
			return ret;
		}
	}
	return 0;
}

/* REQBUFS for DMABUF mode (buffer slots only, no physical memory) */
static int stitcher_uvc_reqbufs_dmabuf(struct uvc_stitcher *stitcher, int idx,
				       unsigned int count)
{
	struct stitcher_slot *src = &stitcher->src[idx];
	struct v4l2_requestbuffers req = {
		.count = count,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	int ret;

	ret = UVC_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &req);
	if (ret)
		return ret;
	if (req.count < count) {
		pr_warn("uvc_stitcher: src%d REQBUFS(DMABUF) got %u/%u\n", idx,
			req.count, count);
		if (req.count < 2)
			return -ENOMEM;
	}
	return 0;
}

static int stitcher_uvc_start_pre_queue(struct uvc_stitcher *stitcher)
{
	unsigned long flags;
	struct stitcher_video_buffer *mbuf;
	unsigned int i, j;
	int ret;
	unsigned int prequeued = 0;

	spin_lock_irqsave(&stitcher->out_q.irqlock, flags);
	while (!list_empty(&stitcher->out_q.irqqueue)) {
		mbuf = list_first_entry(&stitcher->out_q.irqqueue,
					struct stitcher_video_buffer, queue);
		list_del(&mbuf->queue);
		spin_unlock_irqrestore(&stitcher->out_q.irqlock, flags);

		i = mbuf->buf.vb2_buf.index;
		for (j = 0; j < STITCHER_NUM_SOURCES; j++) {
			ret = stitcher_dmabuf_qbuf_with_tmpfd(
				stitcher, j, i, stitcher->dma_slots[i].dbuf[j]);
			if (ret)
				return ret;
		}
		prequeued++;

		spin_lock_irqsave(&stitcher->out_q.irqlock, flags);
	}
	spin_unlock_irqrestore(&stitcher->out_q.irqlock, flags);

	atomic_set(&stitcher->bufs_in_flight, prequeued);
	pr_info("uvc_stitcher: pre-queued %u buffers to UVC sources\n",
		prequeued);

	return 0;
}

static int stitcher_uvc_streamon(struct uvc_stitcher *stitcher, int idx)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = UVC_CALL_OP(&stitcher->src[idx], vidioc_streamon,
			      stitcher->src[idx].filp, stitcher->src[idx].fh,
			      type);
	if (ret)
		pr_err("uvc_stitcher: src%d STREAMON failed: %d\n", idx, ret);
	return ret;
}

/**
 * stitcher_uvc_streamoff_only - STREAMOFF without freeing buffers.
 * Used in the stop path to unblock kthread's DQBUF before kthread_stop.
 */
static void stitcher_uvc_streamoff_only(struct uvc_stitcher *stitcher, int idx)
{
	struct stitcher_slot *src = &stitcher->src[idx];
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (!stitcher_src_valid(src))
		return;
	UVC_CALL_OP(src, vidioc_streamoff, src->filp, src->fh, type);
}

/**
 * stitcher_uvc_free_bufs - REQBUFS(0) to free UVC buffers.
 * Must be called AFTER all held page references are released.
 */
static void stitcher_uvc_free_bufs(struct uvc_stitcher *stitcher, int idx)
{
	struct stitcher_slot *src = &stitcher->src[idx];
	u32 mem = stitcher->use_dmabuf ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
	struct v4l2_requestbuffers req = {
		.count = 0,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = mem,
	};

	if (!stitcher_src_valid(src))
		return;
	UVC_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &req);
}

static int stitcher_uvc_start(struct uvc_stitcher *stitcher)
{
	unsigned int out_count;
	unsigned int half_size;
	unsigned int i;
	int ret;

	/* Negotiate format and request buffers for each source */
	mutex_lock(&stitcher->uvc_ctrl_lock);

	/* Verify all sources are ready */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		if (!stitcher_src_valid(&stitcher->src[i])) {
			pr_err("uvc_stitcher: src%d is not ready (filp=%p, vdev=%p)\n",
			       i, stitcher->src[i].filp, stitcher->src[i].vdev);
			mutex_unlock(&stitcher->uvc_ctrl_lock);
			return -ENODEV;
		}
	}

	/*
	 * Resume the UVC sources and hold a runtime-PM ref for the whole
	 * streaming session.  On 6.17 uvcvideo no longer pins runtime PM
	 * across open(), so a device opened at discovery time can autosuspend
	 * before this point, making the PROBE control transfer in S_FMT fail
	 * with -EHOSTUNREACH (-113).  Released by stitcher_uvc_stop() on the
	 * success path or by err_out below on failure.
	 */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		ret = usb_autopm_get_interface(stitcher->src[i].intf);
		if (ret) {
			pr_err("uvc_stitcher: autopm_get src%d failed: %d\n",
			       i, ret);
			while (i--)
				usb_autopm_put_interface(stitcher->src[i].intf);
			mutex_unlock(&stitcher->uvc_ctrl_lock);
			return ret;
		}
	}

	/* Negotiate format and framerate */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		ret = stitcher_uvc_negotiate_src_fmt(stitcher, i);
		if (ret) {
			pr_err("uvc_stitcher: failed to negotiate format for src%d\n",
			       i);
			goto err_out;
		}
		stitcher_uvc_negotiate_src_fps(stitcher, i);
	}

	/* Determine DMABUF mode at runtime */
	stitcher->use_dmabuf = zerocopy &&
			       stitcher_dmabuf_check_support(stitcher);

	/* Count queued output buffers */
	out_count = vb2_get_num_buffers(&stitcher->out_q.vbq);
	if (out_count > STITCHER_MAX_OUT_BUFS)
		out_count = STITCHER_MAX_OUT_BUFS;

	half_size = stitcher_image_size(stitcher->out_width,
					stitcher->out_height,
					stitcher->out_pixfmt) /
		    2;

	if (stitcher->use_dmabuf) {
		pr_info("uvc_stitcher: using DMABUF feed-through (out_bufs=%u half=%u)\n",
			out_count, half_size);

		/* REQBUFS(DMABUF) for each UVC source */
		for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
			ret = stitcher_uvc_reqbufs_dmabuf(stitcher, i,
							  out_count);
			if (ret)
				goto err_out;
		}

		/* Create dma_bufs for all output buffers */
		stitcher->num_dmabuf_slots = out_count;
		for (i = 0; i < out_count; i++) {
			struct vb2_buffer *vb =
				vb2_get_buffer(&stitcher->out_q.vbq, i);
			if (!vb) {
				ret = -EINVAL;
				goto err_dmabuf;
			}
			stitcher->out_vb_map[i] = vb;

			ret = stitcher_dmabuf_setup_slot(
				stitcher, &stitcher->dma_slots[i], vb,
				half_size);
			if (ret)
				goto err_dmabuf;
		}

		/*
		 * Pre-queue all output buffers to UVC sources.
		 * Take them from irqqueue (queued by userspace) and submit.
		 */
		ret = stitcher_uvc_start_pre_queue(stitcher);
		if (ret)
			goto err_dmabuf;
	} else {
		pr_info("uvc_stitcher: using memcpy fallback\n");

		for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
			ret = stitcher_uvc_reqbufs_mmap(stitcher, i,
							STITCHER_OUT_NUM_BUFS);
			if (ret)
				goto err_out;
		}
	}

	/* STREAMON each source */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		ret = stitcher_uvc_streamon(stitcher, i);
		if (ret) {
			pr_err("uvc_stitcher: failed to stream on src%d\n", i);
			goto err_streamoff;
		}
	}

	/* Start kthread */
	atomic_set(&stitcher->streaming, 1);
	stitcher->stitcher_task =
		kthread_run(stitcher_thread_fn, stitcher, "uvc_stitcher");
	if (IS_ERR(stitcher->stitcher_task)) {
		ret = PTR_ERR(stitcher->stitcher_task);
		stitcher->stitcher_task = NULL;
		atomic_set(&stitcher->streaming, 0);
		goto err_streamoff;
	}

	mutex_unlock(&stitcher->uvc_ctrl_lock);
	return 0;

err_streamoff:
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		stitcher_uvc_streamoff_only(stitcher, i);
err_dmabuf:
	if (stitcher->use_dmabuf)
		stitcher_dmabuf_teardown_all(stitcher);
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		stitcher_uvc_free_bufs(stitcher, i);
err_out:
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		if (stitcher->src[i].intf)
			usb_autopm_put_interface(stitcher->src[i].intf);
	mutex_unlock(&stitcher->uvc_ctrl_lock);
	return ret;
}

static void stitcher_uvc_stop(struct uvc_stitcher *stitcher)
{
	int i;

	if (!atomic_cmpxchg(&stitcher->streaming, 1, 0))
		return;

	pr_info("uvc_stitcher: stopping pipeline\n");

	/* STREAMOFF — unblocks kthread's blocking DQBUF */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		stitcher_uvc_streamoff_only(stitcher, i);

	/* wake and stop kthread */
	wake_up_all(&stitcher->out_q.buf_wq);
	if (stitcher->stitcher_task) {
		kthread_stop(stitcher->stitcher_task);
		stitcher->stitcher_task = NULL;
	}

	/**
	 * REQBUFS(0) — releases UVC buffer slots.
	 * For DMABUF mode: vb2 detaches our dma_bufs (dma_buf_put).
	 */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		stitcher_uvc_free_bufs(stitcher, i);

	/* Destroy dma_bufs. */
	if (stitcher->use_dmabuf)
		stitcher_dmabuf_teardown_all(stitcher);

	/*
	 * Release the runtime-PM ref taken in stitcher_uvc_start().
	 * Reached only when streaming was 1 (cmpxchg above), i.e. after a
	 * successful get, so this is always balanced.
	 */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		if (stitcher->src[i].intf)
			usb_autopm_put_interface(stitcher->src[i].intf);

	pr_info("uvc_stitcher: pipeline stopped\n");
}

/* DQBUF for MMAP mode — full: saves cur_vb, timestamp */
static int stitcher_dqbuf_src_mmap(struct uvc_stitcher *stitcher, int slot)
{
	struct stitcher_slot *src = &stitcher->src[slot];
	struct v4l2_buffer buf = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
	};
	int ret;

	if (!stitcher_src_valid(src) || !src->vbq) {
		pr_err("uvc_stitcher: src%d invalid state (filp=%p, vdev=%p, vbq=%p)\n",
		       slot, src->filp, src->vdev, src->vbq);
		return -ENODEV;
	}

	ret = UVC_CALL_OP(src, vidioc_dqbuf, src->filp, src->fh, &buf);
	if (ret == -EINVAL && !atomic_read(&stitcher->streaming))
		return ret;
	if (ret) {
		pr_err("uvc_stitcher: src%d DQBUF failed: %d\n", slot, ret);
		return ret;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 8, 0)
	if (buf.index >= src->vbq->num_buffers) {
#else
	if (buf.index >= src->vbq->max_num_buffers) {
#endif
		pr_err("uvc_stitcher: src%d DQBUF got invalid buffer index %d\n",
		       slot, buf.index);
		return -EFAULT;
	}

	src->cur_vb = vb2_get_buffer(src->vbq, buf.index);
	if (!src->cur_vb) {
		pr_err("uvc_stitcher: src%d DQBUF got invalid buffer index %d\n",
		       slot, buf.index);
		return -EFAULT;
	}
	src->cur_ts = ns_to_ktime((u64)buf.timestamp.tv_sec * NSEC_PER_SEC +
				  (u64)buf.timestamp.tv_usec * NSEC_PER_USEC);
	src->buf_ready = true;

	return 0;
}

static void stitcher_qbuf_src_mmap(struct uvc_stitcher *stitcher, int slot)
{
	struct stitcher_slot *src = &stitcher->src[slot];
	struct v4l2_buffer buf = {
		.index = src->cur_vb->index,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
	};

	if (!stitcher_src_valid(src) || !src->cur_vb)
		return;

	UVC_CALL_OP(src, vidioc_qbuf, src->filp, src->fh, &buf);
	src->cur_vb = NULL;
	src->buf_ready = false;
}

/* Memcpy */
static void stitcher_stitch_memcpy(u8 *dst, const u8 *src0, const u8 *src1,
				   u64 raw_size)
{
	memcpy(dst, src0, raw_size);
	memcpy(dst + raw_size, src1, raw_size);
}

/**
 * stitcher_requeue_pair_dmabuf - re-QBUF both halves back to UVC sources.
 *
 * Used when a DQBUF pair must be discarded (index or sequence mismatch).
 */
static void stitcher_requeue_pair_dmabuf(struct uvc_stitcher *stitcher,
					 unsigned int idx0, unsigned int idx1)
{
	stitcher_qbuf_src_dmabuf(stitcher, 0, idx0,
				 stitcher->dma_slots[idx0].cached_fd[0]);
	stitcher_qbuf_src_dmabuf(stitcher, 1, idx1,
				 stitcher->dma_slots[idx1].cached_fd[1]);
}

/**
 * stitcher_requeue_outbuf - return an output buffer to irqqueue.
 *
 * Re-adds a stitcher_video_buffer to the tail of the output irqqueue so the
 * kthread can retry it on the next iteration.
 */
static void stitcher_requeue_outbuf(struct stitcher_video_queue *q,
				    struct stitcher_video_buffer *mbuf)
{
	unsigned long flags;

	spin_lock_irqsave(&q->irqlock, flags);
	list_add_tail(&mbuf->queue, &q->irqqueue);
	spin_unlock_irqrestore(&q->irqlock, flags);
}

/**
 * stitcher_dmabuf_resync_sources - STREAMOFF + re-QBUF + STREAMON both sources.
 *
 * Resets both UVC sources so their sequence counters restart at 0.
 * Used when the warm-up drain detects a sequence offset caused by an
 * asymmetric -EPROTO at initial STREAMON.
 *
 * Must be called from kthread context with cached FDs already created.
 */
static int stitcher_dmabuf_resync_sources(struct uvc_stitcher *stitcher)
{
	unsigned int i, s;
	int ret;

	/* STREAMOFF both — stops URBs, resets UVC state */
	for (s = 0; s < STITCHER_NUM_SOURCES; s++)
		stitcher_uvc_streamoff_only(stitcher, s);

	/* Re-QBUF all output buffers to both sources */
	for (i = 0; i < stitcher->num_dmabuf_slots; i++) {
		for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
			ret = stitcher_qbuf_src_dmabuf(
				stitcher, s, i,
				stitcher->dma_slots[i].cached_fd[s]);
			if (ret) {
				pr_err("uvc_stitcher: resync re-QBUF "
				       "slot %u src%u failed: %d\n",
				       i, s, ret);
				return ret;
			}
		}
	}

	/* STREAMON both — UVC resets sequence counter, submits URBs */
	for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
		ret = stitcher_uvc_streamon(stitcher, s);
		if (ret)
			return ret;
	}

	atomic_set(&stitcher->bufs_in_flight, stitcher->num_dmabuf_slots);
	return 0;
}

/**
 * stitcher_warmup_sync - drain initial frame pairs and check sequence numbers.
 * 
 * At STREAMON the 2 sources may start at different times (alt-setting switch latency)
 * and a transient -EPROTO can widen the gap.
 * 
 * fix:
 * - Drain N pairs of frames, then compare sequence numbers. 
 * - If the sequence don't match, attempt a resync and retry.
 */
static int stitcher_warmup_sync(struct uvc_stitcher *stitcher)
{
	unsigned int w_idx0, w_idx1;
	u32 w_seq0, w_seq1;
	int attempt, n, ret;

	for (attempt = 0; attempt <= STITCHER_SYNC_MAX_RETRIES; attempt++) {
		w_seq0 = 0;
		w_seq1 = 0;

		for (n = 0; n < STITCHER_SYNC_DRAIN_FRAMES; n++) {
			ret = stitcher_dqbuf_src_dmabuf(stitcher, 0, &w_idx0,
							&w_seq0, NULL);
			if (ret)
				return ret;

			ret = stitcher_dqbuf_src_dmabuf(stitcher, 1, &w_idx1,
							&w_seq1, NULL);
			if (ret) {
				stitcher_qbuf_src_dmabuf(
					stitcher, 0, w_idx0,
					stitcher->dma_slots[w_idx0]
						.cached_fd[0]);
				return ret;
			}

			stitcher_requeue_pair_dmabuf(stitcher, w_idx0, w_idx1);
		}

		if (w_seq0 == w_seq1) {
			pr_info("uvc_stitcher: synced at seq %u (attempt %d, drained %d)\n",
				w_seq0, attempt, STITCHER_SYNC_DRAIN_FRAMES);
			return 0;
		}

		if (attempt < STITCHER_SYNC_MAX_RETRIES) {
			pr_warn("uvc_stitcher: sequence mismatch after draining %d frames: "
				"src0 seq=%u vs src1 seq=%u (attempt %d)\n",
				STITCHER_SYNC_DRAIN_FRAMES, w_seq0, w_seq1,
				attempt);
			ret = stitcher_dmabuf_resync_sources(stitcher);
			if (ret) {
				pr_err("uvc_stitcher: failed to resync sources: %d\n",
				       ret);
				return ret;
			}
		}
	}

	pr_warn("uvc_stitcher: sync failed after %d retries (seq %u vs %u), proceeding\n",
		STITCHER_SYNC_MAX_RETRIES, w_seq0, w_seq1);
	return 0;
}

/* ------------------------------------------------------------------ */
/* kthread                                                            */
/* ------------------------------------------------------------------ */
/**
 * DMABUF feed-through path
 *
 * Output buffer pages are exported as dma_buf and pre-queued to UVC sources.
 * Kthread just waits for DMA completion, signals done, re-queues.
 */
static void stitcher_dma(struct uvc_stitcher *stitcher, u32 out_size)
{
	struct stitcher_video_buffer *mbuf;
	struct vb2_buffer *out_vb;
	struct vb2_v4l2_buffer *v4l2_vb;
	unsigned long flags;
	unsigned int idx0, idx1, offset;
	u32 seq0 = 0, seq1 = 0;
	ktime_t ts0 = 0, ts1 = 0;
	int i, ret, s;

	/* create cached fds in init_files */
	ret = stitcher_dmabuf_create_cached_fds(stitcher);
	if (ret) {
		pr_err("uvc_stitcher: failed to create cached fds: %d\n", ret);
		goto dmabuf_exit;
	}

	ret = stitcher_warmup_sync(stitcher);
	if (ret) {
		pr_err("uvc_stitcher: warm-up sync failed: %d\n", ret);
		goto dmabuf_exit;
	}

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(
			stitcher->out_q.buf_wq,
			atomic_read(&stitcher->bufs_in_flight) > 0 ||
				!list_empty(&stitcher->out_q.irqqueue) ||
				stitcher_should_stop(stitcher));

		if (stitcher_should_stop(stitcher))
			break;
		if (ret == -ERESTARTSYS)
			continue;

		/* Re-submit output buffers that userspace re-QBUF'd */
		spin_lock_irqsave(&stitcher->out_q.irqlock, flags);
		while (!list_empty(&stitcher->out_q.irqqueue)) {
			mbuf = list_first_entry(&stitcher->out_q.irqqueue,
						struct stitcher_video_buffer,
						queue);
			list_del(&mbuf->queue);
			spin_unlock_irqrestore(&stitcher->out_q.irqlock, flags);

			i = mbuf->buf.vb2_buf.index;
			for (s = 0; s < STITCHER_NUM_SOURCES; s++)
				stitcher_qbuf_src_dmabuf(
					stitcher, s, i,
					stitcher->dma_slots[i].cached_fd[s]);
			atomic_inc(&stitcher->bufs_in_flight);

			spin_lock_irqsave(&stitcher->out_q.irqlock, flags);
		}
		spin_unlock_irqrestore(&stitcher->out_q.irqlock, flags);

		if (!atomic_read(&stitcher->streaming))
			break;
		if (atomic_read(&stitcher->bufs_in_flight) <= 0)
			continue;

		/* DQBUF from both sources (blocking) */
		ret = stitcher_dqbuf_src_dmabuf(stitcher, 0, &idx0, &seq0,
						&ts0);
		if (ret) {
			if (stitcher_should_stop(stitcher) ||
			    stitcher_is_fatal_err(ret))
				break;
			continue;
		}

		ret = stitcher_dqbuf_src_dmabuf(stitcher, 1, &idx1, &seq1,
						&ts1);
		if (ret) {
			stitcher_qbuf_src_dmabuf(
				stitcher, 0, idx0,
				stitcher->dma_slots[idx0].cached_fd[0]);
			if (stitcher_should_stop(stitcher) ||
			    stitcher_is_fatal_err(ret))
				break;
			continue;
		}

		/* Discard mismatched pairs */
		if (idx0 != idx1) {
			pr_warn("uvc_stitcher: DQBUF index mismatch %u vs %u\n",
				idx0, idx1);
			stitcher_requeue_pair_dmabuf(stitcher, idx0, idx1);
			continue;
		}
		if (seq0 != seq1) {
            offset = (seq0 > seq1) ? (seq0 - seq1) : (seq1 - seq0);
            if (offset > STITCHER_SEQ_MISMATCH_THRESHOLD) {
                 pr_warn("uvc_stitcher: DQBUF sequence mismatch %u vs %u (offset %u), resyncing sources\n",
                         seq0, seq1, offset);
                 ret = stitcher_dmabuf_resync_sources(stitcher);
                 if (ret) {
                     pr_err("uvc_stitcher: failed to resync sources: %d\n", ret);
                     break;
                 }
                 continue;
            }

			pr_info_ratelimited(
				"uvc_stitcher: seq mismatch %u vs %u, "
				"offset %u\n",
				seq0, seq1, offset);
		}

		/* Output buffer idx0 is filled by DMA — signal done */
		if (idx0 >= stitcher->num_dmabuf_slots) {
			pr_err("uvc_stitcher: DQBUF index %u out of range\n",
			       idx0);
			break;
		}

		out_vb = stitcher->out_vb_map[idx0];
		if (!out_vb) {
			pr_err("uvc_stitcher: out_vb_map[%u] NULL\n", idx0);
			break;
		}
		v4l2_vb = to_vb2_v4l2_buffer(out_vb);
		vb2_set_plane_payload(out_vb, 0, out_size);

		out_vb->timestamp =
			ktime_to_ns(ktime_before(ts0, ts1) ? ts0 : ts1);
		v4l2_vb->sequence = seq0;
		vb2_buffer_done(out_vb, VB2_BUF_STATE_DONE);
		atomic_dec(&stitcher->bufs_in_flight);
	}

dmabuf_exit:
	/* Close cached fds in init_files before exiting */
	stitcher_dmabuf_close_cached_fds(stitcher);
}

static void stitcher_mmap(struct uvc_stitcher *stitcher, u32 out_size,
			  u64 half_size)
{
	struct stitcher_video_buffer *mbuf;
	struct vb2_buffer *out_vb;
	unsigned long flags;
	int ret;
	void *src0_va, *src1_va, *dst;

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(
			stitcher->out_q.buf_wq,
			!list_empty(&stitcher->out_q.irqqueue) ||
				stitcher_should_stop(stitcher));

		if (stitcher_should_stop(stitcher))
			break;
		if (ret == -ERESTARTSYS)
			continue;

		spin_lock_irqsave(&stitcher->out_q.irqlock, flags);
		if (list_empty(&stitcher->out_q.irqqueue)) {
			spin_unlock_irqrestore(&stitcher->out_q.irqlock, flags);
			continue;
		}
		mbuf = list_first_entry(&stitcher->out_q.irqqueue,
					struct stitcher_video_buffer, queue);
		list_del(&mbuf->queue);
		spin_unlock_irqrestore(&stitcher->out_q.irqlock, flags);

		out_vb = &mbuf->buf.vb2_buf;

		if (!atomic_read(&stitcher->streaming))
			goto return_buf;

		/* DQBUF from UVC sources */
		ret = stitcher_dqbuf_src_mmap(stitcher, 0);
		if (ret) {
			if (!atomic_read(&stitcher->streaming))
				goto return_buf;
			if (stitcher_is_fatal_err(ret)) {
				vb2_buffer_done(out_vb, VB2_BUF_STATE_ERROR);
				break;
			}
			stitcher_requeue_outbuf(&stitcher->out_q, mbuf);
			continue;
		}

		if (!atomic_read(&stitcher->streaming)) {
			stitcher_qbuf_src_mmap(stitcher, 0);
			goto return_buf;
		}

		ret = stitcher_dqbuf_src_mmap(stitcher, 1);
		if (ret) {
			stitcher_qbuf_src_mmap(stitcher, 0);
			if (!atomic_read(&stitcher->streaming))
				goto return_buf;
			if (stitcher_is_fatal_err(ret)) {
				vb2_buffer_done(out_vb, VB2_BUF_STATE_ERROR);
				break;
			}
			stitcher_requeue_outbuf(&stitcher->out_q, mbuf);
			continue;
		}

		src0_va = vb2_plane_vaddr(stitcher->src[0].cur_vb, 0);
		src1_va = vb2_plane_vaddr(stitcher->src[1].cur_vb, 0);
		dst = vb2_plane_vaddr(out_vb, 0);

		if (!dst || !src0_va || !src1_va) {
			stitcher_qbuf_src_mmap(stitcher, 0);
			stitcher_qbuf_src_mmap(stitcher, 1);
			vb2_buffer_done(out_vb, VB2_BUF_STATE_ERROR);
			continue;
		}

		stitcher_stitch_memcpy(dst, src0_va, src1_va, half_size);

		stitcher_qbuf_src_mmap(stitcher, 0);
		stitcher_qbuf_src_mmap(stitcher, 1);

		vb2_set_plane_payload(out_vb, 0, out_size);
		out_vb->timestamp = ktime_to_ns(stitcher->src[0].cur_ts);
		vb2_buffer_done(out_vb, VB2_BUF_STATE_DONE);
	}

return_buf:
	/*
	 * Shutdown path: return the output buffer to irqqueue so that
	 * stop_streaming -> stitcher_video_queue_return_buffers can finalize it.
	 * Do NOT call vb2_buffer_done here — stop_streaming handles that.
	 */
	spin_lock_irqsave(&stitcher->out_q.irqlock, flags);
	list_add(&mbuf->queue, &stitcher->out_q.irqqueue);
	spin_unlock_irqrestore(&stitcher->out_q.irqlock, flags);
	pr_info("uvc_stitcher: kthread shutdown — returned output buffer to queue\n");
}

static int stitcher_thread_fn(void *data)
{
	struct uvc_stitcher *stitcher = data;
	const bool dmabuf_mode = stitcher->use_dmabuf;
	u64 half_size;
	u32 pixfmt;
	u32 out_size;

	sched_set_fifo(current);

	mutex_lock(&stitcher->lock);
	pixfmt = stitcher->out_pixfmt;
	out_size = stitcher_image_size(stitcher->out_width,
				       stitcher->out_height, pixfmt);
	mutex_unlock(&stitcher->lock);
	half_size = (u64)out_size / 2;

	if (dmabuf_mode) {
		stitcher_dma(stitcher, out_size);
	} else {
		stitcher_mmap(stitcher, out_size, half_size);
	}

	pr_info("uvc_stitcher: kthread exiting\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/**
 * vb2 queue operations
 */
static void stitcher_video_queue_return_buffers(struct stitcher_video_queue *q,
						enum vb2_buffer_state state)
{
	while (!list_empty(&q->irqqueue)) {
		struct stitcher_video_buffer *mbuf = list_first_entry(
			&q->irqqueue, struct stitcher_video_buffer, queue);
		list_del(&mbuf->queue);
		vb2_buffer_done(&mbuf->buf.vb2_buf, state);
	}
}

static int stitcher_queue_setup(struct vb2_queue *vbq,
				unsigned int *num_buffers,
				unsigned int *num_planes, unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct stitcher_video_queue *q = vb2_get_drv_priv(vbq);
	struct uvc_stitcher *stitcher =
		container_of(q, struct uvc_stitcher, out_q);
	unsigned int size;

	mutex_lock(&stitcher->lock);
	size = stitcher_image_size(stitcher->out_width, stitcher->out_height,
				   stitcher->out_pixfmt);
	mutex_unlock(&stitcher->lock);

	pr_debug("uvc_stitcher: queue_setup (nplanes=%d, nbuffers=%d)\n",
		 *num_planes, *num_buffers);
	if (*num_planes)
		return sizes[0] < size ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = size;
	*num_buffers =
		clamp_t(unsigned int, *num_buffers, 2, STITCHER_MAX_OUT_BUFS);

	pr_info("uvc_stitcher: queue_setup size=%u buffers=%u\n", size,
		*num_buffers);
	return 0;
}

static int stitcher_buf_prepare(struct vb2_buffer *vb)
{
	struct stitcher_video_queue *q = vb2_get_drv_priv(vb->vb2_queue);
	struct uvc_stitcher *stitcher =
		container_of(q, struct uvc_stitcher, out_q);
	unsigned int size;

	mutex_lock(&stitcher->lock);
	size = stitcher_image_size(stitcher->out_width, stitcher->out_height,
				   stitcher->out_pixfmt);
	mutex_unlock(&stitcher->lock);

	if (vb2_plane_size(vb, 0) < size) {
		pr_err("uvc_stitcher: buffer plane size < required %d\n", size);
		return -EINVAL;
	}

	return 0;
}

static void stitcher_buf_queue(struct vb2_buffer *vb)
{
	struct stitcher_video_queue *q = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct stitcher_video_buffer *mbuf =
		container_of(vbuf, struct stitcher_video_buffer, buf);
	unsigned long flags;

	pr_debug("uvc_stitcher: buf_queue idx=%d\n", vb->index);

	spin_lock_irqsave(&q->irqlock, flags);
	list_add_tail(&mbuf->queue, &q->irqqueue);
	spin_unlock_irqrestore(&q->irqlock, flags);

	wake_up_interruptible(&q->buf_wq);
}

static int stitcher_start_streaming(struct vb2_queue *vbq, unsigned int count)
{
	struct stitcher_video_queue *q = vb2_get_drv_priv(vbq);
	struct uvc_stitcher *stitcher =
		container_of(q, struct uvc_stitcher, out_q);
	unsigned long flags;
	int ret;

	pr_info("uvc_stitcher: start_streaming called with %u buffers\n",
		count);
	lockdep_assert_irqs_enabled();

	ret = stitcher_uvc_start(stitcher);
	if (ret) {
		unsigned int i, num;

		pr_err("uvc_stitcher: uvc_start failed: %d, returning buffers\n",
		       ret);

		/* Return buffers still in irqqueue */
		spin_lock_irqsave(&q->irqlock, flags);
		stitcher_video_queue_return_buffers(q, VB2_BUF_STATE_QUEUED);
		spin_unlock_irqrestore(&q->irqlock, flags);

		/**
		 * DMABUF mode: some buffers may have been taken from
		 * irqqueue and submitted to UVC before the error.
		 * UVC free_bufs already ran, but our vb2 still holds
		 * them as ACTIVE.  Return them too.
		 */
		num = vb2_get_num_buffers(vbq);
		for (i = 0; i < num; i++) {
			struct vb2_buffer *vb = vb2_get_buffer(vbq, i);

			if (vb && vb->state == VB2_BUF_STATE_ACTIVE)
				vb2_buffer_done(vb, VB2_BUF_STATE_QUEUED);
		}

		return ret;
	}

	pr_info("uvc_stitcher: streaming started successfully\n");
	return 0;
}

static void stitcher_stop_streaming(struct vb2_queue *vbq)
{
	struct stitcher_video_queue *q = vb2_get_drv_priv(vbq);
	struct uvc_stitcher *stitcher =
		container_of(q, struct uvc_stitcher, out_q);
	unsigned long flags;
	unsigned int i, num;
	int s;

	pr_info("uvc_stitcher: stop_streaming called\n");
	lockdep_assert_irqs_enabled();

	stitcher_uvc_stop(stitcher);

	/* Return any buffers still in our irqqueue (memcpy path mainly) */
	spin_lock_irqsave(&q->irqlock, flags);
	stitcher_video_queue_return_buffers(q, VB2_BUF_STATE_ERROR);
	spin_unlock_irqrestore(&q->irqlock, flags);

	/**
	 * Return ALL remaining active buffers.
	 *
	 * In DMABUF mode, output buffers are submitted to UVC sources
	 * and removed from irqqueue.  After stitcher_uvc_stop(), UVC has
	 * released them (STREAMOFF + REQBUFS 0), but our vb2 queue
	 * still considers them ACTIVE.
	 *
	 * vb2 requires stop_streaming to call vb2_buffer_done() for
	 * every active buffer; otherwise it emits:
	 *   "driver bug: stop_streaming leaving buffer N in active state"
	 */
	num = vb2_get_num_buffers(vbq);
	for (i = 0; i < num; i++) {
		struct vb2_buffer *vb = vb2_get_buffer(vbq, i);

		if (vb && vb->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}

	for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
		stitcher->src[s].cur_vb = NULL;
		stitcher->src[s].buf_ready = false;
	}
	pr_info("uvc_stitcher: stop_streaming completed\n");
}

const struct vb2_ops stitcher_vb2_ops = {
	// clang-format off
    .queue_setup     = stitcher_queue_setup,
	.buf_prepare     = stitcher_buf_prepare,
	.buf_queue       = stitcher_buf_queue,
	.start_streaming = stitcher_start_streaming,
	.stop_streaming  = stitcher_stop_streaming,
	/*
	 * Since commit 88785982a19d ("media: vb2: use lock if
	 * wait_prepare/finish are NULL") vb2 drops/retakes q->lock itself
	 * while sleeping in DQBUF; the ops were later removed from
	 * struct vb2_ops.  We always set q->lock, so they are only needed
	 * on older kernels.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 13, 0)
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
#endif
	// clang-format on
};

static void stitcher_queue_release(struct stitcher_video_queue *q)
{
	vb2_queue_release(&q->vbq);
}

static int init_queue(struct stitcher_video_queue *q)
{
	mutex_init(&q->mutex);
	spin_lock_init(&q->irqlock);
	init_waitqueue_head(&q->buf_wq);
	INIT_LIST_HEAD(&q->irqqueue);

	q->vbq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->vbq.io_modes = VB2_MMAP | VB2_READ;
	q->vbq.drv_priv = q;
	q->vbq.ops = &stitcher_vb2_ops;
	q->vbq.mem_ops = &vb2_vmalloc_memops;
	q->vbq.buf_struct_size = sizeof(struct stitcher_video_buffer);
	q->vbq.lock = &q->mutex;
	q->vbq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	return vb2_queue_init(&q->vbq);
}

/* ------------------------------------------------------------------ */
/* vidioc_streamon */
static int stitcher_vidioc_streamon(struct file *file, void *fh,
				    enum v4l2_buf_type type)
{
	struct uvc_stitcher *stitcher = video_drvdata(file);
	int ret;

	pr_info("uvc_stitcher: vidioc_streamon called (type=%d)\n", type);
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("uvc_stitcher: invalid buffer type %d\n", type);
		return -EINVAL;
	}

	ret = stitcher_uvc_check_slots(stitcher);
	if (ret) {
		pr_err("uvc_stitcher: slot check failed: %d\n", ret);
		return ret;
	}

	return vb2_ioctl_streamon(file, fh, type);
}

/**
 * vidioc_streamoff
 * Stop kthread and source streaming
 * then call vb2_ioctl_streamoff to complete the stream off process
 */
static int stitcher_vidioc_streamoff(struct file *file, void *fh,
				     enum v4l2_buf_type type)
{
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return vb2_ioctl_streamoff(file, fh, type);
}

static int stitcher_vidioc_querycap(struct file *file, void *fh,
				    struct v4l2_capability *cap)
{
	strscpy(cap->driver, "UVC Stitcher", sizeof(cap->driver));
	strscpy(cap->card, "UVC Stitcher", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:uvc_stitcher", sizeof(cap->bus_info));
	return 0;
}

static const u32 stitcher_formats[] = {
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_NV12,
};
#define MIXER_NUM_FORMATS ARRAY_SIZE(stitcher_formats)

static int stitcher_vidioc_enum_fmt(struct file *file, void *fh,
				    struct v4l2_fmtdesc *fmtdesc)
{
	if (fmtdesc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (fmtdesc->index >= MIXER_NUM_FORMATS)
		return -EINVAL;

	fmtdesc->pixelformat = stitcher_formats[fmtdesc->index];
	return 0;
}

static void fill_fmt(struct uvc_stitcher *stitcher, struct v4l2_format *f)
{
	mutex_lock(&stitcher->lock);
	f->fmt.pix.width = stitcher->out_width;
	f->fmt.pix.height = stitcher->out_height;
	f->fmt.pix.pixelformat = stitcher->out_pixfmt;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline = stitcher_bytesperline(stitcher->out_width,
							stitcher->out_pixfmt);
	f->fmt.pix.sizeimage = stitcher_image_size(stitcher->out_width,
						   stitcher->out_height,
						   stitcher->out_pixfmt);
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	mutex_unlock(&stitcher->lock);
}

static int stitcher_vidioc_g_fmt(struct file *file, void *fh,
				 struct v4l2_format *fmt)
{
	struct uvc_stitcher *stitcher = video_drvdata(file);

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	fill_fmt(stitcher, fmt);
	return 0;
}

static int stitcher_vidioc_s_fmt(struct file *file, void *fh,
				 struct v4l2_format *fmt)
{
	struct uvc_stitcher *stitcher = video_drvdata(file);

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Accept YUYV or NV12.  Anything else falls back to NV12. */
	mutex_lock(&stitcher->lock);
	stitcher->out_pixfmt = stitcher_pixfmt_valid(fmt->fmt.pix.pixelformat) ?
				       fmt->fmt.pix.pixelformat :
				       V4L2_PIX_FMT_NV12;

	if (fmt->fmt.pix.width > 0 && fmt->fmt.pix.height >= 2) {
		stitcher->out_width = fmt->fmt.pix.width;
		stitcher->out_height = fmt->fmt.pix.height &
				       ~1u; /* round down to even */
		stitcher->src_width = stitcher->out_width;
		stitcher->src_height = stitcher->out_height / 2;
		pr_info("uvc_stitcher: s_fmt accepted %ux%u fmt=0x%X (src %ux%u)\n",
			stitcher->out_width, stitcher->out_height,
			stitcher->out_pixfmt, stitcher->src_width,
			stitcher->src_height);
	}
	mutex_unlock(&stitcher->lock);

	/* Return the (possibly adjusted) format back to userspace */
	fill_fmt(stitcher, fmt);
	return 0;
}

static int stitcher_vidioc_try_fmt(struct file *file, void *fh,
				   struct v4l2_format *fmt)
{
	struct uvc_stitcher *stitcher = video_drvdata(file);
	u32 pixfmt;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	pixfmt = stitcher_pixfmt_valid(fmt->fmt.pix.pixelformat) ?
			 fmt->fmt.pix.pixelformat :
			 V4L2_PIX_FMT_NV12;

	fmt->fmt.pix.pixelformat = pixfmt;
	fmt->fmt.pix.field = V4L2_FIELD_NONE;
	fmt->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

	/* Accept proposed size if valid, otherwise return current */
	if (fmt->fmt.pix.width == 0 || fmt->fmt.pix.height < 2) {
		mutex_lock(&stitcher->lock);
		fmt->fmt.pix.width = stitcher->out_width;
		fmt->fmt.pix.height = stitcher->out_height;
		mutex_unlock(&stitcher->lock);
	} else {
		fmt->fmt.pix.height &= ~1u; /* round down to even */
	}

	fmt->fmt.pix.bytesperline =
		stitcher_bytesperline(fmt->fmt.pix.width, pixfmt);
	fmt->fmt.pix.sizeimage = stitcher_image_size(
		fmt->fmt.pix.width, fmt->fmt.pix.height, pixfmt);
	return 0;
}

static int stitcher_vidioc_s_input(struct file *file, void *fh,
				   unsigned int input)
{
	return input ? -EINVAL : 0;
}

static int stitcher_vidioc_g_input(struct file *file, void *fh,
				   unsigned int *input)
{
	*input = 0;
	return 0;
}

static int stitcher_vidioc_enum_input(struct file *file, void *fh,
				      struct v4l2_input *input)
{
	if (input->index != 0)
		return -EINVAL;

	strscpy(input->name, "Mixed Input", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;
	return 0;
}

static int stitcher_vidioc_g_parm(struct file *file, void *fh,
				  struct v4l2_streamparm *a)
{
	struct uvc_stitcher *stitcher = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(&a->parm.capture, 0, sizeof(a->parm.capture));
	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;

	mutex_lock(&stitcher->lock);
	a->parm.capture.timeperframe.numerator = stitcher->fps_num;
	a->parm.capture.timeperframe.denominator = stitcher->fps_den;
	mutex_unlock(&stitcher->lock);

	return 0;
}

static int stitcher_vidioc_s_parm(struct file *file, void *fh,
				  struct v4l2_streamparm *a)
{
	struct uvc_stitcher *stitcher = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	mutex_lock(&stitcher->lock);
	if (a->parm.capture.timeperframe.numerator &&
	    a->parm.capture.timeperframe.denominator) {
		stitcher->fps_num = a->parm.capture.timeperframe.numerator;
		stitcher->fps_den = a->parm.capture.timeperframe.denominator;
		pr_info("uvc_stitcher: s_parm fps=%u/%u\n", stitcher->fps_den,
			stitcher->fps_num);
	}
	mutex_unlock(&stitcher->lock);

	/* Read back what we stored */
	return stitcher_vidioc_g_parm(file, fh, a);
}

static int stitcher_vidioc_enum_framesizes(struct file *file, void *fh,
					   struct v4l2_frmsizeenum *fsize)
{
	struct uvc_stitcher *stitcher = video_drvdata(file);
	struct stitcher_slot *src;
	int ret;

	if (!stitcher_pixfmt_valid(fsize->pixel_format))
		return -EINVAL;

	mutex_lock(&stitcher->src_disc_lock);
	if (stitcher->src_ready_count < 1 ||
	    !stitcher_src_valid(&stitcher->src[0])) {
		mutex_unlock(&stitcher->src_disc_lock);
		return -ENODEV;
	}
	src = &stitcher->src[0];

	ret = UVC_CALL_OP(src, vidioc_enum_framesizes, src->filp, src->fh,
			  fsize);
	mutex_unlock(&stitcher->src_disc_lock);

	if (ret)
		return ret;

	/**
     * The UVC driver returns DISCRETE entries.  Double the height
     * to represent the stitched output.
     */
	if (fsize->type == V4L2_FRMSIZE_TYPE_DISCRETE) {
		fsize->discrete.height *= 2;
	} else if (fsize->type == V4L2_FRMSIZE_TYPE_STEPWISE ||
		   fsize->type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
		fsize->stepwise.min_height *= 2;
		fsize->stepwise.max_height *= 2;
		fsize->stepwise.step_height *= 2;
	}

	return 0;
}

static int stitcher_vidioc_enum_frameintervals(struct file *file, void *fh,
					       struct v4l2_frmivalenum *fival)
{
	struct uvc_stitcher *stitcher = video_drvdata(file);
	struct stitcher_slot *src;
	int ret;

	if (!stitcher_pixfmt_valid(fival->pixel_format))
		return -EINVAL;

	mutex_lock(&stitcher->src_disc_lock);
	if (stitcher->src_ready_count < 1 ||
	    !stitcher_src_valid(&stitcher->src[0])) {
		mutex_unlock(&stitcher->src_disc_lock);
		return -ENODEV;
	}
	src = &stitcher->src[0];

	/**
     * OBS passes the stitcher's output height (doubled).
     * Halve it to match the source camera's native resolution.
     */
	fival->height /= 2;

	ret = UVC_CALL_OP(src, vidioc_enum_frameintervals, src->filp, src->fh,
			  fival);

	/* Restore height to stitcher output value for userspace */
	fival->height *= 2;

	mutex_unlock(&stitcher->src_disc_lock);
	return ret;
}

const struct v4l2_ioctl_ops stitcher_ioctl_ops = {
	// clang-format off
    .vidioc_querycap             = stitcher_vidioc_querycap,
    .vidioc_enum_fmt_vid_cap     = stitcher_vidioc_enum_fmt,
    .vidioc_g_fmt_vid_cap        = stitcher_vidioc_g_fmt,
    .vidioc_s_fmt_vid_cap        = stitcher_vidioc_s_fmt,
    .vidioc_try_fmt_vid_cap      = stitcher_vidioc_try_fmt,

    .vidioc_s_input              = stitcher_vidioc_s_input,
    .vidioc_g_input              = stitcher_vidioc_g_input,
    .vidioc_enum_input           = stitcher_vidioc_enum_input,

    .vidioc_g_parm               = stitcher_vidioc_g_parm,
    .vidioc_s_parm               = stitcher_vidioc_s_parm,

    .vidioc_enum_framesizes      = stitcher_vidioc_enum_framesizes,
    .vidioc_enum_frameintervals  = stitcher_vidioc_enum_frameintervals,

    .vidioc_reqbufs              = vb2_ioctl_reqbufs,
    .vidioc_querybuf             = vb2_ioctl_querybuf,
    .vidioc_qbuf                 = vb2_ioctl_qbuf,
    .vidioc_dqbuf                = vb2_ioctl_dqbuf,
    .vidioc_expbuf               = vb2_ioctl_expbuf,

    .vidioc_streamon             = stitcher_vidioc_streamon,
    .vidioc_streamoff            = stitcher_vidioc_streamoff,
	// clang-format on
};

const struct v4l2_file_operations stitcher_fops = {
	// clang-format off
    .owner          = THIS_MODULE,
    .open           = v4l2_fh_open,
    .release        = vb2_fop_release,
    .unlocked_ioctl = video_ioctl2,
    .mmap           = vb2_fop_mmap,
    .read           = vb2_fop_read,
    .poll           = vb2_fop_poll,
	// clang-format on
};

/**/
static void init_vdev(struct uvc_stitcher *stitcher)
{
	struct video_device *vdev = stitcher->vdev;

	vdev->fops = &stitcher_fops;
	vdev->ioctl_ops = &stitcher_ioctl_ops;
	vdev->release = video_device_release;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE;
	strscpy(vdev->name, "UVC Stitch", sizeof(vdev->name));
	vdev->vfl_type = VFL_TYPE_VIDEO;
	vdev->vfl_dir = VFL_DIR_RX;

	/*
     * Use the output queue mutex as the vdev lock.  This serializes
     * all ioctls and — critically — allows STREAMOFF to preempt a
     * blocking DQBUF: vb2 releases q->lock (== vdev->lock) while
     * sleeping, so STREAMOFF can acquire it and cancel the queue.
     *
     * Without this, ffplay's SDL event loop cannot interrupt a
     * blocking DQBUF to process the 'q' key, because close() →
     * vb2_fop_release needs vdev->lock to proceed.
     */
	vdev->lock = &stitcher->out_q.mutex;

	vdev->queue = &stitcher->out_q.vbq;
	vdev->v4l2_dev = &stitcher->v4l2_dev;
	video_set_drvdata(vdev, stitcher);
}

/* ------------------------------------------------------------------ */
static void stitcher_unregister(struct uvc_stitcher *stitcher)
{
	if (!stitcher)
		return;

	if (stitcher->vdev) {
		video_set_drvdata(stitcher->vdev, NULL);
		video_unregister_device(stitcher->vdev);
		stitcher->vdev = NULL;
	}

	if (stitcher->out_q_initialized) {
		stitcher_queue_release(&stitcher->out_q);
		stitcher->out_q_initialized = false;
	}

	if (stitcher->v4l2_dev.dev)
		v4l2_device_unregister(&stitcher->v4l2_dev);
}

/**/
static int stitcher_add(void)
{
	struct uvc_stitcher *stitcher;
	STITCHER_CLASS_CONST struct class *vcls;
	int ret;

	pr_info("uvc_stitcher: initializing stitcher...\n");
	stitcher = kzalloc(sizeof(*stitcher), GFP_KERNEL);
	if (!stitcher) {
		pr_err("uvc_stitcher: failed to allocate memory for stitcher\n");
		return -ENOMEM;
	}

	mutex_init(&stitcher->lock);
	mutex_init(&stitcher->src_disc_lock);
	mutex_init(&stitcher->uvc_ctrl_lock);
	atomic_set(&stitcher->streaming, 0);

	stitcher->out_width = STITCHER_DEFAULT_OUT_WIDTH;
	stitcher->out_height = STITCHER_DEFAULT_OUT_HEIGHT;
	stitcher->out_pixfmt = V4L2_PIX_FMT_YUYV;
	stitcher->src_width = STITCHER_DEFAULT_OUT_WIDTH;
	stitcher->src_height = STITCHER_DEFAULT_OUT_HEIGHT / 2;
	stitcher->fps_num = STITCHER_DEFAULT_FPS_NUM;
	stitcher->fps_den = STITCHER_DEFAULT_FPS_DEN;

	stitcher->disc_wq = alloc_ordered_workqueue("uvc_stitcher_disc", 0);
	if (!stitcher->disc_wq) {
		pr_err("uvc_stitcher: failed to allocate workqueue\n");
		ret = -ENOMEM;
		goto err_free;
	}
	pr_info("uvc_stitcher: workqueue created\n");

	strscpy(stitcher->v4l2_dev.name, "uvc_stitcher",
		sizeof(stitcher->v4l2_dev.name));
	ret = v4l2_device_register(NULL, &stitcher->v4l2_dev);
	if (ret) {
		pr_err("uvc_stitcher: v4l2_device_register failed: %d\n", ret);
		goto err_wq;
	}

	ret = init_queue(&stitcher->out_q);
	if (ret) {
		pr_err("uvc_stitcher: init_queue failed: %d\n", ret);
		goto err_v4l2;
	}
	stitcher->out_q_initialized = true;
	pr_info("uvc_stitcher: output queue initialized\n");

	stitcher->vdev = video_device_alloc();
	if (!stitcher->vdev) {
		pr_err("uvc_stitcher: failed to allocate video device\n");
		goto err_queue;
	}

	init_vdev(stitcher);
	pr_info("uvc_stitcher: video device initialized\n");

	ret = video_register_device(stitcher->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		pr_err("uvc_stitcher: video_register_device failed: %d\n", ret);
		goto err_vdev;
	}
	pr_info("uvc_stitcher: video device registered as /dev/video%d\n",
		stitcher->vdev->num);
	g_stitcher = stitcher;

	vcls = stitcher->vdev->dev.class;
	if (!vcls) {
		pr_err("uvc_stitcher: cannot get video_class from vdev\n");
		ret = -ENODEV;
		goto err_intf;
	}
	ret = stitcher_register_class_intf(stitcher, vcls);
	if (ret) {
		pr_err("uvc_stitcher: failed to register class interface: %d\n",
		       ret);
		goto err_intf;
	}
	pr_info("uvc_stitcher: class interface registered, monitoring devices...\n");

	pr_info("uvc_stitcher: initialization complete\n");
	return 0;

err_intf:
	video_set_drvdata(stitcher->vdev, NULL);
	video_unregister_device(stitcher->vdev);
err_vdev:
	video_device_release(stitcher->vdev);
	stitcher->vdev = NULL;
err_queue:
	stitcher_queue_release(&stitcher->out_q);
	stitcher->out_q_initialized = false;
err_v4l2:
	v4l2_device_unregister(&stitcher->v4l2_dev);
err_wq:
	destroy_workqueue(stitcher->disc_wq);
err_free:
	kfree(stitcher);
	return ret;
}

static void stitcher_remove(struct uvc_stitcher *stitcher)
{
	int i;

	pr_info("uvc_stitcher: removing stitcher...\n");
	if (!stitcher) {
		pr_err("uvc_stitcher: stitcher is NULL\n");
		return;
	}

	/* unregister the class interface */
	pr_info("uvc_stitcher: unregistering class interface...\n");
	stitcher_unregister_class_intf(stitcher);

	pr_info("uvc_stitcher: draining and destroying workqueue...\n");
	drain_workqueue(stitcher->disc_wq);
	destroy_workqueue(stitcher->disc_wq);
	stitcher->disc_wq = NULL;

	/* If streaming, stop */
	if (atomic_read(&stitcher->streaming)) {
		pr_info("uvc_stitcher: stopping active streaming...\n");
		stitcher_uvc_stop(stitcher);
	}

	/* close all source filp */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		if (!IS_ERR_OR_NULL(stitcher->src[i].filp)) {
			pr_info("uvc_stitcher: closing src%d...\n", i);
			filp_close(stitcher->src[i].filp, NULL);
			stitcher->src[i].filp = NULL;
		}
	}

	/* unregister video node */
	pr_info("uvc_stitcher: unregistering video device...\n");
	stitcher_unregister(stitcher);
	kfree(stitcher);
	g_stitcher = NULL;
	pr_info("uvc_stitcher: removed\n");
}

/**/
static int __init uvc_stitcher_init(void)
{
	pr_info("uvc_stitcher: module loading...\n");
	return stitcher_add();
}

static void __exit uvc_stitcher_exit(void)
{
	pr_info("uvc_stitcher: module unloading...\n");
	stitcher_remove(g_stitcher);
	pr_info("uvc_stitcher: module unloaded\n");
}

module_init(uvc_stitcher_init);
module_exit(uvc_stitcher_exit);