#include <linux/module.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/kref.h>
#include <linux/completion.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/sched.h>
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
#define STITCHER_SRC_P 0
#define STITCHER_SRC_S 1
#define STITCHER_MAX_OUT_BUFS 8
#define STITCHER_SYNC_DRAIN_FRAMES 4
#define STITCHER_SYNC_MAX_RETRIES 3
#define STITCHER_SEQ_MISMATCH_THRESHOLD 2

/* Restarting a source that stopped (see the kthread section) */
#define STITCHER_ERR_BURST 2		/* corrupt frames in a row */
#define STITCHER_SRC_TIMEOUT_MS 250	/* minimum silence before a restart */
#define STITCHER_SRC_TIMEOUT_FRAMES 8	/* ... or this many frame periods */
#define STITCHER_RESTART_FAST 3		/* immediate restarts before backing off */
#define STITCHER_RESTART_BACKOFF_MS 100
#define STITCHER_RESTART_BACKOFF_MAX_MS 1000
#define STITCHER_WATCH_HEADS 4		/* wait queues a source's poll() may use */

/* Limits for the format list copied from the S-side source */
#define STITCHER_MAX_FRMSIZES 16
#define STITCHER_MAX_FRMIVALS 8
#define STITCHER_ENUM_LIMIT 64

#define STITCHER_DEFAULT_VID 0x1E4E
#define STITCHER_DEFAULT_PID_P 0x7301
#define STITCHER_DEFAULT_PID_S 0x7302

#define STITCHER_DEFAULT_PIXFMT V4L2_PIX_FMT_NV12

/* VIDEO_NUM_DEVICES in v4l2-dev.c; not exported in a header */
#define STITCHER_V4L2_MINORS 256

/* How long discovery waits for a new UVC node to finish registering */
#define STITCHER_REG_WAIT_MS 2000
#define STITCHER_REG_POLL_MS 10

/**
 * UVC_CALL_OP - call a UVC ioctl op directly, holding the vdev lock.
 *
 * NOTE: This bypasses V4L2 core's capability checks and v4l2_fh state
 * validation. It is intentional for this kernel-space stitching design, but
 * callers must ensure the source filp/vdev remain valid (see the locking
 * notes on struct uvc_stitcher).
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
/* Pixel formats                                                      */
/* ------------------------------------------------------------------ */
/*
 * Formats the stitcher can rebuild.  The EJ535 always splits the byte
 * stream of the full frame in half (P = first half, S = second half), so
 * any format whose frame size is known can be joined by placing the two
 * halves back to back.
 */
struct stitcher_pixfmt {
	u32 fourcc;
	u32 bpl_mul;	/* bytesperline = width * bpl_mul */
	u32 size_num;	/* sizeimage = width * height * size_num / size_den */
	u32 size_den;
};

static const struct stitcher_pixfmt stitcher_pixfmts[] = {
	{ V4L2_PIX_FMT_NV12, 1, 3, 2 },
	{ V4L2_PIX_FMT_YUYV, 2, 2, 1 },
	{ V4L2_PIX_FMT_UYVY, 2, 2, 1 },
	{ V4L2_PIX_FMT_BGR24, 3, 3, 1 },
};

#define STITCHER_NUM_PIXFMTS ARRAY_SIZE(stitcher_pixfmts)

/* ------------------------------------------------------------------ */
struct stitcher_slot {
	struct file *filp;
	struct video_device *vdev;
	struct v4l2_fh *fh;
	struct usb_interface *intf;
	struct v4l2_format fmt;
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
 * cached_fd: persistent fd in init_files for kthread QBUF.
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

/* A buffer dequeued from a source */
struct stitcher_dq {
	unsigned int index;
	u32 sequence;
	u32 flags;
	u32 bytesused;
	ktime_t ts;
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

/* One output frame size: S-side width, twice the S-side height. */
struct stitcher_frmsize {
	u32 width;
	u32 height;
	unsigned int num_ivals;
	struct v4l2_fract ivals[STITCHER_MAX_FRMIVALS];
};

struct stitcher_format {
	const struct stitcher_pixfmt *info;
	unsigned int num_sizes;
	struct stitcher_frmsize sizes[STITCHER_MAX_FRMSIZES];
};

/**
 * struct stitcher_out - one registered "UVC Stitch" video node.
 *
 * Created when both P and S are connected and unregistered as soon as
 * either one goes away.  An application may keep the node open after it
 * has been unregistered, so the object is freed from vdev->release once
 * the last user is gone; a replug always gets a new stitcher_out.
 */
struct stitcher_out {
	struct video_device vdev;
	struct stitcher_video_queue q;
	struct uvc_stitcher *stitcher;

	struct kref ref;
	bool vdev_released;

	/* Copied from the S-side source at registration; read-only after. */
	unsigned int num_formats;
	struct stitcher_format formats[STITCHER_NUM_PIXFMTS];

	/* Protected by q.mutex */
	bool disconnected;
	const struct stitcher_format *cur_fmt;
	const struct stitcher_frmsize *cur_size;
	struct v4l2_fract timeperframe;
};

/**
 * struct uvc_stitcher - module-wide state.
 *
 * Lock order, outermost first.  A lock is never taken while a later one
 * in this list is held:
 *
 *   disc_lock       src[], src_ready_count, out, exiting.  Taken only by
 *                   the discovery work and module exit.
 *   out->q.mutex    vb2 queue lock == vdev->lock.  The V4L2 core holds it
 *                   across every ioctl on the node; it also protects the
 *                   node's current format and its disconnected flag.
 *   uvc_ctrl_lock   streaming engine start/stop and the engine_* fields.
 *   src_stream_lock source STREAMON/STREAMOFF (engine stop vs. kthread
 *                   resync).
 *   UVC vdev lock   taken inside UVC_CALL_OP.
 *   out->q.irqlock  spinlock for the queued output buffer list.
 *
 * The stitcher kthread only takes src_stream_lock, the UVC vdev locks
 * (also through the sources' poll()) and irqlock, so the stop path may
 * wait in kthread_stop() while holding the mutexes above it.  The kthread
 * sleeps holding no lock; its wait queue callbacks only wake it up.
 *
 * src[] may be read without disc_lock while the engine runs: a slot is
 * only cleared after the node using it has been marked disconnected and
 * its engine stopped.
 */
struct uvc_stitcher {
	struct v4l2_device v4l2_dev;

	struct class_interface class_intf;
	struct workqueue_struct *disc_wq;

	struct mutex disc_lock;
	bool exiting;
	int src_ready_count;
	struct stitcher_slot src[STITCHER_NUM_SOURCES];
	struct stitcher_out *out;

	struct mutex uvc_ctrl_lock;
	struct stitcher_out *engine_out;
	struct task_struct *stitcher_task;
	atomic_t streaming;
	struct completion engine_started;
	int engine_start_ret;
	u32 frame_size;
	unsigned long src_timeout;
	unsigned int num_dmabuf_slots;
	struct stitcher_dmabuf_slot dma_slots[STITCHER_MAX_OUT_BUFS];
	struct vb2_buffer *out_vb_map[STITCHER_MAX_OUT_BUFS];

	struct mutex src_stream_lock;
};

/* ------------------------------------------------------------------ */
/* Forward declarations                                               */
/* ------------------------------------------------------------------ */
static int init_queue(struct stitcher_video_queue *q);
static void stitcher_out_init_vdev(struct uvc_stitcher *stitcher,
				   struct stitcher_out *out);

static struct uvc_stitcher *g_stitcher;

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

/* ------------------------------------------------------------------ */
/* Pixel-format helpers                                               */
/* ------------------------------------------------------------------ */
static const struct stitcher_pixfmt *stitcher_pixfmt_find(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < STITCHER_NUM_PIXFMTS; i++)
		if (stitcher_pixfmts[i].fourcc == fourcc)
			return &stitcher_pixfmts[i];
	return NULL;
}

static inline u32 stitcher_bytesperline(const struct stitcher_pixfmt *info,
					u32 w)
{
	return w * info->bpl_mul;
}

/* Sizes in the format list are checked against this at registration. */
static inline u64 stitcher_image_size(const struct stitcher_pixfmt *info,
				      u32 w, u32 h)
{
	return (u64)w * h * info->size_num / info->size_den;
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
				     struct stitcher_dq *dq)
{
	struct stitcher_slot *src = &stitcher->src[slot];
	struct v4l2_buffer buf = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	int ret;

	if (!stitcher_src_valid(src))
		return -ENODEV;

	ret = UVC_CALL_OP(src, vidioc_dqbuf, src->filp, src->fh, &buf);
	if (ret) {
		/* STREAMOFF from the stop path makes DQBUF fail; not an error */
		if (atomic_read(&stitcher->streaming))
			pr_err("uvc_stitcher: src%d DQBUF failed: %d\n", slot,
			       ret);
		return ret;
	}

	/*
	 * Once the device is gone, older uvcvideo completes every queued
	 * buffer with an error immediately; stop instead of spinning.
	 */
	if ((buf.flags & V4L2_BUF_FLAG_ERROR) &&
	    interface_to_usbdev(src->intf)->state == USB_STATE_NOTATTACHED) {
		pr_info("uvc_stitcher: src%d disconnected\n", slot);
		return -ENODEV;
	}

	dq->index = buf.index;
	dq->sequence = buf.sequence;
	dq->flags = buf.flags;
	dq->bytesused = buf.bytesused;
	dq->ts = ns_to_ktime((u64)buf.timestamp.tv_sec * NSEC_PER_SEC +
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
 * stitcher_dmabuf_create_cached_fds - create persistent fds for QBUF.
 *
 * Must run in the stitcher kthread: kthreads share init_files, so the fds
 * never appear in an application's fd table.
 */
static int stitcher_dmabuf_create_cached_fds(struct uvc_stitcher *stitcher)
{
	struct stitcher_dmabuf_slot *ds;
	unsigned int i;
	int s, fd;

	for (i = 0; i < stitcher->num_dmabuf_slots; i++) {
		ds = &stitcher->dma_slots[i];
		if (!ds->created)
			continue;

		for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
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
 *
 * Must run in the stitcher kthread, like stitcher_dmabuf_create_cached_fds().
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

	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		ds->cached_fd[i] = -1;

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
		ds->cached_fd[i] = -1;
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

/* ------------------------------------------------------------------ */
/* SRC UVC streaming control                                          */
/* ------------------------------------------------------------------ */
static int stitcher_uvc_negotiate_src_fmt(struct uvc_stitcher *stitcher,
					  int slot, u32 pixfmt, u32 width,
					  u32 height, u32 half_size)
{
	struct stitcher_slot *src = &stitcher->src[slot];
	struct v4l2_pix_format *pix = &src->fmt.fmt.pix;
	int ret;

	memset(&src->fmt, 0, sizeof(src->fmt));
	src->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	pix->width = width;
	pix->height = height;
	pix->pixelformat = pixfmt;
	pix->field = V4L2_FIELD_NONE;

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

	pr_info("uvc_stitcher: src%d format negotiated: %ux%u %p4cc sizeimage=%u\n",
		slot, pix->width, pix->height, &pix->pixelformat,
		pix->sizeimage);

	if (pix->pixelformat != pixfmt || pix->width != width ||
	    pix->height != height) {
		pr_err("uvc_stitcher: src%d does not support %ux%u %p4cc\n",
		       slot, width, height, &pixfmt);
		return -EINVAL;
	}

	/* Each source writes into one half of the output buffer */
	if (pix->sizeimage > half_size) {
		pr_err("uvc_stitcher: src%d sizeimage %u exceeds half frame %u\n",
		       slot, pix->sizeimage, half_size);
		return -EINVAL;
	}
	if (pix->sizeimage != half_size)
		pr_warn("uvc_stitcher: src%d sizeimage %u differs from half frame %u\n",
			slot, pix->sizeimage, half_size);

	return 0;
}

/**
 * stitcher_uvc_negotiate_src_fps - set the frame interval on a UVC source.
 *
 * Called after s_fmt so the UVC driver knows the active resolution.
 */
static void stitcher_uvc_negotiate_src_fps(struct uvc_stitcher *stitcher,
					   int slot,
					   const struct v4l2_fract *tpf)
{
	struct stitcher_slot *src = &stitcher->src[slot];
	struct v4l2_streamparm parm = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	};
	int ret;

	parm.parm.capture.timeperframe = *tpf;

	ret = UVC_CALL_OP(src, vidioc_s_parm, src->filp, src->fh, &parm);
	if (ret) {
		pr_info("uvc_stitcher: src%d s_parm failed: %d (non-fatal)\n",
			slot, ret);
		return;
	}

	pr_info("uvc_stitcher: src%d fps negotiated: %u/%u\n", slot,
		parm.parm.capture.timeperframe.denominator,
		parm.parm.capture.timeperframe.numerator);
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
	if (ret) {
		pr_err("uvc_stitcher: src%d REQBUFS(DMABUF) failed: %d\n", idx,
		       ret);
		return ret;
	}

	/* Output buffer i is paired with buffer i on both sources */
	if (req.count < count) {
		pr_err("uvc_stitcher: src%d REQBUFS(DMABUF) got %u/%u\n", idx,
		       req.count, count);
		return -ENOMEM;
	}
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
 * Makes the source detach from our dma_bufs.
 */
static void stitcher_uvc_free_bufs(struct uvc_stitcher *stitcher, int idx)
{
	struct stitcher_slot *src = &stitcher->src[idx];
	struct v4l2_requestbuffers req = {
		.count = 0,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_DMABUF,
	};

	if (!stitcher_src_valid(src))
		return;
	UVC_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &req);
}

/* ------------------------------------------------------------------ */
/* kthread                                                            */
/* ------------------------------------------------------------------ */
/*
 * Output buffer pages are exported as dma_buf and queued to the UVC
 * sources, which write their half directly into the output buffer.  The
 * kthread pairs up completed halves and returns the output buffer.
 *
 * A source can stop by itself: when its bulk URBs fail (the EJ535 does
 * this with -EPROTO right after STREAMON now and then), uvcvideo returns
 * every queued buffer with an error and never resubmits the URBs, so no
 * further frame arrives.  The kthread therefore watches both sources at
 * the same time and restarts them as soon as one of them
 *   - returns a cancelled buffer (error flag, nothing written),
 *   - returns STITCHER_ERR_BURST corrupt frames in a row, or
 *   - stays silent while the other source keeps delivering.
 */

struct stitcher_engine;

/* Wait queue entry the kthread adds to every queue it watches */
struct stitcher_waiter {
	struct wait_queue_entry wait;
	wait_queue_head_t *head;
	struct stitcher_engine *e;
};

enum stitcher_fault {
	STITCHER_FAULT_NONE,
	STITCHER_FAULT_CANCELLED,
	STITCHER_FAULT_CORRUPT,
	STITCHER_FAULT_SILENT,
	STITCHER_FAULT_SEQUENCE,
	STITCHER_FAULT_RETRY,
};

static const char *const stitcher_fault_name[] = {
	[STITCHER_FAULT_NONE] = "ok",
	[STITCHER_FAULT_CANCELLED] = "stopped (buffers cancelled)",
	[STITCHER_FAULT_CORRUPT] = "keeps returning corrupt frames",
	[STITCHER_FAULT_SILENT] = "stopped delivering frames",
	[STITCHER_FAULT_SEQUENCE] = "out of sync",
	[STITCHER_FAULT_RETRY] = "failed to restart",
};

/* State of one kthread run; allocated by the kthread */
struct stitcher_engine {
	struct uvc_stitcher *stitcher;
	struct stitcher_out *out;

	/* Output buffers handed to the sources and not completed yet */
	unsigned long inflight;
	/* Buffers queued to each source and not dequeued yet */
	unsigned long queued[STITCHER_NUM_SOURCES];
	/* Dequeued halves waiting for the other source, oldest first */
	struct stitcher_dq half[STITCHER_NUM_SOURCES][STITCHER_MAX_OUT_BUFS];
	unsigned int nhalf[STITCHER_NUM_SOURCES];
	unsigned int err_run[STITCHER_NUM_SOURCES];
	/* jiffies: start of the silence window, and last dequeue */
	unsigned long armed[STITCHER_NUM_SOURCES];
	unsigned long last_rx[STITCHER_NUM_SOURCES];
	unsigned long timeout;

	bool synced;
	unsigned int warm_pairs;
	unsigned int warm_attempts;

	/* Restarts since the last completed output frame */
	unsigned int restarts;
	bool restart_pending;
	ktime_t fault_time;

	/* Wake-ups from both sources and from the output queue */
	poll_table pt;
	bool triggered;
	bool watch_overflow;
	unsigned int nwaiters;
	struct stitcher_waiter
		waiters[STITCHER_NUM_SOURCES * STITCHER_WATCH_HEADS + 1];
};

/* Runs in the waker's context, possibly in an interrupt */
static int stitcher_wake(struct wait_queue_entry *wait, unsigned int mode,
			 int sync, void *key)
{
	struct stitcher_waiter *w =
		container_of(wait, struct stitcher_waiter, wait);

	/* Pairs with smp_store_mb() in stitcher_wait() */
	smp_wmb();
	WRITE_ONCE(w->e->triggered, true);
	return default_wake_function(wait, mode, sync, key);
}

static void stitcher_add_waiter(struct stitcher_engine *e,
				wait_queue_head_t *head)
{
	struct stitcher_waiter *w;

	if (e->nwaiters == ARRAY_SIZE(e->waiters)) {
		e->watch_overflow = true;
		return;
	}

	w = &e->waiters[e->nwaiters++];
	w->head = head;
	w->e = e;
	init_waitqueue_func_entry(&w->wait, stitcher_wake);
	w->wait.private = current;
	add_wait_queue(head, &w->wait);
}

/* poll_table callback: the source's poll() reports each queue it uses */
static void stitcher_watch_queue(struct file *file, wait_queue_head_t *head,
				 poll_table *pt)
{
	stitcher_add_waiter(container_of(pt, struct stitcher_engine, pt), head);
}

/*
 * Get woken up by both sources, the same way poll() would, and by the
 * output queue.  The entries stay until stitcher_unwatch(), which runs
 * before the kthread exits; the source files and the output queue both
 * outlive the kthread.
 */
static int stitcher_watch(struct stitcher_engine *e)
{
	struct uvc_stitcher *stitcher = e->stitcher;
	unsigned int before;
	int s;

	init_poll_funcptr(&e->pt, stitcher_watch_queue);
	for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
		before = e->nwaiters;
		vfs_poll(stitcher->src[s].filp, &e->pt);
		if (e->nwaiters == before) {
			pr_err("uvc_stitcher: cannot watch src%d\n", s);
			return -ENODEV;
		}
	}
	stitcher_add_waiter(e, &e->out->q.buf_wq);

	if (e->watch_overflow) {
		pr_err("uvc_stitcher: too many wait queues to watch\n");
		return -EINVAL;
	}
	return 0;
}

static void stitcher_unwatch(struct stitcher_engine *e)
{
	struct stitcher_waiter *w;

	while (e->nwaiters) {
		w = &e->waiters[--e->nwaiters];
		remove_wait_queue(w->head, &w->wait);
	}
}

/* Sleep until a watched queue has news, @timeout passes or we must stop. */
static void stitcher_wait(struct stitcher_engine *e, long timeout)
{
	set_current_state(TASK_INTERRUPTIBLE);
	if (!READ_ONCE(e->triggered) && !stitcher_should_stop(e->stitcher))
		schedule_timeout(timeout);
	__set_current_state(TASK_RUNNING);
	/* Pairs with smp_wmb() in stitcher_wake() */
	smp_store_mb(e->triggered, false);
}

static bool stitcher_src_detached(struct uvc_stitcher *stitcher)
{
	int s;

	for (s = 0; s < STITCHER_NUM_SOURCES; s++)
		if (interface_to_usbdev(stitcher->src[s].intf)->state ==
		    USB_STATE_NOTATTACHED)
			return true;
	return false;
}

/* QBUF one half of output buffer @idx to source @s. */
static int stitcher_queue_half(struct stitcher_engine *e, int s,
			       unsigned int idx)
{
	struct uvc_stitcher *stitcher = e->stitcher;
	int ret;

	ret = stitcher_qbuf_src_dmabuf(stitcher, s, idx,
				       stitcher->dma_slots[idx].cached_fd[s]);
	if (ret)
		return ret;

	/* A source that had nothing to do starts a new silence window */
	if (!e->queued[s])
		e->armed[s] = jiffies;
	__set_bit(idx, &e->queued[s]);
	return 0;
}

/* Hand the output buffers queued by userspace to both sources. */
static int stitcher_submit_queued(struct stitcher_engine *e)
{
	struct uvc_stitcher *stitcher = e->stitcher;
	struct stitcher_out *out = e->out;
	struct stitcher_video_buffer *mbuf;
	unsigned long flags;
	unsigned int idx;
	int s, ret;

	for (;;) {
		spin_lock_irqsave(&out->q.irqlock, flags);
		mbuf = list_first_entry_or_null(&out->q.irqqueue,
						struct stitcher_video_buffer,
						queue);
		if (mbuf)
			list_del(&mbuf->queue);
		spin_unlock_irqrestore(&out->q.irqlock, flags);

		if (!mbuf)
			return 0;

		idx = mbuf->buf.vb2_buf.index;
		if (idx >= stitcher->num_dmabuf_slots ||
		    test_bit(idx, &e->inflight)) {
			pr_err("uvc_stitcher: cannot submit output buffer %u\n",
			       idx);
			return -EINVAL;
		}

		for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
			ret = stitcher_queue_half(e, s, idx);
			if (ret)
				return ret;
		}
		__set_bit(idx, &e->inflight);
	}
}

/*
 * Dequeue every buffer source @s has ready.  Sets *fault and stops when
 * @s has to be restarted.
 */
static int stitcher_collect(struct stitcher_engine *e, int s,
			    enum stitcher_fault *fault, bool *busy)
{
	struct uvc_stitcher *stitcher = e->stitcher;
	struct stitcher_dq dq;
	int ret;

	while (e->queued[s]) {
		/* EPOLLERR: not streaming any more; DQBUF reports why */
		if (!(vfs_poll(stitcher->src[s].filp, NULL) &
		      (EPOLLIN | EPOLLERR)))
			return 0;

		ret = stitcher_dqbuf_src_dmabuf(stitcher, s, &dq);
		if (ret)
			return ret;
		*busy = true;

		if (dq.index >= stitcher->num_dmabuf_slots ||
		    !test_bit(dq.index, &e->queued[s])) {
			pr_err("uvc_stitcher: src%d returned unexpected buffer %u\n",
			       s, dq.index);
			return -EIO;
		}
		__clear_bit(dq.index, &e->queued[s]);
		e->armed[s] = jiffies;
		e->last_rx[s] = e->armed[s];

		if (dq.flags & V4L2_BUF_FLAG_ERROR) {
			/*
			 * vb2 clears bytesused at QBUF and only uvcvideo's
			 * frame completion sets it, so an empty error buffer
			 * was cancelled: the source's URBs are gone.
			 */
			if (!dq.bytesused) {
				*fault = STITCHER_FAULT_CANCELLED;
				return 0;
			}
			if (++e->err_run[s] >= STITCHER_ERR_BURST) {
				*fault = STITCHER_FAULT_CORRUPT;
				return 0;
			}
		} else {
			e->err_run[s] = 0;
		}

		/* Each buffer is dequeued once per QBUF, so this fits */
		e->half[s][e->nhalf[s]++] = dq;
	}
	return 0;
}

static void stitcher_pop_half(struct stitcher_engine *e, int s,
			      struct stitcher_dq *dq)
{
	*dq = e->half[s][0];
	e->nhalf[s]--;
	memmove(&e->half[s][0], &e->half[s][1],
		e->nhalf[s] * sizeof(e->half[s][0]));
}

/* Both halves of output buffer p->index are filled: return it. */
static void stitcher_complete(struct stitcher_engine *e,
			      const struct stitcher_dq *p,
			      const struct stitcher_dq *s)
{
	struct uvc_stitcher *stitcher = e->stitcher;
	struct vb2_buffer *out_vb = stitcher->out_vb_map[p->index];
	struct vb2_v4l2_buffer *v4l2_vb = to_vb2_v4l2_buffer(out_vb);

	__clear_bit(p->index, &e->inflight);
	vb2_set_plane_payload(out_vb, 0, stitcher->frame_size);
	out_vb->timestamp = ktime_to_ns(ktime_before(p->ts, s->ts) ? p->ts :
								    s->ts);
	v4l2_vb->sequence = p->sequence;
	v4l2_vb->field = V4L2_FIELD_NONE;
	vb2_buffer_done(out_vb, VB2_BUF_STATE_DONE);

	if (e->fault_time) {
		pr_info("uvc_stitcher: recovered %lld ms after the fault (%u restart(s))\n",
			ktime_ms_delta(ktime_get(), e->fault_time), e->restarts);
		e->fault_time = 0;
	}
	e->restarts = 0;
}

/*
 * After a (re)start, drain STITCHER_SYNC_DRAIN_FRAMES pairs and check that
 * both sources reached the same frame.  At STREAMON the two sources may
 * start at different times (alt-setting switch latency).
 */
static int stitcher_warmup(struct stitcher_engine *e,
			   const struct stitcher_dq *p,
			   const struct stitcher_dq *s,
			   enum stitcher_fault *fault)
{
	int ret;

	ret = stitcher_queue_half(e, 0, p->index);
	if (!ret)
		ret = stitcher_queue_half(e, 1, s->index);
	if (ret)
		return ret;

	if (++e->warm_pairs < STITCHER_SYNC_DRAIN_FRAMES)
		return 0;

	if (p->sequence != s->sequence &&
	    e->warm_attempts < STITCHER_SYNC_MAX_RETRIES) {
		pr_warn("uvc_stitcher: sequence mismatch after draining %d frames: "
			"src0 seq=%u vs src1 seq=%u (attempt %u)\n",
			STITCHER_SYNC_DRAIN_FRAMES, p->sequence, s->sequence,
			e->warm_attempts);
		e->warm_attempts++;
		*fault = STITCHER_FAULT_SEQUENCE;
		return 0;
	}

	if (p->sequence == s->sequence)
		pr_info("uvc_stitcher: synced at seq %u (attempt %u, drained %d)\n",
			p->sequence, e->warm_attempts,
			STITCHER_SYNC_DRAIN_FRAMES);
	else
		pr_warn("uvc_stitcher: sync failed after %d retries (seq %u vs %u), proceeding\n",
			STITCHER_SYNC_MAX_RETRIES, p->sequence, s->sequence);

	e->synced = true;
	e->warm_attempts = 0;
	return 0;
}

/* Pair dequeued halves and return the completed output buffers. */
static int stitcher_pair(struct stitcher_engine *e, enum stitcher_fault *fault,
			 bool *busy)
{
	struct stitcher_dq p, s;
	unsigned int offset;
	int ret;

	while (e->nhalf[0] && e->nhalf[1]) {
		stitcher_pop_half(e, 0, &p);
		stitcher_pop_half(e, 1, &s);
		*busy = true;

		/* Halves of different output buffers: give both back */
		if (p.index != s.index) {
			pr_warn_ratelimited("uvc_stitcher: DQBUF index mismatch %u vs %u\n",
					    p.index, s.index);
			ret = stitcher_queue_half(e, 0, p.index);
			if (!ret)
				ret = stitcher_queue_half(e, 1, s.index);
			if (ret)
				return ret;
			continue;
		}

		if (!e->synced) {
			ret = stitcher_warmup(e, &p, &s, fault);
			if (ret || *fault)
				return ret;
			continue;
		}

		if (p.sequence != s.sequence) {
			offset = (p.sequence > s.sequence) ?
					 (p.sequence - s.sequence) :
					 (s.sequence - p.sequence);
			if (offset > STITCHER_SEQ_MISMATCH_THRESHOLD) {
				pr_warn_ratelimited("uvc_stitcher: DQBUF sequence mismatch %u vs %u (offset %u)\n",
						    p.sequence, s.sequence,
						    offset);
				/* p.index stays in flight; the restart requeues it */
				*fault = STITCHER_FAULT_SEQUENCE;
				return 0;
			}

			pr_info_ratelimited("uvc_stitcher: seq mismatch %u vs %u, offset %u\n",
					    p.sequence, s.sequence, offset);
		}

		stitcher_complete(e, &p, &s);
	}
	return 0;
}

/*
 * Source that is silent while the other one has delivered something since
 * the silence began, or -1.  When both are silent (no input signal, for
 * instance) restarting would not help.
 */
static int stitcher_silent_src(struct stitcher_engine *e)
{
	unsigned long now = jiffies;
	int s;

	for (s = 0; s < STITCHER_NUM_SOURCES; s++)
		if (e->queued[s] && time_after(now, e->armed[s] + e->timeout) &&
		    time_after(e->last_rx[!s], e->armed[s]))
			return s;
	return -1;
}

/* How long to sleep before the silence check has to run again. */
static long stitcher_next_check(struct stitcher_engine *e)
{
	unsigned long now = jiffies, due;
	long wait = MAX_SCHEDULE_TIMEOUT;
	int s;

	for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
		if (!e->queued[s])
			continue;
		due = e->armed[s] + e->timeout;
		/* Overdue but not restartable: look again a window later */
		wait = min_t(long, wait,
			     time_after(due, now) ? due - now : e->timeout);
	}
	return wait;
}

/*
 * Restart both sources: STREAMOFF, re-QBUF every buffer in flight,
 * STREAMON.  The UVC sequence counters restart at 0, so the warm-up runs
 * again.  Restarts that keep failing are spaced out.
 */
static int stitcher_restart(struct stitcher_engine *e,
			    enum stitcher_fault fault, int src)
{
	struct uvc_stitcher *stitcher = e->stitcher;
	unsigned long deadline, now;
	unsigned int i, ms;
	int s, ret = 0;
	long left;

	if (src >= 0)
		pr_warn_ratelimited("uvc_stitcher: src%d %s, restarting sources\n",
				    src, stitcher_fault_name[fault]);
	else
		pr_warn_ratelimited("uvc_stitcher: sources %s, restarting\n",
				    stitcher_fault_name[fault]);

	if (!e->fault_time)
		e->fault_time = ktime_get();

	if (e->restarts >= STITCHER_RESTART_FAST) {
		ms = STITCHER_RESTART_BACKOFF_MS
		     << min(e->restarts - STITCHER_RESTART_FAST, 4u);
		ms = min_t(unsigned int, ms, STITCHER_RESTART_BACKOFF_MAX_MS);
		deadline = jiffies + msecs_to_jiffies(ms);
		while (!stitcher_should_stop(stitcher)) {
			left = (long)(deadline - jiffies);
			if (left <= 0)
				break;
			stitcher_wait(e, left);
		}
	}
	e->restarts++;

	mutex_lock(&stitcher->src_stream_lock);

	/* The stop path has already turned the sources off; keep them off */
	if (!atomic_read(&stitcher->streaming))
		goto out;

	/* STREAMOFF also gives back every buffer still queued */
	for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
		stitcher_uvc_streamoff_only(stitcher, s);
		e->queued[s] = 0;
		e->nhalf[s] = 0;
		e->err_run[s] = 0;
	}

	for_each_set_bit(i, &e->inflight, STITCHER_MAX_OUT_BUFS) {
		for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
			ret = stitcher_queue_half(e, s, i);
			if (ret)
				goto out;
		}
	}

	for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
		ret = stitcher_uvc_streamon(stitcher, s);
		if (ret)
			break;
	}

	if (ret) {
		/* Leave both sources off and try again later */
		for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
			stitcher_uvc_streamoff_only(stitcher, s);
			e->queued[s] = 0;
		}
		if (stitcher_src_detached(stitcher)) {
			ret = -ENODEV;
			goto out;
		}
		e->restart_pending = true;
		ret = 0;
		goto out;
	}
	e->restart_pending = false;

	now = jiffies;
	for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
		e->armed[s] = now;
		e->last_rx[s] = now;
	}
	e->synced = false;
	e->warm_pairs = 0;
	if (fault != STITCHER_FAULT_SEQUENCE)
		e->warm_attempts = 0;

out:
	mutex_unlock(&stitcher->src_stream_lock);
	return ret;
}

/*
 * Main loop.  Returns 0 when asked to stop, or a negative error when
 * streaming cannot continue.
 */
static int stitcher_run(struct stitcher_engine *e)
{
	struct uvc_stitcher *stitcher = e->stitcher;
	enum stitcher_fault fault;
	int s, src, ret;
	bool busy;

	while (!stitcher_should_stop(stitcher)) {
		fault = STITCHER_FAULT_NONE;
		src = -1;
		busy = false;

		/* Submit output buffers that userspace (re-)queued */
		ret = stitcher_submit_queued(e);
		if (ret)
			return ret;

		if (e->restart_pending) {
			fault = STITCHER_FAULT_RETRY;
		} else {
			for (s = 0; s < STITCHER_NUM_SOURCES && !fault; s++) {
				ret = stitcher_collect(e, s, &fault, &busy);
				if (ret)
					return ret;
				if (fault)
					src = s;
			}
			if (!fault) {
				ret = stitcher_pair(e, &fault, &busy);
				if (ret)
					return ret;
			}
			if (!fault) {
				src = stitcher_silent_src(e);
				if (src >= 0)
					fault = STITCHER_FAULT_SILENT;
			}
		}

		if (fault) {
			ret = stitcher_restart(e, fault, src);
			if (ret)
				return ret;
			continue;
		}

		if (!busy)
			stitcher_wait(e, stitcher_next_check(e));
	}

	return 0;
}

/* Install fds, queue the initial buffers and turn both sources on. */
static int stitcher_engine_prepare(struct stitcher_engine *e)
{
	struct uvc_stitcher *stitcher = e->stitcher;
	unsigned long now;
	int s, ret;

	ret = stitcher_dmabuf_create_cached_fds(stitcher);
	if (ret) {
		pr_err("uvc_stitcher: failed to create cached fds: %d\n", ret);
		return ret;
	}

	/* Before STREAMON, so that no wake-up is missed */
	ret = stitcher_watch(e);
	if (ret)
		return ret;

	ret = stitcher_submit_queued(e);
	if (ret)
		return ret;

	mutex_lock(&stitcher->src_stream_lock);
	for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
		ret = stitcher_uvc_streamon(stitcher, s);
		if (ret)
			break;
	}
	mutex_unlock(&stitcher->src_stream_lock);

	now = jiffies;
	for (s = 0; s < STITCHER_NUM_SOURCES; s++) {
		e->armed[s] = now;
		e->last_rx[s] = now;
	}
	return ret;
}

static int stitcher_thread_fn(void *data)
{
	struct uvc_stitcher *stitcher = data;
	struct stitcher_engine *e;
	int ret;

	sched_set_fifo(current);

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (e) {
		e->stitcher = stitcher;
		e->out = stitcher->engine_out;
		e->timeout = stitcher->src_timeout;
		ret = stitcher_engine_prepare(e);
	} else {
		ret = -ENOMEM;
	}
	stitcher->engine_start_ret = ret;
	complete(&stitcher->engine_started);

	if (!ret) {
		ret = stitcher_run(e);
		if (ret && !stitcher_should_stop(stitcher)) {
			/* Make DQBUF/poll on the output node fail with -EIO */
			pr_err("uvc_stitcher: streaming aborted: %d\n", ret);
			vb2_queue_error(&e->out->q.vbq);
		}
	}

	/* No wake-up may reach @e once it is freed */
	if (e) {
		stitcher_unwatch(e);
		kfree(e);
	}
	stitcher_dmabuf_close_cached_fds(stitcher);

	/*
	 * kthread_stop() is always called for this thread and the caller
	 * holds no task_struct reference, so never exit before it.
	 */
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	__set_current_state(TASK_RUNNING);

	pr_info("uvc_stitcher: kthread exiting\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Streaming engine                                                   */
/* ------------------------------------------------------------------ */
/**
 * stitcher_engine_stop - stop the kthread and release the sources' buffers.
 *
 * Safe on a partially started engine.  Caller holds uvc_ctrl_lock.
 */
static void stitcher_engine_stop(struct uvc_stitcher *stitcher)
{
	struct stitcher_out *out = stitcher->engine_out;
	int i;

	lockdep_assert_held(&stitcher->uvc_ctrl_lock);

	if (!out)
		return;

	pr_info("uvc_stitcher: stopping pipeline\n");
	atomic_set(&stitcher->streaming, 0);

	/*
	 * STREAMOFF wakes the kthread up.  src_stream_lock keeps a restart
	 * in the kthread from turning the sources back on.
	 */
	mutex_lock(&stitcher->src_stream_lock);
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		stitcher_uvc_streamoff_only(stitcher, i);
	mutex_unlock(&stitcher->src_stream_lock);

	if (stitcher->stitcher_task) {
		wake_up_all(&out->q.buf_wq);
		kthread_stop(stitcher->stitcher_task);
		stitcher->stitcher_task = NULL;
	}

	/* REQBUFS(0) makes the sources drop their dma_buf attachments */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		stitcher_uvc_free_bufs(stitcher, i);

	stitcher_dmabuf_teardown_all(stitcher);
	memset(stitcher->out_vb_map, 0, sizeof(stitcher->out_vb_map));

	/* Release the runtime-PM refs taken in stitcher_engine_start() */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		usb_autopm_put_interface(stitcher->src[i].intf);

	stitcher->engine_out = NULL;
	pr_info("uvc_stitcher: pipeline stopped\n");
}

/**
 * stitcher_engine_start - start streaming into @out's buffers.
 *
 * Called from start_streaming with out->q.mutex held.
 */
static int stitcher_engine_start(struct uvc_stitcher *stitcher,
				 struct stitcher_out *out)
{
	const struct stitcher_pixfmt *info = out->cur_fmt->info;
	u32 width = out->cur_size->width;
	u32 height = out->cur_size->height;
	u32 frame_size = stitcher_image_size(info, width, height);
	u32 half_size = frame_size / 2;
	struct task_struct *task;
	unsigned int out_count, i;
	int ret;

	mutex_lock(&stitcher->uvc_ctrl_lock);

	if (out->disconnected) {
		ret = -ENODEV;
		goto unlock;
	}
	if (stitcher->engine_out) {
		pr_err("uvc_stitcher: pipeline already in use\n");
		ret = -EBUSY;
		goto unlock;
	}

	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		if (!stitcher_src_valid(&stitcher->src[i])) {
			pr_err("uvc_stitcher: src%d is not ready\n", i);
			ret = -ENODEV;
			goto unlock;
		}
	}

	out_count = vb2_get_num_buffers(&out->q.vbq);
	if (!out_count || out_count > STITCHER_MAX_OUT_BUFS) {
		pr_err("uvc_stitcher: invalid output buffer count %u\n",
		       out_count);
		ret = -EINVAL;
		goto unlock;
	}

	/*
	 * Resume the UVC sources and hold a runtime-PM ref for the whole
	 * streaming session.  On 6.17 uvcvideo no longer pins runtime PM
	 * across open(), so a device opened at discovery time can autosuspend
	 * before this point, making the PROBE control transfer in S_FMT fail
	 * with -EHOSTUNREACH (-113).  Released by stitcher_engine_stop().
	 */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		ret = usb_autopm_get_interface(stitcher->src[i].intf);
		if (ret) {
			pr_err("uvc_stitcher: autopm_get src%d failed: %d\n",
			       i, ret);
			while (i--)
				usb_autopm_put_interface(stitcher->src[i].intf);
			goto unlock;
		}
	}

	/* From here on stitcher_engine_stop() undoes everything */
	stitcher->engine_out = out;

	/* Both sources use the same format: half the output height */
	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		ret = stitcher_uvc_negotiate_src_fmt(stitcher, i, info->fourcc,
						     width, height / 2,
						     half_size);
		if (ret)
			goto err_stop;
		stitcher_uvc_negotiate_src_fps(stitcher, i, &out->timeperframe);
	}

	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		ret = stitcher_uvc_reqbufs_dmabuf(stitcher, i, out_count);
		if (ret)
			goto err_stop;
	}

	/* A source silent for this long while the other delivers is restarted */
	stitcher->src_timeout = msecs_to_jiffies(max_t(
		u64, STITCHER_SRC_TIMEOUT_MS,
		div_u64((u64)out->timeperframe.numerator * MSEC_PER_SEC *
				STITCHER_SRC_TIMEOUT_FRAMES,
			out->timeperframe.denominator)));

	/* Export each output buffer's two halves as dma_bufs */
	stitcher->frame_size = frame_size;
	stitcher->num_dmabuf_slots = out_count;
	for (i = 0; i < out_count; i++) {
		struct vb2_buffer *vb = vb2_get_buffer(&out->q.vbq, i);

		if (!vb) {
			ret = -EINVAL;
			goto err_stop;
		}
		stitcher->out_vb_map[i] = vb;

		ret = stitcher_dmabuf_setup_slot(
			stitcher, &stitcher->dma_slots[i], vb, half_size);
		if (ret)
			goto err_stop;
	}

	pr_info("uvc_stitcher: starting %ux%u %p4cc (out_bufs=%u half=%u)\n",
		width, height, &info->fourcc, out_count, half_size);

	atomic_set(&stitcher->streaming, 1);
	reinit_completion(&stitcher->engine_started);
	stitcher->engine_start_ret = 0;

	task = kthread_run(stitcher_thread_fn, stitcher, "uvc_stitcher");
	if (IS_ERR(task)) {
		ret = PTR_ERR(task);
		goto err_stop;
	}
	stitcher->stitcher_task = task;

	/*
	 * The kthread queues the buffers and turns the sources on.  It takes
	 * none of the locks held here, so waiting cannot deadlock.
	 */
	wait_for_completion(&stitcher->engine_started);
	ret = stitcher->engine_start_ret;
	if (ret)
		goto err_stop;

	mutex_unlock(&stitcher->uvc_ctrl_lock);
	return 0;

err_stop:
	stitcher_engine_stop(stitcher);
unlock:
	mutex_unlock(&stitcher->uvc_ctrl_lock);
	return ret;
}

/* ------------------------------------------------------------------ */
/**
 * vb2 queue operations
 */
static struct stitcher_out *vbq_to_out(struct vb2_queue *vbq)
{
	struct stitcher_video_queue *q = vb2_get_drv_priv(vbq);

	return container_of(q, struct stitcher_out, q);
}

static u32 stitcher_out_frame_size(const struct stitcher_out *out)
{
	return stitcher_image_size(out->cur_fmt->info, out->cur_size->width,
				   out->cur_size->height);
}

/**
 * stitcher_out_return_buffers - give every buffer owned by the driver back
 * to vb2.  The engine must be stopped and out->q.mutex held.
 */
static void stitcher_out_return_buffers(struct stitcher_out *out,
					enum vb2_buffer_state state)
{
	struct vb2_queue *vbq = &out->q.vbq;
	unsigned long flags;
	unsigned int i, num;

	/* Buffers in irqqueue are ACTIVE too; they are completed below */
	spin_lock_irqsave(&out->q.irqlock, flags);
	INIT_LIST_HEAD(&out->q.irqqueue);
	spin_unlock_irqrestore(&out->q.irqlock, flags);

	num = vb2_get_num_buffers(vbq);
	for (i = 0; i < num; i++) {
		struct vb2_buffer *vb = vb2_get_buffer(vbq, i);

		if (vb && vb->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb, state);
	}
}

static int stitcher_queue_setup(struct vb2_queue *vbq,
				unsigned int *num_buffers,
				unsigned int *num_planes, unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct stitcher_out *out = vbq_to_out(vbq);
	unsigned int size = stitcher_out_frame_size(out);

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
	struct stitcher_out *out = vbq_to_out(vb->vb2_queue);
	unsigned int size = stitcher_out_frame_size(out);

	if (vb2_plane_size(vb, 0) < size) {
		pr_err("uvc_stitcher: buffer plane size < required %u\n", size);
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
	struct stitcher_out *out = vbq_to_out(vbq);
	int ret;

	pr_info("uvc_stitcher: start_streaming called with %u buffers\n",
		count);

	ret = stitcher_engine_start(out->stitcher, out);
	if (ret) {
		pr_err("uvc_stitcher: start failed: %d, returning buffers\n",
		       ret);
		stitcher_out_return_buffers(out, VB2_BUF_STATE_QUEUED);
		return ret;
	}

	pr_info("uvc_stitcher: streaming started successfully\n");
	return 0;
}

static void stitcher_stop_streaming(struct vb2_queue *vbq)
{
	struct stitcher_out *out = vbq_to_out(vbq);
	struct uvc_stitcher *stitcher = out->stitcher;

	pr_info("uvc_stitcher: stop_streaming called\n");

	/* A node whose sources went away no longer owns the engine */
	mutex_lock(&stitcher->uvc_ctrl_lock);
	if (stitcher->engine_out == out)
		stitcher_engine_stop(stitcher);
	mutex_unlock(&stitcher->uvc_ctrl_lock);

	/*
	 * vb2 requires stop_streaming to return every ACTIVE buffer;
	 * otherwise it emits:
	 *   "driver bug: stop_streaming leaving buffer N in active state"
	 */
	stitcher_out_return_buffers(out, VB2_BUF_STATE_ERROR);
	pr_info("uvc_stitcher: stop_streaming completed\n");
}

static const struct vb2_ops stitcher_vb2_ops = {
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

	/*
	 * Buffer count is limited to STITCHER_MAX_OUT_BUFS by queue_setup
	 * (CREATE_BUFS is not supported).  max_num_buffers cannot be used:
	 * vb2 rejects values below VB2_MAX_FRAME.
	 */
	return vb2_queue_init(&q->vbq);
}

/* ------------------------------------------------------------------ */
/* V4L2 ioctls                                                        */
/* ------------------------------------------------------------------ */
static const struct stitcher_format *
stitcher_out_find_format(const struct stitcher_out *out, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < out->num_formats; i++)
		if (out->formats[i].info->fourcc == fourcc)
			return &out->formats[i];
	return NULL;
}

static const struct stitcher_frmsize *
stitcher_format_find_size(const struct stitcher_format *fmt, u32 width,
			  u32 height)
{
	unsigned int i;

	for (i = 0; i < fmt->num_sizes; i++)
		if (fmt->sizes[i].width == width &&
		    fmt->sizes[i].height == height)
			return &fmt->sizes[i];
	return NULL;
}

static const struct stitcher_frmsize *
stitcher_format_nearest_size(const struct stitcher_format *fmt, u32 width,
			     u32 height)
{
	const struct stitcher_frmsize *best = &fmt->sizes[0];
	u64 best_dist = U64_MAX;
	unsigned int i;

	for (i = 0; i < fmt->num_sizes; i++) {
		const struct stitcher_frmsize *s = &fmt->sizes[i];
		u64 dist = abs((s64)s->width - width) +
			   abs((s64)s->height - height);

		if (dist < best_dist) {
			best = s;
			best_dist = dist;
		}
	}
	return best;
}

/* Frame interval of @size closest to @want; the first one if @want is 0. */
static struct v4l2_fract
stitcher_frmsize_nearest_ival(const struct stitcher_frmsize *size,
			      const struct v4l2_fract *want)
{
	struct v4l2_fract best = size->ivals[0];
	u64 want_us, us, dist, best_dist = U64_MAX;
	unsigned int i;

	if (!want->numerator || !want->denominator)
		return best;

	want_us = div_u64((u64)want->numerator * USEC_PER_SEC,
			  want->denominator);
	for (i = 0; i < size->num_ivals; i++) {
		us = div_u64((u64)size->ivals[i].numerator * USEC_PER_SEC,
			     size->ivals[i].denominator);
		dist = us > want_us ? us - want_us : want_us - us;
		if (dist < best_dist) {
			best = size->ivals[i];
			best_dist = dist;
		}
	}
	return best;
}

static void stitcher_fill_pix(const struct stitcher_format *fmt,
			      const struct stitcher_frmsize *size,
			      struct v4l2_pix_format *pix)
{
	pix->width = size->width;
	pix->height = size->height;
	pix->pixelformat = fmt->info->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = stitcher_bytesperline(fmt->info, size->width);
	pix->sizeimage = stitcher_image_size(fmt->info, size->width,
					     size->height);
	pix->colorspace = V4L2_COLORSPACE_SRGB;
}

/* Snap @pix to the closest entry of the format list. */
static void stitcher_out_try(const struct stitcher_out *out,
			     struct v4l2_pix_format *pix,
			     const struct stitcher_format **pfmt,
			     const struct stitcher_frmsize **psize)
{
	const struct stitcher_format *fmt;
	const struct stitcher_frmsize *size;

	fmt = stitcher_out_find_format(out, pix->pixelformat);
	if (!fmt)
		fmt = out->cur_fmt;
	size = stitcher_format_nearest_size(fmt, pix->width, pix->height);

	stitcher_fill_pix(fmt, size, pix);
	*pfmt = fmt;
	*psize = size;
}

static int stitcher_vidioc_querycap(struct file *file, void *fh,
				    struct v4l2_capability *cap)
{
	strscpy(cap->driver, "UVC Stitcher", sizeof(cap->driver));
	strscpy(cap->card, "UVC Stitcher", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:uvc_stitcher", sizeof(cap->bus_info));
	return 0;
}

static int stitcher_vidioc_enum_fmt(struct file *file, void *fh,
				    struct v4l2_fmtdesc *fmtdesc)
{
	struct stitcher_out *out = video_drvdata(file);

	if (fmtdesc->index >= out->num_formats)
		return -EINVAL;

	fmtdesc->pixelformat = out->formats[fmtdesc->index].info->fourcc;
	return 0;
}

static int stitcher_vidioc_g_fmt(struct file *file, void *fh,
				 struct v4l2_format *fmt)
{
	struct stitcher_out *out = video_drvdata(file);

	stitcher_fill_pix(out->cur_fmt, out->cur_size, &fmt->fmt.pix);
	return 0;
}

static int stitcher_vidioc_try_fmt(struct file *file, void *fh,
				   struct v4l2_format *fmt)
{
	struct stitcher_out *out = video_drvdata(file);
	const struct stitcher_format *new_fmt;
	const struct stitcher_frmsize *new_size;

	stitcher_out_try(out, &fmt->fmt.pix, &new_fmt, &new_size);
	return 0;
}

static int stitcher_vidioc_s_fmt(struct file *file, void *fh,
				 struct v4l2_format *fmt)
{
	struct stitcher_out *out = video_drvdata(file);
	const struct stitcher_format *new_fmt;
	const struct stitcher_frmsize *new_size;

	stitcher_out_try(out, &fmt->fmt.pix, &new_fmt, &new_size);

	if (new_fmt == out->cur_fmt && new_size == out->cur_size)
		return 0;

	/* Buffer size depends on the format */
	if (vb2_is_busy(&out->q.vbq))
		return -EBUSY;

	out->cur_fmt = new_fmt;
	out->cur_size = new_size;
	out->timeperframe =
		stitcher_frmsize_nearest_ival(new_size, &out->timeperframe);

	pr_info("uvc_stitcher: s_fmt %ux%u %p4cc (src %ux%u)\n",
		new_size->width, new_size->height, &new_fmt->info->fourcc,
		new_size->width, new_size->height / 2);
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
	struct stitcher_out *out = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(&a->parm.capture, 0, sizeof(a->parm.capture));
	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.timeperframe = out->timeperframe;
	return 0;
}

static int stitcher_vidioc_s_parm(struct file *file, void *fh,
				  struct v4l2_streamparm *a)
{
	struct stitcher_out *out = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* The sources' frame rate is only set when streaming starts */
	if (!vb2_is_streaming(&out->q.vbq) &&
	    a->parm.capture.timeperframe.numerator &&
	    a->parm.capture.timeperframe.denominator) {
		out->timeperframe = stitcher_frmsize_nearest_ival(
			out->cur_size, &a->parm.capture.timeperframe);
		pr_info("uvc_stitcher: s_parm fps=%u/%u\n",
			out->timeperframe.denominator,
			out->timeperframe.numerator);
	}

	/* Read back what we stored */
	return stitcher_vidioc_g_parm(file, fh, a);
}

static int stitcher_vidioc_enum_framesizes(struct file *file, void *fh,
					   struct v4l2_frmsizeenum *fsize)
{
	struct stitcher_out *out = video_drvdata(file);
	const struct stitcher_format *fmt;

	fmt = stitcher_out_find_format(out, fsize->pixel_format);
	if (!fmt || fsize->index >= fmt->num_sizes)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fmt->sizes[fsize->index].width;
	fsize->discrete.height = fmt->sizes[fsize->index].height;
	return 0;
}

static int stitcher_vidioc_enum_frameintervals(struct file *file, void *fh,
					       struct v4l2_frmivalenum *fival)
{
	struct stitcher_out *out = video_drvdata(file);
	const struct stitcher_format *fmt;
	const struct stitcher_frmsize *size;

	fmt = stitcher_out_find_format(out, fival->pixel_format);
	if (!fmt)
		return -EINVAL;
	size = stitcher_format_find_size(fmt, fival->width, fival->height);
	if (!size || fival->index >= size->num_ivals)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = size->ivals[fival->index];
	return 0;
}

static const struct v4l2_ioctl_ops stitcher_ioctl_ops = {
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

	.vidioc_streamon             = vb2_ioctl_streamon,
	.vidioc_streamoff            = vb2_ioctl_streamoff,
	// clang-format on
};

static const struct v4l2_file_operations stitcher_fops = {
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

/* ------------------------------------------------------------------ */
/* Output node lifetime                                               */
/* ------------------------------------------------------------------ */
/* Add one S-side size (doubled height) with its frame intervals. */
static void stitcher_out_add_size(struct stitcher_slot *src,
				  struct stitcher_format *fmt, u32 width,
				  u32 src_height)
{
	u32 fourcc = fmt->info->fourcc;
	struct stitcher_frmsize *size;
	struct v4l2_frmivalenum fival;
	u64 frame_size;
	unsigned int i;

	if (!width || !src_height || src_height > U32_MAX / 2)
		return;

	frame_size = stitcher_image_size(fmt->info, width, src_height * 2);
	if (frame_size > INT_MAX || (frame_size & 1)) {
		pr_warn("uvc_stitcher: %p4cc %ux%u cannot be split in half, skipped\n",
			&fourcc, width, src_height);
		return;
	}

	if (fmt->num_sizes == STITCHER_MAX_FRMSIZES) {
		pr_warn("uvc_stitcher: too many %p4cc sizes, %ux%u skipped\n",
			&fourcc, width, src_height);
		return;
	}

	size = &fmt->sizes[fmt->num_sizes];
	memset(size, 0, sizeof(*size));
	size->width = width;
	size->height = src_height * 2;

	for (i = 0; i < STITCHER_ENUM_LIMIT; i++) {
		memset(&fival, 0, sizeof(fival));
		fival.index = i;
		fival.pixel_format = fourcc;
		fival.width = width;
		fival.height = src_height;

		if (UVC_CALL_OP(src, vidioc_enum_frameintervals, src->filp,
				src->fh, &fival))
			break;

		if (fival.type != V4L2_FRMIVAL_TYPE_DISCRETE) {
			pr_warn("uvc_stitcher: %p4cc %ux%u: non-discrete intervals not supported\n",
				&fourcc, width, src_height);
			break;
		}
		if (!fival.discrete.numerator || !fival.discrete.denominator)
			continue;
		if (size->num_ivals == STITCHER_MAX_FRMIVALS) {
			pr_warn("uvc_stitcher: %p4cc %ux%u: too many intervals\n",
				&fourcc, width, src_height);
			break;
		}
		size->ivals[size->num_ivals++] = fival.discrete;
	}

	if (!size->num_ivals) {
		pr_warn("uvc_stitcher: %p4cc %ux%u has no frame interval, skipped\n",
			&fourcc, width, src_height);
		return;
	}

	fmt->num_sizes++;
	pr_info("uvc_stitcher:   %p4cc %ux%u -> %ux%u, %u interval(s)\n",
		&fourcc, width, src_height, size->width, size->height,
		size->num_ivals);
}

static void stitcher_out_add_sizes(struct stitcher_slot *src,
				   struct stitcher_format *fmt)
{
	u32 fourcc = fmt->info->fourcc;
	struct v4l2_frmsizeenum fsize;
	unsigned int i;

	for (i = 0; i < STITCHER_ENUM_LIMIT; i++) {
		memset(&fsize, 0, sizeof(fsize));
		fsize.index = i;
		fsize.pixel_format = fourcc;

		if (UVC_CALL_OP(src, vidioc_enum_framesizes, src->filp,
				src->fh, &fsize))
			break;

		if (fsize.type != V4L2_FRMSIZE_TYPE_DISCRETE) {
			pr_warn("uvc_stitcher: %p4cc: non-discrete sizes not supported\n",
				&fourcc);
			break;
		}
		stitcher_out_add_size(src, fmt, fsize.discrete.width,
				      fsize.discrete.height);
	}
}

/* Copy the S-side format list: same width, twice the height. */
static int stitcher_out_build_formats(struct uvc_stitcher *stitcher,
				      struct stitcher_out *out)
{
	struct stitcher_slot *src = &stitcher->src[STITCHER_SRC_S];
	const struct stitcher_pixfmt *info;
	struct stitcher_format *fmt;
	struct v4l2_fmtdesc fdesc;
	unsigned int i;

	for (i = 0; i < STITCHER_ENUM_LIMIT; i++) {
		memset(&fdesc, 0, sizeof(fdesc));
		fdesc.index = i;
		fdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (UVC_CALL_OP(src, vidioc_enum_fmt_vid_cap, src->filp,
				src->fh, &fdesc))
			break;

		info = stitcher_pixfmt_find(fdesc.pixelformat);
		if (!info) {
			pr_info("uvc_stitcher: S-side format %p4cc not supported, skipped\n",
				&fdesc.pixelformat);
			continue;
		}
		if (stitcher_out_find_format(out, info->fourcc))
			continue;

		/* Unique entries of stitcher_pixfmts[], so this cannot overflow */
		fmt = &out->formats[out->num_formats];
		fmt->info = info;
		fmt->num_sizes = 0;
		stitcher_out_add_sizes(src, fmt);
		if (fmt->num_sizes)
			out->num_formats++;
	}

	if (!out->num_formats) {
		pr_err("uvc_stitcher: S side reports no usable format\n");
		return -ENODEV;
	}
	return 0;
}

/* Default: the largest NV12 size at its first frame interval. */
static void stitcher_out_set_default_format(struct stitcher_out *out)
{
	const struct stitcher_format *fmt;
	const struct stitcher_frmsize *size;
	u32 fourcc = STITCHER_DEFAULT_PIXFMT;
	unsigned int i;

	fmt = stitcher_out_find_format(out, fourcc);
	if (!fmt) {
		fmt = &out->formats[0];
		pr_warn("uvc_stitcher: S side has no %p4cc, default is %p4cc\n",
			&fourcc, &fmt->info->fourcc);
	}

	size = &fmt->sizes[0];
	for (i = 1; i < fmt->num_sizes; i++)
		if ((u64)fmt->sizes[i].width * fmt->sizes[i].height >
		    (u64)size->width * size->height)
			size = &fmt->sizes[i];

	out->cur_fmt = fmt;
	out->cur_size = size;
	out->timeperframe = size->ivals[0];
}

static void stitcher_out_free(struct kref *ref)
{
	struct stitcher_out *out = container_of(ref, struct stitcher_out, ref);

	/* No file handle is left, so nothing else can hold q.mutex */
	mutex_lock(&out->q.mutex);
	vb2_queue_release(&out->q.vbq);
	mutex_unlock(&out->q.mutex);
	mutex_destroy(&out->q.mutex);
	kfree(out);
}

/*
 * Runs when the last reference to the video device is dropped: from
 * video_unregister_device() if the node is not open, otherwise from the
 * last close(), during which the char device still pins this module.
 */
static void stitcher_out_vdev_release(struct video_device *vdev)
{
	struct stitcher_out *out = container_of(vdev, struct stitcher_out, vdev);

	out->vdev_released = true;
	kref_put(&out->ref, stitcher_out_free);
}

static void stitcher_out_init_vdev(struct uvc_stitcher *stitcher,
				   struct stitcher_out *out)
{
	struct video_device *vdev = &out->vdev;

	vdev->fops = &stitcher_fops;
	vdev->ioctl_ops = &stitcher_ioctl_ops;
	vdev->release = stitcher_out_vdev_release;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE;
	strscpy(vdev->name, "UVC Stitch", sizeof(vdev->name));
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
	vdev->lock = &out->q.mutex;

	vdev->queue = &out->q.vbq;
	vdev->v4l2_dev = &stitcher->v4l2_dev;
	video_set_drvdata(vdev, out);
}

/**
 * stitcher_out_register - expose the stitched node.
 *
 * Called with disc_lock held once both sources are open.
 */
static void stitcher_out_register(struct uvc_stitcher *stitcher)
{
	struct stitcher_out *out;
	int ret;

	lockdep_assert_held(&stitcher->disc_lock);

	out = kzalloc(sizeof(*out), GFP_KERNEL);
	if (!out)
		return;
	out->stitcher = stitcher;

	pr_info("uvc_stitcher: building format list from S side\n");
	ret = stitcher_out_build_formats(stitcher, out);
	if (ret) {
		kfree(out);
		return;
	}
	stitcher_out_set_default_format(out);

	ret = init_queue(&out->q);
	if (ret) {
		pr_err("uvc_stitcher: init_queue failed: %d\n", ret);
		mutex_destroy(&out->q.mutex);
		kfree(out);
		return;
	}
	stitcher_out_init_vdev(stitcher, out);

	/*
	 * One reference for this function and one dropped by vdev->release.
	 * A failed video_register_device() does not call vdev->release
	 * before v6.17; since v6.17 a device_register() failure does.
	 */
	kref_init(&out->ref);
	kref_get(&out->ref);

	ret = video_register_device(&out->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		pr_err("uvc_stitcher: video_register_device failed: %d\n", ret);
		if (!out->vdev_released)
			kref_put(&out->ref, stitcher_out_free);
		kref_put(&out->ref, stitcher_out_free);
		return;
	}

	stitcher->out = out;
	pr_info("uvc_stitcher: P and S connected, registered %s (%ux%u %p4cc)\n",
		video_device_node_name(&out->vdev), out->cur_size->width,
		out->cur_size->height, &out->cur_fmt->info->fourcc);

	/* The registered device keeps out alive until it is released */
	kref_put(&out->ref, stitcher_out_free);
}

/*
 * Detach the node from the sources: stop the engine if the node owns it
 * and fail every buffer so a blocked DQBUF or poll returns.
 */
static void stitcher_out_disconnect(struct uvc_stitcher *stitcher,
				    struct stitcher_out *out)
{
	mutex_lock(&out->q.mutex);
	out->disconnected = true;

	mutex_lock(&stitcher->uvc_ctrl_lock);
	if (stitcher->engine_out == out)
		stitcher_engine_stop(stitcher);
	mutex_unlock(&stitcher->uvc_ctrl_lock);

	vb2_queue_error(&out->q.vbq);
	stitcher_out_return_buffers(out, VB2_BUF_STATE_ERROR);
	mutex_unlock(&out->q.mutex);
}

/* Remove the registered node.  Called with disc_lock held. */
static void stitcher_out_remove(struct uvc_stitcher *stitcher)
{
	struct stitcher_out *out = stitcher->out;

	lockdep_assert_held(&stitcher->disc_lock);

	if (!out)
		return;

	stitcher->out = NULL;
	pr_info("uvc_stitcher: removing %s\n",
		video_device_node_name(&out->vdev));

	stitcher_out_disconnect(stitcher, out);

	/* May drop the last reference; out must not be used afterwards */
	video_unregister_device(&out->vdev);
}

/* ------------------------------------------------------------------ */
/* Source discovery                                                   */
/* ------------------------------------------------------------------ */
/**
 * stitcher_match_device - return true if vdev is one of target devices.
 *
 * Matches the uvcvideo capture node of the EJ535 P or S port.
 */
static bool stitcher_match_device(struct video_device *vdev, bool verbose)
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
		if (verbose)
			pr_info("uvc_stitcher: ignoring USB device with VID=0x%04X\n",
				le16_to_cpu(udev->descriptor.idVendor));
		return false;
	}

	pid = le16_to_cpu(udev->descriptor.idProduct);
	if (pid != STITCHER_DEFAULT_PID_P && pid != STITCHER_DEFAULT_PID_S) {
		if (verbose)
			pr_info("uvc_stitcher: ignoring USB device with PID=0x%04X\n",
				pid);
		return false;
	}

	return true;
}

/*
 * The class add_dev callback runs inside device_add(), before the V4L2 core
 * marks the node registered, so this work can run before the node may be
 * opened.  Wait for it, unless the device is removed in the meantime.
 */
static bool stitcher_wait_registered(struct video_device *vdev)
{
	unsigned int waited = 0;

	while (!video_is_registered(vdev)) {
		if (!device_is_registered(&vdev->dev) ||
		    waited >= STITCHER_REG_WAIT_MS)
			return false;
		msleep(STITCHER_REG_POLL_MS);
		waited += STITCHER_REG_POLL_MS;
	}
	return true;
}

static void stitcher_src_add(struct uvc_stitcher *stitcher,
			     struct video_device *vdev)
{
	struct usb_interface *intf = to_usb_interface(vdev->v4l2_dev->dev);
	struct usb_device *udev = interface_to_usbdev(intf);
	u16 pid = le16_to_cpu(udev->descriptor.idProduct);
	struct stitcher_slot *src;
	struct inode *inode;
	char devpath[32];
	struct file *filp;
	int slot;

	if (pid == STITCHER_DEFAULT_PID_P)
		slot = STITCHER_SRC_P;
	else if (pid == STITCHER_DEFAULT_PID_S)
		slot = STITCHER_SRC_S;
	else
		return;

	/* Wait without holding disc_lock; module exit takes it */
	if (!stitcher_wait_registered(vdev)) {
		pr_info("uvc_stitcher: %s went away before it could be opened\n",
			video_device_node_name(vdev));
		return;
	}

	mutex_lock(&stitcher->disc_lock);

	if (stitcher->exiting)
		goto out_unlock;
	if (stitcher->src[slot].filp) {
		pr_info("uvc_stitcher: slot%d already in use, ignoring %s\n",
			slot, video_device_node_name(vdev));
		goto out_unlock;
	}

	/* filp_open - Open without O_NONBLOCK. */
	snprintf(devpath, sizeof(devpath), "/dev/video%d", vdev->num);
	filp = filp_open(devpath, O_RDWR, 0);
	if (IS_ERR(filp)) {
		pr_err("uvc_stitcher: filp_open(%s): %ld\n", devpath,
		       PTR_ERR(filp));
		goto out_unlock;
	}

	/*
	 * The node number may have been reused by another device by now.
	 * video_devdata() indexes a 256-entry array by minor, so check the
	 * file is a V4L2 char device first.
	 */
	inode = file_inode(filp);
	if (!S_ISCHR(inode->i_mode) || imajor(inode) != VIDEO_MAJOR ||
	    iminor(inode) >= STITCHER_V4L2_MINORS ||
	    video_devdata(filp) != vdev) {
		pr_err("uvc_stitcher: %s is no longer the slot%d device\n",
		       devpath, slot);
		filp_close(filp, NULL);
		goto out_unlock;
	}

	src = &stitcher->src[slot];
	src->filp = filp;
	src->vdev = vdev;
	src->fh = filp->private_data;
	src->intf = intf;
	stitcher->src_ready_count++;

	pr_info("uvc_stitcher: slot%d filled (%s), ready=%d/%d\n", slot,
		devpath, stitcher->src_ready_count, STITCHER_NUM_SOURCES);

	if (stitcher->src_ready_count == STITCHER_NUM_SOURCES)
		stitcher_out_register(stitcher);

out_unlock:
	mutex_unlock(&stitcher->disc_lock);
}

static void stitcher_src_remove(struct uvc_stitcher *stitcher,
				struct video_device *vdev)
{
	int i;

	mutex_lock(&stitcher->disc_lock);

	for (i = 0; i < STITCHER_NUM_SOURCES; i++)
		if (stitcher->src[i].vdev == vdev)
			break;
	if (i == STITCHER_NUM_SOURCES)
		goto out_unlock;

	/* Hide the stitched node first; it also stops the engine */
	stitcher_out_remove(stitcher);

	filp_close(stitcher->src[i].filp, NULL);
	memset(&stitcher->src[i], 0, sizeof(stitcher->src[i]));
	stitcher->src_ready_count--;
	pr_info("uvc_stitcher: slot%d removed, ready=%d/%d\n", i,
		stitcher->src_ready_count, STITCHER_NUM_SOURCES);

out_unlock:
	mutex_unlock(&stitcher->disc_lock);
}

/**
 * stitcher_disc_work_fn - workqueue function for device discovery/removal.
 *
 * Runs on an ordered workqueue so the class_interface callbacks never
 * block the caller (e.g., uvcvideo) and add/remove are handled in order.
 */
static void stitcher_disc_work_fn(struct work_struct *w)
{
	struct stitcher_disc_work *dw =
		container_of(w, struct stitcher_disc_work, work);

	if (dw->is_add)
		stitcher_src_add(dw->stitcher, dw->vdev);
	else
		stitcher_src_remove(dw->stitcher, dw->vdev);

	put_device(&dw->vdev->dev);
	kfree(dw);
}

static void stitcher_schedule_disc_work(struct uvc_stitcher *stitcher,
					struct video_device *vdev, bool is_add)
{
	struct stitcher_disc_work *dw;

	dw = kzalloc(sizeof(*dw), GFP_KERNEL);
	if (!dw) {
		pr_err("uvc_stitcher: cannot handle %s %s: out of memory\n",
		       is_add ? "add of" : "removal of",
		       video_device_node_name(vdev));
		return;
	}

	get_device(&vdev->dev);
	INIT_WORK(&dw->work, stitcher_disc_work_fn);
	dw->stitcher = stitcher;
	dw->vdev = vdev;
	dw->is_add = is_add;

	queue_work(stitcher->disc_wq, &dw->work);
}

/*
 * class_interface callbacks run under the class mutex (and, on add, under
 * videodev_lock), so they only queue work.  The stitched node never
 * matches: its v4l2_device has no parent device.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
static int stitcher_class_add_dev(struct device *dev,
				  struct class_interface *ci)
#else
static int stitcher_class_add_dev(struct device *dev)
#endif
{
	struct video_device *vdev = to_video_device(dev);

	if (stitcher_match_device(vdev, true))
		stitcher_schedule_disc_work(g_stitcher, vdev, true);
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
static void stitcher_class_remove_dev(struct device *dev,
				      struct class_interface *ci)
#else
static void stitcher_class_remove_dev(struct device *dev)
#endif
{
	struct video_device *vdev = to_video_device(dev);

	/*
	 * Queue the removal even if the add work has not run yet; the
	 * ordered workqueue runs it after the add.
	 */
	if (stitcher_match_device(vdev, false))
		stitcher_schedule_disc_work(g_stitcher, vdev, false);
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

/*
 * video_class is not exported.  Register a short-lived v4l-subdev node
 * only to read its class pointer: applications that list /dev/video*
 * never see it, and open() on it always fails.
 */
static struct video_device stitcher_probe_vdev;
static DECLARE_COMPLETION(stitcher_probe_released);

static int stitcher_probe_open(struct file *file)
{
	return -ENODEV;
}

static int stitcher_probe_release(struct file *file)
{
	return 0;
}

static const struct v4l2_file_operations stitcher_probe_fops = {
	.owner = THIS_MODULE,
	.open = stitcher_probe_open,
	.release = stitcher_probe_release,
};

static void stitcher_probe_vdev_release(struct video_device *vdev)
{
	complete(&stitcher_probe_released);
}

static STITCHER_CLASS_CONST struct class *
stitcher_find_video_class(struct uvc_stitcher *stitcher)
{
	struct video_device *vdev = &stitcher_probe_vdev;
	STITCHER_CLASS_CONST struct class *vcls;
	int ret;

	vdev->v4l2_dev = &stitcher->v4l2_dev;
	vdev->fops = &stitcher_probe_fops;
	vdev->release = stitcher_probe_vdev_release;
	strscpy(vdev->name, "uvc_stitcher probe", sizeof(vdev->name));

	ret = video_register_device(vdev, VFL_TYPE_SUBDEV, -1);
	if (ret) {
		pr_err("uvc_stitcher: probe node registration failed: %d\n",
		       ret);
		return ERR_PTR(ret);
	}

	vcls = vdev->dev.class;
	video_unregister_device(vdev);

	/*
	 * vdev is static: make sure its release has run before the module
	 * can go away.  Only an open() racing with the unregister can hold
	 * a reference, and that open() fails immediately.
	 */
	wait_for_completion(&stitcher_probe_released);

	return vcls ? vcls : ERR_PTR(-ENODEV);
}

/* ------------------------------------------------------------------ */
/* Module init / exit                                                 */
/* ------------------------------------------------------------------ */
static int stitcher_add(void)
{
	STITCHER_CLASS_CONST struct class *vcls;
	struct uvc_stitcher *stitcher;
	int i, s, ret;

	pr_info("uvc_stitcher: initializing stitcher...\n");
	stitcher = kzalloc(sizeof(*stitcher), GFP_KERNEL);
	if (!stitcher) {
		pr_err("uvc_stitcher: failed to allocate memory for stitcher\n");
		return -ENOMEM;
	}

	mutex_init(&stitcher->disc_lock);
	mutex_init(&stitcher->uvc_ctrl_lock);
	mutex_init(&stitcher->src_stream_lock);
	init_completion(&stitcher->engine_started);
	atomic_set(&stitcher->streaming, 0);
	for (i = 0; i < STITCHER_MAX_OUT_BUFS; i++)
		for (s = 0; s < STITCHER_NUM_SOURCES; s++)
			stitcher->dma_slots[i].cached_fd[s] = -1;

	stitcher->disc_wq = alloc_ordered_workqueue("uvc_stitcher_disc", 0);
	if (!stitcher->disc_wq) {
		pr_err("uvc_stitcher: failed to allocate workqueue\n");
		ret = -ENOMEM;
		goto err_free;
	}

	strscpy(stitcher->v4l2_dev.name, "uvc_stitcher",
		sizeof(stitcher->v4l2_dev.name));
	ret = v4l2_device_register(NULL, &stitcher->v4l2_dev);
	if (ret) {
		pr_err("uvc_stitcher: v4l2_device_register failed: %d\n", ret);
		goto err_wq;
	}

	vcls = stitcher_find_video_class(stitcher);
	if (IS_ERR(vcls)) {
		ret = PTR_ERR(vcls);
		goto err_v4l2;
	}

	g_stitcher = stitcher;
	ret = stitcher_register_class_intf(stitcher, vcls);
	if (ret) {
		pr_err("uvc_stitcher: failed to register class interface: %d\n",
		       ret);
		g_stitcher = NULL;
		goto err_v4l2;
	}

	pr_info("uvc_stitcher: waiting for EJ535 P and S\n");
	return 0;

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

	mutex_lock(&stitcher->disc_lock);
	stitcher->exiting = true;
	mutex_unlock(&stitcher->disc_lock);

	/* Calls remove_dev for every device, which queues P/S removal */
	stitcher_unregister_class_intf(stitcher);

	drain_workqueue(stitcher->disc_wq);
	destroy_workqueue(stitcher->disc_wq);
	stitcher->disc_wq = NULL;

	/* Normally already done by the removal work */
	mutex_lock(&stitcher->disc_lock);
	stitcher_out_remove(stitcher);
	for (i = 0; i < STITCHER_NUM_SOURCES; i++) {
		if (!IS_ERR_OR_NULL(stitcher->src[i].filp)) {
			pr_info("uvc_stitcher: closing src%d...\n", i);
			filp_close(stitcher->src[i].filp, NULL);
		}
		memset(&stitcher->src[i], 0, sizeof(stitcher->src[i]));
	}
	stitcher->src_ready_count = 0;
	mutex_unlock(&stitcher->disc_lock);

	v4l2_device_unregister(&stitcher->v4l2_dev);
	g_stitcher = NULL;
	kfree(stitcher);
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
