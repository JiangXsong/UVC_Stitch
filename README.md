# UVC_Stitch

UVC_Stitch is a Linux kernel driver that combines the video streams from two
USB cameras into a single virtual camera device. The two camera feeds are
stitched top-to-bottom and exposed as one `/dev/videoN` capture device,
which can be used directly in OBS Studio, ffmpeg, or any other V4L2-compatible
application — no additional software or configuration required.

The virtual camera is available as soon as the driver is loaded. The two
physical cameras are detected automatically when plugged in. Streaming starts
once both cameras are connected and ready.

# Hardware Requirements

| Parameter         | Value                                                         |
|-------------------|---------------------------------------------------------------|
| Pixel Format      | NV12                                                          |
| Stitch Mode       | Top-Bottom                                                    |
| Output Resolution | Up to 3840×2160 (4K) @ 60 fps                                |
| USB Ports         | Two separate USB 3.0 ports (one per camera)                   |
| Host Interface    | PCIe Gen 3 ×2 or above (minimum 10 Gbps) for stable 4K@60fps |

Both cameras must be the same model and produce the same resolution.

For best performance, we recommend a CPU equivalent to Intel 12th Gen Core i7
or AMD Ryzen 7 5000 series and above. Older CPUs (e.g. Intel 7th Gen Core i5
or earlier) can run the driver but will see significantly higher CPU utilization.

# Kernel / Software Requirements

## Linux Kernel

The following kernel versions have been validated:

- 5.15.x
- 6.8.x
- 6.14.x

## Build Dependencies

The following tools are required to build the kernel module:

- `make`
- `gcc`
- Kernel headers matching your running kernel (`linux-headers-$(uname -r)`)

On Ubuntu/Debian, install them with:

```bash
sudo apt install build-essential linux-headers-$(uname -r)
```

## Required Kernel Modules

The following kernel modules must be loaded before using EJ_Stitcher.
They are included in most standard Linux distributions and are typically
loaded automatically:

- `uvcvideo` — UVC camera driver
- `videobuf2` — V4L2 buffer management framework

## Verified Consumer Applications

The following applications have been tested and work out of the box with
the EJ_Stitcher output device:

- [OBS Studio](https://obsproject.com/)
- [ffmpeg](https://ffmpeg.org/)
- [ffplay](https://ffmpeg.org/ffplay.html)

# Build

Before building, make sure the build dependencies listed above are installed.

To compile the driver, run:

```bash
$ make
```

This produces the kernel module file `uvc_stitch.ko` in the current directory.

## Rebuilding

If you make changes to the source or switch kernel versions, clean the previous
build before recompiling:

```bash
$ make clean
$ make
```

# Install

Installing the driver makes it available system-wide via `modprobe`, so you
do not need to specify the full path each time. Root privileges are required.

```bash
$ make && sudo make install
```

# Load the Driver

Once installed, load the driver with:

```bash
$ sudo modprobe uvc_stitch
```

The virtual camera device (`/dev/videoN`) is created immediately. Plug in both
cameras and they will be detected automatically.

To unload the driver:

```bash
$ sudo modprobe -r uvc_stitch
```

# Format Source Files

Requires `clang-format` to be installed.

```bash
$ make clang-format
```

---

# Usage

## Check the Output Device

After loading the driver and connecting both cameras, verify the virtual device
was created:

```bash
$ v4l2-ctl --list-devices
```

View detailed information about the output device:

```bash
$ v4l2-ctl -d /dev/videoN --info
```

Query the supported video formats:

```bash
$ v4l2-ctl -d /dev/videoN --list-formats-ext
```

Replace `/dev/videoN` with the actual device path shown by `--list-devices`.

## Preview

Preview the live stitched stream:

```bash
$ ffplay -f v4l2 -input_format nv12 -video_size 3840x2160 -i /dev/videoN
```

Alternatively, add the device as a Video Capture Source in OBS Studio.

# Default Parameters

The driver uses the following defaults. These can be changed before starting
the stream via `VIDIOC_S_FMT` (resolution/format) and `VIDIOC_S_PARM` (framerate).

| Parameter    | Default | Description                                   |
|--------------|---------|-----------------------------------------------|
| `out_width`  | 3840    | Output frame width (pixels)                   |
| `out_height` | 2160    | Output frame height (= per-source height × 2) |
| `src_width`  | 3840    | Per-camera input width                        |
| `src_height` | 1080    | Per-camera input height                       |
| `fps_num`    | 1       | Frame interval numerator                      |
| `fps_den`    | 60      | Frame interval denominator (60 fps)           |

---

# How It Works — State Machine

The driver moves through the following states from load to streaming:

```
IDLE  ──modprobe──►  video node created
  │
  │  UVC plug-in (×2, VID/PID match)
  │  src_ready_count: 0 → 1 → 2
  │
  ▼
IDLE (slots filled)
  │
  │  VIDIOC_STREAMON
  │    src_ready_count < 2  ──► return -ENODEV
  │    src_ready_count == 2
  │      UVC S_FMT / REQBUFS / QBUF / STREAMON
  │      kthread_run()
  ▼
STREAMING
  │
  ├── VIDIOC_STREAMOFF ──► mixer_uvc_stop() → IDLE (slots kept)
  ├── UVC unplug       ──► mixer_uvc_stop() → IDLE (slot cleared)
  └── rmmod            ──► mixer_uvc_stop() → unregister → gone
```

**In short:** loading the driver creates the virtual camera immediately.
Streaming only starts once both physical cameras are connected and an
application requests the stream. Unplugging a camera or stopping the
stream returns the driver to idle — it does not need to be reloaded.

# Known Limitations

- The target camera model (USB Vendor ID / Product ID) is fixed at compile
  time and cannot be changed without rebuilding the driver.
- Only the NV12 pixel format is supported. 

# License

GPL-2.0-only
