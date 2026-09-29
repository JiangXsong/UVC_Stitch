# UVC Stitch

For a guided setup, open [QuickStart.html](QuickStart.html).

UVC Stitch is a Linux kernel driver for the EJ535 IC, which has two USB ports
named P and S. The IC generates a complete frame and splits it into two halves
(top and bottom); each half is sent through one USB port. The driver reassembles
the two halves into one frame and exposes one `/dev/videoN` capture device for
OBS Studio, ffmpeg, and other V4L2-compatible applications. No separate
user-space stitching process is needed.

The virtual camera is available as soon as the driver is loaded. The UVC devices
from the EJ535's P and S ports are detected when connected. Streaming starts
when both are ready and an application requests video.

# Hardware Requirements

| Parameter         | Value                                                         |
|-------------------|---------------------------------------------------------------|
| 4K60 Pixel Format | NV12                                                          |
| Output Layout     | Top-Bottom (one half of each frame per USB port)              |
| Output Resolution | Up to 3840×2160 (4K) @ 60 fps                                |
| Host USB Ports    | Two separate USB 3.2 Gen 1 (5 Gbps, also labeled USB 3.0) ports, one for each EJ535 USB port |
| Host Interface    | PCIe Gen 3 ×2 or above; at least 10 Gbps of bandwidth is required for stable 4K@60fps |

The driver only recognizes the EJ535 P/S pair with USB VID `1E4E` and PIDs
`7301` (P) and `7302` (S). Both connections are required. For stable 4K60,
select NV12 explicitly in the capture application.

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

The following kernel modules must be loaded before using UVC Stitch.
They are included in most standard Linux distributions and are typically
loaded automatically:

- `uvcvideo` — UVC camera driver
- `videobuf2` — V4L2 buffer management framework

## Secure Boot

The driver is built locally and is not signed by default. On a computer with
Secure Boot enabled, loading it fails with:

```
modprobe: ERROR: could not insert 'uvc_stitch': Key was rejected by service
```

Check the current state:

```bash
$ mokutil --sb-state
```

If Secure Boot is enabled, use one of these options:

- **Disable Secure Boot** in the computer's firmware (BIOS/UEFI) setup. This is
  the simplest option.
- **Sign the module** with a Machine Owner Key (MOK). Build the module, sign
  it, then install it:

  ```bash
  $ openssl req -new -x509 -newkey rsa:2048 -nodes -days 36500 \
        -subj "/CN=UVC Stitch module signing/" \
        -keyout MOK.priv -outform DER -out MOK.der
  $ sudo mokutil --import MOK.der       # set a one-time password, then reboot
                                        # and choose "Enroll MOK"
  $ make
  $ /usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 \
        MOK.priv MOK.der uvc_stitch.ko
  $ sudo make modules_install
  ```

  Keep `MOK.priv` private. Sign again after every rebuild.

## Verified Consumer Applications

The following applications have been tested with the UVC Stitch output device:

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

The module is built for one specific kernel version. After a kernel update,
rebuild and reinstall it (see [Rebuilding](#rebuilding)); a reboot alone does
not require a reinstall.

# Load the Driver

Once installed, load the driver with:

```bash
$ sudo modprobe uvc_stitch
```

The virtual camera device (`/dev/videoN`) is created immediately. Connect both
EJ535 USB ports, P and S, to the host to make both inputs available.

To unload the driver:

```bash
$ sudo modprobe -r uvc_stitch
```

## Load Automatically at Boot

The driver is not loaded automatically after a reboot. Run `sudo modprobe
uvc_stitch` again after each reboot, or enable auto-load once:

```bash
$ sudo make autoload
```

This creates `/etc/modules-load.d/uvc_stitch.conf`. To disable it:

```bash
$ sudo make noautoload
```

# Format Source Files

Requires `clang-format` to be installed.

```bash
$ make clang-format
```

---

# Usage

## Check the Output Device

After loading the driver and connecting the EJ535's P and S USB ports, verify
the virtual device was created. On Ubuntu/Debian, install `v4l-utils` if
`v4l2-ctl` is missing:

```bash
$ sudo apt install v4l-utils
```

Then list the devices:

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

Preview the live stitched stream (`ffplay` must be installed):

```bash
$ ffplay -f v4l2 -input_format nv12 -video_size 3840x2160 -i /dev/videoN
```

Alternatively, add a Video Capture Device (V4L2) source in OBS Studio and select
the NV12 video format for 4K60.

# Default Parameters

The driver uses the following defaults. These can be changed before starting
the stream via `VIDIOC_S_FMT` (resolution/format) and `VIDIOC_S_PARM` (framerate).

| Parameter    | Default | Description                              |
|--------------|---------|------------------------------------------|
| `out_width`  | 3840    | Output frame width (pixels)              |
| `out_height` | 2160    | Output frame height (pixels)             |
| `out_pixfmt` | YUYV    | Initial format; select NV12 for 4K60    |
| `src_width`  | 3840    | Width requested from each P/S UVC device |
| `src_height` | 1080    | Height requested from each P/S UVC device |
| `fps_num`    | 1       | Frame interval numerator                 |
| `fps_den`    | 60      | Frame interval denominator (60 fps)      |

---

# How It Works — State Machine

The driver moves through the following states from load to streaming:

```
IDLE  ──modprobe──►  video node created
  │
  │  EJ535 P and S ports enumerate as UVC devices (fixed VID/PID match)
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
  ├── VIDIOC_STREAMOFF ──► stitcher_uvc_stop() → IDLE (slots kept)
  ├── UVC unplug       ──► stitcher_uvc_stop() → IDLE (slot cleared)
  └── rmmod            ──► stitcher_uvc_stop() → unregister → gone
```

**In short:** loading the driver creates the virtual camera immediately.
Streaming only starts once both EJ535 USB connections are ready and an
application requests the stream. Disconnecting either port or stopping the
stream returns the driver to idle — it does not need to be reloaded.
