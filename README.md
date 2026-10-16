# UVC Stitch

Linux kernel driver for the EJ535. The EJ535 sends the first half of each frame
over its P port and the second half over its S port; both appear as UVC
cameras. The driver joins the halves in the kernel and exposes one camera,
listed as **UVC Stitcher**, for OBS Studio, ffmpeg and other V4L2 apps.

- The camera appears only while both P and S are connected.
- Its formats come from the S port: same width, twice the height.
- Internals: [architecture diagrams](https://claude.ai/artifact/Pa8GqynunnA9inUouBmGz8)

# Requirements

| Item     | Requirement                                                        |
|----------|--------------------------------------------------------------------|
| Device   | EJ535, VID `1E4E`, PID `7301` (P) and `7302` (S)                   |
| Host USB | Two separate USB 3.2 Gen 1 (5 Gbps) ports, one per EJ535 port      |
| Host bus | PCIe Gen 3 ×2 or better (≥ 10 Gbps for 4K60)                       |
| CPU      | Intel 12th Gen Core i7 / AMD Ryzen 7 5000 or newer recommended     |
| Kernel   | Tested on 7.0; builds on 6.8, 6.11, 6.14, 6.17, 7.0                |
| Build    | `sudo apt install build-essential linux-headers-$(uname -r)`       |

**Firmware notes**

- The frame is split by bytes. For NV12 the split falls inside the Y plane, so
  the S half also carries the UV plane.
- Describe each pixel format only once per port. uvcvideo uses the first
  descriptor with a given fourcc and ignores the rest.

# Build, Install, Load

```bash
make                         # builds uvc_stitch.ko
sudo make install            # install, enables modprobe
sudo modprobe uvc_stitch     # load
sudo modprobe -r uvc_stitch  # unload (close apps using the camera first)
sudo make autoload           # load at boot (noautoload to undo)
```

Rebuild with `make clean && make` after a kernel update or source change.
Other targets: `uninstall`, `reload`, `clang-format`, `clang-format-check`.

**Secure Boot:** an unsigned module fails with `Key was rejected by service`.
Either disable Secure Boot, or sign the module with a MOK:

```bash
openssl req -new -x509 -newkey rsa:2048 -nodes -days 36500 \
    -subj "/CN=UVC Stitch module signing/" -keyout MOK.priv -outform DER -out MOK.der
sudo mokutil --import MOK.der   # reboot and choose "Enroll MOK"
make
/usr/src/linux-headers-$(uname -r)/scripts/sign-file sha256 MOK.priv MOK.der uvc_stitch.ko
sudo make modules_install
```

# Usage

```bash
v4l2-ctl --list-devices                        # find /dev/videoN (UVC Stitcher)
v4l2-ctl -d /dev/videoN --list-formats-ext
ffplay -f v4l2 -input_format nv12 -video_size 3840x2160 -i /dev/videoN
```

In OBS Studio, add a "Video Capture Device (V4L2)" source and pick UVC Stitcher.
The device number may change after replugging.

# Formats

Listed formats: NV12, YUYV, UYVY, BGR3 (whatever the S port offers). Default:
NV12 at the largest size, 60 fps. P is set to the same format as S.

Data per port at 3840×2160, 60 fps, against about 500 MB/s for a 5 Gbps port:

| Format     | Per port | 4K60         |
|------------|----------|--------------|
| NV12       | 373 MB/s | fits         |
| YUYV, UYVY | 498 MB/s | no headroom  |
| BGR3       | 746 MB/s | exceeds link |

Use NV12 for 4K60. Every format fits at 1920×1080 and below.

# How It Works

```
WAITING ──(P and S connected)──► READY: node registered, formats copied from S
READY ──(STREAMON)──► STREAMING: kthread pairs the halves via DMABUF
STREAMING ──(P or S stops)──► both ports restarted ──► STREAMING
STREAMING ──(STREAMOFF)──► READY
any ──(P or S unplugged)──► WAITING: node removed
```

## Automatic Restart

The EJ535 sometimes fails all USB transfers on one port right after streaming
starts (`Non-zero status (-71)` in `dmesg`); this also happens without this
driver. uvcvideo then stops that port for good, so the driver restarts both
ports when one of them:

- returns cancelled buffers (restart at once),
- returns two corrupt frames in a row, or
- stays silent for 250 ms while the other keeps delivering.

The first three restarts are immediate, then 100 ms doubling to 1 s.
`dmesg` shows `recovered N ms after the fault`; measured 48–379 ms
(median 97 ms) on Linux 7.0.

**OBS tip:** turn off "Autoreset on Timeout", or set "Frames Until Timeout" to
30 or more. The default (5 frames ≈ 83 ms) interrupts the driver's recovery.
