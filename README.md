# HwsCapture: Linux driver for AVMatrix VC41 / VC42 PCIe capture cards

Kernel driver (`HwsCapture`) for the AVMatrix VC41 and VC42 HDMI capture cards
and other boards built on the same HWS chip. Each input appears as a V4L2 video
device (`/dev/videoN`, card name `HVideo N`) and an ALSA capture card
(`HAudio N`), so it works with OBS, ffmpeg, GStreamer, VLC and similar apps.

This is a reworked version of the AVMatrix vendor driver, focused on stability
and smooth, low-latency capture. It is based on
[GloriousEggroll/AVMATRIX-VC41-VC42-CAPTURE](https://github.com/GloriousEggroll/AVMATRIX-VC41-VC42-CAPTURE)
and includes the additions from the AVMatrix 2026-05-19 driver release.

> **Status:** compile-tested on the kernels listed below and covered by the
> userspace tests in [`tools/driver-tests`](tools/driver-tests). It has not yet
> been verified on a card with this exact version. See
> [Checking that it works](#checking-that-it-works).

## Supported hardware

The driver binds to these PCI devices (subsystem `8888:0007`). Find yours with
`lspci -nn | grep -iE '1f33|8888'`.

| PCI ID (vendor:device) | Video channels created |
|---|---|
| `8888:9534`, `1f33:6524`, `8888:8524`, `1f33:8524` | 4 |
| `1f33:8534`, `1f33:8554`, `8888:8504`, `1f33:8504`, `8888:6504`, `8888:8521` | 4 |
| `8888:8532` | 2 |
| `8888:8512`, `1f33:6502` | 2 |
| `8888:8501` | 1 |

The VC41 and VC42 are among these IDs; AVMatrix does not publish which product
uses which ID, so check `lspci -nn` on your machine. Every channel gets one
video device and one audio card.

**Input:** up to 1920×1080 progressive, or interlaced up to 1080i.
The card reports the input frame rate; the driver passes frames through at
that rate.

**Output:** YUYV (4:2:2) at 1920×1080, 1680×1050, 1440×900, 1360×768,
1280×1024, 1280×800, 1280×768, 1280×720, 1024×768, 800×600, 720×576, 720×480,
640×480, and 1080×1920 (portrait, rotated). When the output size equals the
input size the frame is copied straight from the card into your buffer;
otherwise the driver scales, pads or crops.

**Audio:** 48 kHz, stereo, 16-bit, one ALSA card per channel.

## Supported kernels and distributions

Builds successfully on Linux **5.15, 6.1, 6.8, 6.12, 6.16, 6.18, 6.19 and
7.0** (x86_64). Kernel versions in between should work as well; older than 5.15
is not tested. Other architectures (e.g. arm64) are not tested.

Any distribution with kernel headers, a compiler and (recommended) DKMS works:
Ubuntu, Debian, Fedora, Arch, openSUSE, Kylin and others.

## Installation

### 1. Install the build prerequisites

You need a compiler, `make`, DKMS and the headers **for the running kernel**:

```sh
# Ubuntu / Debian / Linux Mint / Pop!_OS
sudo apt install build-essential dkms linux-headers-$(uname -r)

# Fedora
sudo dnf install gcc make dkms kernel-devel-$(uname -r)

# Arch / Manjaro (use the headers package matching your kernel)
sudo pacman -S --needed base-devel dkms linux-headers

# openSUSE
sudo zypper install gcc make dkms kernel-devel
```

Check that the headers match: `ls /lib/modules/$(uname -r)/build` must exist.
If you just updated the kernel, reboot first.

### 2. Get the driver

```sh
git clone https://github.com/tischfussball-club-zuerich/avmatrix-pci-card-driver.git
cd avmatrix-pci-card-driver
```

### 3. Install (choose one method)

**Option A: DKMS (recommended).** DKMS rebuilds the driver automatically
whenever you install a new kernel.

```sh
sudo ./dkms-install.sh
```

**Option B: plain install.** Builds and installs for the running kernel only;
you have to run it again after every kernel update.

```sh
sudo ./install.sh
```

**Kylin OS:** use `sudo ./Kylin-install.sh` instead.

Both installers check the prerequisites, remove a previous installation, copy
the driver to `/usr/local/share/HWS`, build and install the module, and load it.
A log is written to `hws_install.log` in the current directory. If an older
version of the module is still loaded, the installer asks whether to reboot;
see [Updating](#updating) to switch without a reboot.

#### Secure Boot

With Secure Boot enabled, the kernel only loads signed modules. On Ubuntu and
Debian, DKMS signs the module with a local key; the first time, it asks you to
set a password and then to enroll the key at the next boot ("Perform MOK
management" → "Enroll MOK"). If loading fails with `Key was rejected by
service`, enroll the key DKMS signed with (`sudo mokutil --import <key>`; the
DKMS output names the key file, usually `/var/lib/dkms/mok.pub` or
`/var/lib/shim-signed/mok/MOK.der`) and reboot, or disable Secure Boot.

### 4. Check the installation

```sh
lsmod | grep HwsCapture            # module loaded
dkms status                        # (DKMS install) shows HwsCapture.ko ... installed
v4l2-ctl --list-devices            # one "HVideo N" entry per channel (package v4l-utils)
arecord -l                         # one "HAudio N" card per channel
sudo dmesg | grep -iE 'hws|MV360'  # driver messages, e.g. the detected PCI device
```

### Optional: PipeWire / PulseAudio device names

`PluseAudio/install.sh` adds a WirePlumber rule so the audio inputs show up
as "HAudio 1" … "HAudio 4" in PipeWire and OBS; `PluseAudio/uninstall.sh`
removes it. See the PDFs in that directory for background.

## Using the card

Each channel is a separate device. Example with ffmpeg (channel 1, 1080p):

```sh
ffmpeg -f v4l2 -video_size 1920x1080 -input_format yuyv422 -i /dev/video0 \
       -f alsa -i hw:CARD=HAudio1 out.mkv
```

With GStreamer:

```sh
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=1920,height=1080 ! videoconvert ! autovideosink
```

In OBS add a "Video Capture Device (V4L2)" source and pick `HVideo N`, and an
"Audio Input Capture" (PipeWire/PulseAudio) or ALSA source for `HAudio N`.
Use the input's own resolution for the lowest CPU use and latency.

Several applications can read the same channel at the same time; each one
gets its own copy of every frame for which it has a buffer queued.

The `play/` directory has quick test scripts (`xawtv`, `gst-launch-1.0`).

### Module parameters

Set them when loading (`sudo modprobe HwsCapture timing_debug=1`), permanently
with a file `/etc/modprobe.d/hwscapture.conf` containing e.g.
`options HwsCapture timing_debug=0`, or at runtime via
`/sys/module/HwsCapture/parameters/`.

| Parameter | Default | Meaning |
|---|---|---|
| `direct_io` | `1` | Copy frames straight from the card into the application's buffer when no conversion is needed. Set to `0` to always go through the driver's internal ring. |
| `timing_debug` | `0` | `1`: log per-second frame statistics for each channel to the kernel log; `2`: also warn about late frames. |

## Updating

```sh
cd avmatrix-pci-card-driver
git pull
sudo ./dkms-install.sh        # or sudo ./install.sh, whichever you used
```

The installer replaces the previous version. The new module is only used once
the old one is unloaded: close every application that uses the card (OBS,
browsers, PipeWire may keep the audio device open), then

```sh
sudo modprobe -r HwsCapture && sudo modprobe HwsCapture
```

or simply reboot.

## Uninstallation

```sh
cd avmatrix-pci-card-driver
sudo ./uninstall.sh           # Kylin OS: sudo ./Kylin-uninstall.sh
```

The uninstaller is also installed system-wide, so it works without the source
directory: `sudo hws-uninstall.sh`.

It removes the DKMS registration, the module file, `/usr/local/share/HWS` and
the helper scripts in `/usr/bin`. If the module is still loaded it offers to
reboot; alternatively close all applications using the card and run
`sudo modprobe -r HwsCapture`.

Remove it manually if the script is not available:

```sh
sudo modprobe -r HwsCapture
sudo dkms remove -m HwsCapture.ko -v 1.0.0.230324 --all
sudo rm -rf /usr/local/share/HWS /usr/src/HwsCapture.ko-1.0.0.230324 \
            /usr/bin/hws-uninstall.sh /usr/bin/hws-repair.sh
sudo find /lib/modules -name HwsCapture.ko -delete
sudo depmod -a
```

## Checking that it works

1. Load the module with statistics enabled:
   `sudo modprobe -r HwsCapture; sudo modprobe HwsCapture timing_debug=1`
2. Start capturing as usual, then watch `sudo dmesg -w | grep HwsCapture`.
   Each second and channel shows frames in/out and counters. In a healthy
   setup `in` and `out` match the input frame rate and `dma_overrun`,
   `overwrite` and `skip` stay at 0. `direct` counts frames that took the
   fast path.
3. For a deeper analysis run `sudo tools/stutter-monitor.py --device /dev/video0`
   while capturing; it reports dropped, duplicated and late frames together
   with likely system causes (interrupt rate, saturated or throttled CPUs,
   CPU/memory/IO pressure, busy application threads, kernel errors).

## Troubleshooting

| Symptom | What to do |
|---|---|
| Installer: "missing kernel development packages" | Install the headers for the running kernel (step 1); reboot if the kernel was just updated. |
| `modprobe: ERROR ... Key was rejected by service` | Secure Boot: enroll the DKMS key (see [Secure Boot](#secure-boot)). |
| No `/dev/video*` device | `lspci -nn` must list the card with an ID from the table above; check `sudo dmesg` for errors. |
| Picture shows "no signal" (blue) | No or unsupported input signal. Check the source and cable; inputs above 1920×1080 are not supported. |
| Stutter or dropped frames | Run the checks in [Checking that it works](#checking-that-it-works). Test with `direct_io=0` to rule out the fast path. |
| Module does not unload (`in use`) | Close all applications using the video or audio devices, including PipeWire clients, or reboot. |

## For developers

- `src/`: driver source (`hws_video.c`, `hws.h`, `hws_reg.h`) and the DKMS
  config.
- `tools/driver-tests/`: userspace tests that run the driver's own code (frame
  converters against guard pages, a simulation of the capture core with fault
  injection, ThreadSanitizer). Run `tools/driver-tests/verify.sh src`; see its
  README.
- `tools/stutter-monitor.py`: frame pacing and system monitor (above).
- `doc/`: the vendor's original user guide.

Licensed under the GPL (see `MODULE_LICENSE` in the source).
