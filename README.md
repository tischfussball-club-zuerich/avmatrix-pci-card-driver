# HwsCapture: Linux driver for AVMatrix VC41 / VC42 PCIe capture cards

Kernel driver (`HwsCapture`) for the AVMatrix VC41 (4× 3G-SDI) and VC42
(4× HDMI) capture cards and the other boards built on the same HWS chip
(VC22, VC21, VC11 and single-channel variants). Each input appears as a V4L2 video
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

| PCI ID (vendor:device) | Video channels created | Card models |
|---|---|---|
| `8888:9534`, `1f33:6524`, `8888:8524`, `1f33:8524` | 4 | VC42 (4× HDMI), VC41 (4× 3G-SDI) |
| `1f33:8534`, `1f33:8554`, `8888:8504`, `1f33:8504`, `8888:6504`, `8888:8521` | 4 | VC42 (4× HDMI), VC41 (4× 3G-SDI) |
| `8888:8532` | 2 | VC22 (2× HDMI), VC21 (2× SDI) |
| `8888:8512`, `1f33:6502` | 2 | VC22 (2× HDMI), VC21 (2× SDI) |
| `8888:8501` | 1 | single-channel series, e.g. VC11 (3G-SDI / HDMI) and VC12 variants |

Known exact mappings: `8888:8504` is the **VC42** (4× HDMI); `8888:8501` is the
single-channel series. For the other IDs AVMatrix doesn't publish which
product uses which; check `lspci -nn` on your machine. Every channel gets one
video device and one audio card.

**Not supported by this driver:** the **VC12-4K** (`8888:8581`, 1× 4K HDMI).
Its ID isn't in this driver's table, and this driver only handles inputs up to
1920×1080; use a driver made for that card, such as
[GloriousEggroll/AVMATRIX-VC12-4K-CAPTURE](https://github.com/GloriousEggroll/AVMATRIX-VC12-4K-CAPTURE).

**Input:** HDMI or SDI (depending on the card), up to 1920×1080 progressive,
or interlaced up to 1080i.
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
sudo dmesg | grep -i hwscapture     # driver messages, e.g. the detected card
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
| `nt_copy` | `0` | `1`: on the direct path, write frames into the application's buffer with non-temporal stores that bypass the CPU cache. May help with several channels at once, may cost if the app reads each frame right away; measure on your system. |
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
   **Another HwsCapture driver installed?** `tools/test-on-card.sh` builds
   this driver from the checkout, loads it in place of the installed one (in
   memory only), runs the check below, and loads the original driver again,
   also after an error or Ctrl-C. If only your PipeWire/WirePlumber hold the
   card, it stops them for the test and starts them again afterwards.
   Nothing is installed; a log file is written
   to the current directory. `--keep` leaves the new driver loaded for manual
   testing until `sudo rmmod HwsCapture` or a reboot.
3. Run `tools/card-check.sh` with a source connected. Per channel it runs
   `v4l2-compliance`, a 10 s capture that counts lost and late frames, and a
   short audio capture, and it lists problem messages from the kernel log. It
   only reads from the devices. Expected: `v4l2-compliance` reports failures
   only in its multi-open tests ("a second open can't request buffers or
   stream"); this driver deliberately gives every application its own queue,
   so several apps can capture the same channel.
4. For a deeper analysis run `sudo tools/stutter-monitor.py --device /dev/video0`
   while capturing; it reports dropped, duplicated and late frames together
   with likely system causes (interrupt rate, saturated or throttled CPUs,
   CPU/memory/IO pressure, busy application threads, kernel errors).

## Troubleshooting

| Symptom | What to do |
|---|---|
| Installer: "missing kernel development packages" | Install the headers for the running kernel (step 1); reboot if the kernel was just updated. |
| `modprobe: ERROR ... Key was rejected by service` | Secure Boot: enroll the DKMS key (see [Secure Boot](#secure-boot)). |
| No `/dev/video*` device | `lspci -nn` must list the card with an ID from the table above (the VC12-4K, `8888:8581`, needs a different driver); check `sudo dmesg` for errors. |
| Picture shows "no signal" (blue) | No or unsupported input signal. Check the source and cable; inputs above 1920×1080 are not supported. |
| Stutter or dropped frames | Run the checks in [Checking that it works](#checking-that-it-works). Test with `direct_io=0` to rule out the fast path. |
| Module does not unload (`in use`) | Close all applications using the video or audio devices, including PipeWire clients, or reboot. |

## Changes compared to the official driver

Compared to the latest official AVMatrix driver (2026-05-19 release) and the
GloriousEggroll repository, this version fixes the following. All of it is
verified by code review, the userspace tests in `tools/driver-tests` and
compile tests; none of it has been confirmed on a card yet.

### Crashes and memory corruption

- **Interlaced inputs** (480i, 576i, 720i) made the deinterlacer write twice
  the frame size into the application's buffer.
- **YV12/NV12 boards** read before the output buffer for widths that are not a
  multiple of 16 (e.g. 1366). Over all input/output size combinations the
  original converters faulted in 2,210 of 26,832; now in none.
- **Changing the format while streaming** overflowed buffers (`S_FMT` now
  returns `-EBUSY`), and `S_FMT` raced buffer allocation on another file handle.
- **NULL pointer crash** when the HDMI signal came back.
- **Divide-by-zero** on a 0×0 resolution during mode switches, and in
  `S_PARM` when an application passed a zero numerator.
- **Use-after-free on STREAMOFF/close:** the worker wasn't waited for, USERPTR
  buffers were handed back before all applications had their copy, and
  `poll()` could read freed memory.
- **Blocking `DQBUF` crashed** on kernels older than 7.0 (`mutex_unlock(NULL)`).
- **Audio:** buffer overflow with small ALSA buffers, and writes into freed
  memory after close.
- **Unbind/remove** freed memory still used by interrupts, workers and open
  files (lifetime is now reference-counted); a wedged card could keep DMAing
  into freed buffers; registers were accessed after unmap (probe error path,
  control writes racing remove); on surprise removal the driver now stops
  touching a card that is gone.
- **Probe:** failures were reported as success (and crashed on remove), the
  interrupt could fire before setup, and partial failures unregistered devices
  that were never registered and leaked sound cards.
- **Hard lockup in the interrupt handler:** interrupt sources the driver
  didn't handle were never acknowledged, so a level-triggered line kept the
  CPU in the handler. All sources are acknowledged now; a stuck status bit is
  detected, and after a storm (1000 empty interrupts within 100 ms) the
  card's interrupts are paused for a second.
- **Reboot/kexec:** there was no shutdown handler, so the card kept DMAing
  and could hold a shared interrupt line asserted into the next kernel. The
  handler also works when hibernation powers off (frozen status thread).

### Hangs and freezes

- **Blocking `DQBUF` held a per-device lock,** so other applications and
  threads froze in uninterruptible sleep.
- **Stopping a hung card looped forever;** it now times out.
- **Removing the device** waited for every audio application to close.
- **Interrupt acknowledgements were lost** (bits added with `+=`), stalling
  channels; channels could switch each other off through an unlocked shared
  register.
- **Shared interrupt lines were refused,** and a failed MSI setup aborted
  probe instead of falling back to legacy interrupts.
- **Signal loss mid-frame** left an application's buffer stuck.
- **A card that was gone at resume** and came back later was never
  re-initialized (and never noticed); it now gets the full bring-up.
- **A dead card** (all registers read as ones) was accepted at probe and never
  detected at runtime, while a single bad read marked it lost forever. It is
  now detected reliably and recovers automatically.

### Stutter, dropped frames and latency

- **Same size in and out (the common case):** frames are copied straight from
  the card's DMA memory into the application's buffer: 1 copy instead of 2,
  and no worker wakeup. Several applications on one channel are served by a
  separate worker, so they don't hold up the capture.
- **One frame filled all of an application's buffers,** causing bursts of
  duplicates followed by gaps.
- **Frame conversion ran with interrupts disabled** (0.2–0.9 ms per buffer).
- **The backlog grew without limit** (up to ~50 ms). The worker now drains the
  ring and skips to the newest frame when an application is short of buffers.
- **Torn frames:** ring slots were overwritten while being read, half-frame
  interrupts were dropped while a copy was busy, frames mixed two resolutions
  during a mode switch, and halves the card overwrote mid-copy were shown.
  These are now detected and dropped (in a 480 fps stress simulation: 148 torn
  frames per minute before, 0 after). Lost or duplicated half-frame interrupts
  are handled, halves far apart in time (interrupts paused, card recovery)
  are never paired, and the first frame after a mode change that the card
  started in the old layout is skipped.
- **Big copies ran in softirq,** delaying network, timer and disk work. Each
  channel now has its own copy thread (lowest real-time priority), so with
  4 channels the copies (up to ~1 GB/s at 4×1080p60) run on 4 CPUs instead
  of queuing behind each other on one. Conversion and delivery, including
  the copies for additional applications on the same channel, also run on
  one real-time thread per channel, so a busy encoder can't delay frames; the
  copy threads have priority over conversion, since a half-frame must be
  copied before the card rewrites it.
- **Audio** is delivered straight from the interrupt thread, without an
  extra wakeup per packet, and no longer waits behind video copies.
- **Interlaced input** (1080i, 576i, 480i) shown at the input size is now
  copied straight from DMA into the application's buffer with the lines
  doubled, instead of going through the ring and the worker.
- **Workers:** one wakeup per frame instead of two, on high-priority workers
  that run the channels in parallel instead of one shared thread.
- **Timestamps** now come from the frame's interrupt instead of worker
  scheduling; sequence numbers are per application and always increase, so a
  gap means a real drop.
- **The no-signal picture** came once per second; now at about 30 fps.
- **Mode changes** (resolution, frame rate, signal) are noticed within
  250 ms instead of up to 1 s.
- **ALSA xruns:** the minimum period was below the card's 4096-byte packet,
  the BATCH flag was missing and a stale packet played at start.
- **Faster converters** (byte-identical output; userspace, kernel compiler
  flags): 1080p→720p 0.43→0.19 ms, 720p→1080p 0.33→0.24 ms, YV12 1080p
  0.77→0.53 ms, NV12 1080p 0.65→0.37 ms per frame.
- **No signal:** nothing is copied from the card while there's no signal,
  and the "no signal" picture is limited to ~30 fps.
- **Several applications on one channel:** up to three are served directly
  by the channel's copy thread; an application that queues a buffer while a
  frame is already arriving still gets that frame.
- **Log flooding** from paths applications can trigger is rate-limited.

### Correctness and compatibility

- **Suspend/resume** was missing, so the card came back unconfigured.
- **Picture controls** (brightness, contrast, saturation, hue) never reached
  the hardware; they now use the kernel's control framework, so `v4l2-ctl`
  lists them, and the hue default is right.
- **V4L2 reporting:** the real input frame rate instead of a fixed 60, no fake
  analog TV standard, "no signal" status in `ENUM_INPUT`, and a unique
  `bus_info` (`PCI:<address>`) so tools can tell cards apart.
- **Portrait output (1080×1920)** from 4:2:0 boards was scaled with width and
  height swapped.
- **PCI setup:** explicit 32-bit DMA mask, PCI regions requested, bus
  mastering enabled only after the card reset, and Relaxed Ordering no longer
  forced on root ports where the kernel disables it as broken.
- **Memory:** the 16 × 4 MB ring buffers no longer need physically contiguous
  memory (which failed after long uptime), and applications can no longer
  allocate oversized buffers through `CREATE_BUFS`.
- **Scalers:** small inputs were centred with U/V swapped and junk pixels at
  the edge, NV12 read stale bytes past the frame, an unaligned read in the
  1080p→720p scaler, and unknown pixel layouts were sized inconsistently.
- **32-bit builds:** link errors and torn 64-bit timestamps; deprecated
  tasklets replaced by a threaded interrupt.

## For developers

- `src/`: driver source (`hws_video.c`, `hws.h`, `hws_reg.h`) and the DKMS
  config.
- `tools/driver-tests/`: userspace tests that run the driver's own code (frame
  converters against guard pages, a simulation of the capture core with fault
  injection, ThreadSanitizer). Run `tools/driver-tests/verify.sh src`; see its
  README.
- `tools/stutter-monitor.py`: frame pacing and system monitor (above).
- `tools/card-check.sh`: checks on the real card (above).
- `tools/test-on-card.sh`: tries this driver next to an installed one (above).
- `doc/`: the vendor's original user guide.

Licensed under the GPL (see `MODULE_LICENSE` in the source).
