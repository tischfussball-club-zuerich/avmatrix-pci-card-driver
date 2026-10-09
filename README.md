# HwsCapture: Linux driver for AVMatrix VC41 / VC42 PCIe capture cards

Kernel driver (`HwsCapture`) for the AVMatrix VC42 (4× HDMI) and VC41
(4× 3G-SDI) capture cards and the other boards built on the same chip (VC22,
VC21, VC11 and single-channel variants). Every input appears as a V4L2 video
device (`/dev/videoN`, name `HVideo N`) and an ALSA capture card (`HAudio N`),
so it works with OBS, ffmpeg, GStreamer, VLC, browsers and PipeWire.

This is a reworked version of the AVMatrix vendor driver. It is based on
[GloriousEggroll/AVMATRIX-VC41-VC42-CAPTURE](https://github.com/GloriousEggroll/AVMATRIX-VC41-VC42-CAPTURE),
includes the additions from the AVMatrix 2026-05-19 release, and fixes a long
list of crashes, hangs and causes of stutter in the original code.

**What's better than the official driver, in short:**

- **No crashes or hangs** from the many bugs in the vendor code: buffer
  overflows, use-after-free on stop/close/unplug, divide-by-zero, an
  interrupt-handler lockup, deadlocks. Details in
  [Improvements](#improvements-over-the-official-driver).
- **Smooth capture:** no torn or duplicated frames, no growing latency, every
  frame delivered with an accurate timestamp. Frames go straight from the card
  into the application's buffer (one copy instead of two), and each channel
  has its own real-time copy thread, so four channels don't wait for each
  other.
- **Audio that keeps running** when the source is unplugged (silence instead
  of an I/O error), without xruns.
- **Several applications per channel** (e.g. OBS and a preview), each getting
  every frame.
- **Correct V4L2/ALSA behaviour** (`v4l2-compliance` passes except for the
  intentional multi-app design), working picture controls, suspend/resume,
  clean unplug and reboot.
- **Tested:** userspace test suite that runs the driver's own code, compile
  tests on kernels 5.15 to 7.0, and test scripts for the real card.

## Status

Tested on a **VC42** (`8888:8504`, hardware version 121) with kernel 6.17,
two 1080p60 HDMI sources, using [`tools/test-on-card.sh`](#testing-on-your-card):

- `v4l2-compliance`: 54 of 55 tests pass on every channel. The one failure is
  by design: every application gets its own buffer queue.
- 1080p60 capture, 60 s per channel and on both channels at once together
  with audio on all four cards: **no frame lost by the driver** (frames
  delivered = frames the card produced), in the direct path and when scaling.
- Audio on all four cards, with and without a signal.
- CPU: the driver's threads used about 6 % of one core for 2 × 1080p60.

Not yet tested on the card: all four inputs with a signal at once, long
sessions (hours), suspend/resume, hot-unplugging the source while streaming.
Reports are welcome; `tools/test-on-card.sh` writes a log that helps.

## Supported hardware

The driver binds to these PCI devices (subsystem `8888:0007`). Find yours with
`lspci -nn | grep -iE '1f33|8888'`.

| PCI ID (vendor:device) | Channels | Card models |
|---|---|---|
| `8888:9534`, `1f33:6524`, `8888:8524`, `1f33:8524` | 4 | VC42 (4× HDMI), VC41 (4× 3G-SDI) |
| `1f33:8534`, `1f33:8554`, `8888:8504`, `1f33:8504`, `8888:6504`, `8888:8521` | 4 | VC42 (4× HDMI), VC41 (4× 3G-SDI) |
| `8888:8532` | 2 | VC22 (2× HDMI), VC21 (2× SDI) |
| `8888:8512`, `1f33:6502` | 2 | VC22 (2× HDMI), VC21 (2× SDI) |
| `8888:8501` | 1 | single-channel series, e.g. VC11 (3G-SDI / HDMI) and VC12 variants |

Known exact mappings: `8888:8504` is the **VC42**; `8888:8501` is the
single-channel series. AVMatrix doesn't publish which product uses the other
IDs. Every channel gets one video device and one audio card.

**Not supported:** the **VC12-4K** (`8888:8581`, 1× 4K HDMI). Its ID isn't in
this driver's table and the driver handles inputs up to 1920×1080 only; use a
driver made for that card, e.g.
[GloriousEggroll/AVMATRIX-VC12-4K-CAPTURE](https://github.com/GloriousEggroll/AVMATRIX-VC12-4K-CAPTURE).

- **Input:** HDMI or SDI (depending on the card), up to 1920×1080 progressive
  or 1080i. The card reports the input frame rate; frames are passed through
  at that rate.
- **Output:** YUYV (4:2:2) at 1920×1080, 1680×1050, 1440×900, 1360×768,
  1280×1024, 1280×800, 1280×768, 1280×720, 1024×768, 800×600, 720×576,
  720×480, 640×480 and 1080×1920 (portrait, rotated). At the input size the
  frame is copied straight from the card into your buffer; other sizes are
  scaled, padded or cropped by the driver.
- **Audio:** 48 kHz, stereo, 16-bit, one ALSA card per channel.

## Supported kernels and distributions

Builds without warnings on Linux **5.15, 6.1, 6.8, 6.12, 6.16, 6.18, 6.19 and
7.0** (x86_64); versions in between should work too. Older kernels and other
architectures (e.g. arm64) are not tested.

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

To try it first without installing anything, see
[Testing on your card](#testing-on-your-card).

### 3. Install (choose one method)

**Option A: DKMS (recommended).** DKMS rebuilds the driver automatically
whenever you install a new kernel.

```sh
sudo ./dkms-install.sh
```

**Option B: plain install.** Builds and installs for the running kernel only;
run it again after every kernel update.

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
sudo dmesg | grep -i hwscapture    # e.g. "AVMatrix capture card 8888:8504"
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

Several applications can capture the same channel at the same time; each one
gets every frame for which it has a buffer queued. The `play/` directory has
quick test scripts (`xawtv`, `gst-launch-1.0`).

### Module parameters

Set them when loading (`sudo modprobe HwsCapture timing_debug=1`), permanently
with a file `/etc/modprobe.d/hwscapture.conf` containing e.g.
`options HwsCapture timing_debug=1`, or at runtime via
`/sys/module/HwsCapture/parameters/`.

| Parameter | Default | Meaning |
|---|---|---|
| `direct_io` | `1` | Copy frames straight from the card into the application's buffer when no conversion is needed. `0`: always go through the driver's internal ring (for comparison). |
| `nt_copy` | `0` | `1`: on the direct path, write frames with non-temporal stores that bypass the CPU cache. May help with several channels, may cost if the application reads each frame right away; measure on your system. |
| `timing_debug` | `0` | `1`: log per-second statistics for each streaming channel to the kernel log (see below); `2`: also warn about late frames. |

With `timing_debug=1`, `sudo dmesg -w | grep HwsCapture` shows a line per second
and channel, e.g.

```
HwsCapture ch0: in 60 fps (hw 60) out 60 fps | frames card 61 in 61 | ... dma_overrun 0 unpaired 0 direct 61 ...
```

`frames card` / `in` are the frames the card completed and the frames the
driver took in; they should be equal. `dma_overrun`, `unpaired`, `stale_half`,
`overwrite` and `skip` count frames the driver had to drop and should stay 0
in normal operation; `no_app_buf` counts frames no application had a free
buffer for; `direct` counts frames that took the fast path.

## Testing on your card

Two scripts check the driver on the real hardware. Both only read from the
devices; anything they change for a test (format, picture controls) is put
back afterwards.

**`tools/test-on-card.sh`: try this driver without installing it.** It builds
the driver from the checkout, loads it in place of the installed one (in
memory only), runs `card-check.sh`, and loads the original driver again, also
after an error or Ctrl-C. If only your own PipeWire/WirePlumber hold the card,
it stops them for the test and starts them again afterwards. A log file is
written to the current directory. `--keep` leaves the new driver loaded for
manual testing until `sudo rmmod HwsCapture` or a reboot; a reboot always
brings back the installed driver.

```sh
DUR=60 tools/test-on-card.sh
```

**`tools/card-check.sh`: check the loaded driver.** Connect a source to the
inputs you want checked. Per channel it runs `v4l2-compliance`, a capture at
the input size and one scaled to another size, counting lost and late frames.
Then it captures all channels with a signal plus audio on every card at the
same time, reports the CPU use, compares the frames the card produced with the
frames the driver took in, and lists the driver's drop counters and problem
messages from the kernel log. `DUR=60` makes the captures longer;
`SKIP_COMPLIANCE=1` leaves out `v4l2-compliance`, e.g. to compare with another
driver.

Expected results: `v4l2-compliance` fails only its multi-open test ("a
second open can't request buffers or stream", see above). Occasional "late"
frames (a 33 ms gap at 60 fps) with equal card and driver frame counts come
from the card or source, not the driver: the card skips a frame now and then,
most likely because the source's clock differs slightly from the card's.

**Memory leaks:** `tools/test-on-card.sh --leak` runs `tools/leak-check.sh`
instead of the card check. It repeats the allocation-heavy paths many times:
300 open/stream/close cycles per channel (with mmap and user-pointer buffers,
at the input size and scaled), 100 audio open/close cycles per card, and 10
unload/reload cycles of the driver. After a warm-up it checks that vmalloc
memory, the video buffers and the unreclaimable slab memory come back to
where they started, and if the kernel has `kmemleak`, that kmemleak finds
nothing from the driver. It takes about 10 minutes; `CYCLES`, `ACYCLES` and
`RELOADS` change the counts.

For frame pacing over a longer time, run
`sudo tools/stutter-monitor.py --device /dev/video0` while capturing; it
reports dropped, duplicated and late frames together with likely system causes
(interrupt rate, saturated or throttled CPUs, CPU/memory/IO pressure, busy
application threads, kernel errors).

## Updating

```sh
cd avmatrix-pci-card-driver
git pull
sudo ./dkms-install.sh        # or sudo ./install.sh, whichever you used
```

The installer replaces the previous version. The new module is used once the
old one is unloaded: close every application that uses the card (OBS,
browsers; PipeWire may keep the audio device open), then

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
directory: `sudo hws-uninstall.sh`. It removes the DKMS registration, the
module file, `/usr/local/share/HWS` and the helper scripts in `/usr/bin`. If
the module is still loaded it offers to reboot; alternatively close all
applications using the card and run `sudo modprobe -r HwsCapture`.

Manual removal, if the script is not available:

```sh
sudo modprobe -r HwsCapture
sudo dkms remove -m HwsCapture.ko -v 1.0.0.230324 --all
sudo rm -rf /usr/local/share/HWS /usr/src/HwsCapture.ko-1.0.0.230324 \
            /usr/bin/hws-uninstall.sh /usr/bin/hws-repair.sh
sudo find /lib/modules -name HwsCapture.ko -delete
sudo depmod -a
```

## Troubleshooting

| Symptom | What to do |
|---|---|
| Installer: "missing kernel development packages" | Install the headers for the running kernel (step 1); reboot if the kernel was just updated. |
| `Key was rejected by service` | Secure Boot: enroll the DKMS key (see [Secure Boot](#secure-boot)). |
| No `/dev/video*` device | `lspci -nn` must list the card with an ID from the table above (the VC12-4K needs a different driver); check `sudo dmesg \| grep -i hwscapture`. |
| Picture shows "no signal" | No or unsupported input signal. Check the source and cable; inputs above 1920×1080 are not supported. |
| Stutter or dropped frames | Load with `timing_debug=1` and run `tools/card-check.sh` (see [Testing on your card](#testing-on-your-card)); `direct_io=0` rules out the fast path. |
| Module does not unload (`in use`) | Close all applications using the video or audio devices, including PipeWire (`systemctl --user stop pipewire.socket pipewire-pulse.socket pipewire wireplumber`), or reboot. |

## Improvements over the official driver

Compared to the latest official AVMatrix driver (2026-05-19 release) and the
GloriousEggroll repository. Everything is covered by code review, the
userspace tests and compile tests; the capture path, audio and V4L2 behaviour
have also been verified on a VC42 (see [Status](#status)).

### Crashes and memory corruption

- **Interlaced input** (480i, 576i, 720i) made the deinterlacer write twice
  the frame size into the application's buffer.
- **YV12/NV12 boards** read before the output buffer for widths that are not a
  multiple of 16 (e.g. 1366): the original converters faulted in 2,210 of
  26,832 input/output combinations, now in none.
- **Changing the format while streaming** overflowed buffers (`S_FMT` now
  returns `EBUSY`), and raced buffer allocation on another file handle.
- **NULL pointer crash** when the signal came back.
- **Divide-by-zero** on a 0×0 resolution during mode switches, and in
  `S_PARM` with a zero frame rate from an application.
- **Use-after-free on stop/close:** the worker wasn't waited for, buffers of
  applications sharing a channel were handed back too early, and `poll()`
  could read freed memory.
- **Blocking `DQBUF` crashed** on kernels older than 7.0.
- **Audio:** buffer overflow with small ALSA buffers, and writes into freed
  memory after close.
- **Unplug/unbind/remove** freed memory still used by interrupts, workers and
  open files (the lifetime is now reference-counted); a wedged card could keep
  writing into freed buffers; registers were accessed after unmap; a card that
  is already gone is no longer touched.
- **Probe:** failures were reported as success (and crashed later), the
  interrupt could fire before setup, and partial failures leaked or
  unregistered the wrong devices.
- **Interrupt-handler lockup:** interrupt sources the driver didn't handle
  were never acknowledged, so the CPU stayed in the handler. All sources are
  acknowledged; a stuck status bit or an interrupt storm (1000 empty
  interrupts within 100 ms) pauses the card's interrupts for a second.
- **Reboot/kexec/hibernation:** there was no shutdown handler, so the card
  kept writing to memory and could hold a shared interrupt line asserted into
  the next kernel.

### Hangs and freezes

- **Blocking `DQBUF` held a per-device lock,** freezing other applications.
- **Stopping a hung card looped forever;** it now times out after 1 s.
- **Removing the device** waited for every audio application to close.
- **Interrupt acknowledgements were lost** (bits added with `+=`), stalling
  channels; channels could switch each other off through an unlocked shared
  register.
- **Shared interrupt lines were refused,** and a failed MSI setup aborted
  probe instead of falling back to legacy interrupts.
- **Signal loss mid-frame** left an application's buffer stuck.
- **A dead card** (all registers read as ones) was accepted at probe and never
  detected, while a single bad read marked it lost forever. It's now detected
  after repeated failures and recovers automatically, also when it was gone
  at resume and comes back later.

### Smooth capture and performance

- **Direct path:** at the input size (also interlaced, with lines doubled)
  each frame is copied once, straight from the card's DMA memory into the
  application's buffer, instead of twice plus a worker wakeup.
- **One real-time copy thread per channel** instead of one shared softirq
  that also delayed network, timer and disk work: four channels copy on four
  CPUs instead of queuing behind each other. Copies (which have a deadline:
  the card rewrites the buffer) run above conversion; conversion and the
  copies for additional applications run on a real-time thread per channel,
  so a busy encoder can't delay frames.
- **No torn frames:** ring slots were overwritten while being read, half-frame
  interrupts were dropped while a copy was busy, frames mixed two resolutions,
  and halves the card had already overwritten were shown. These are now
  detected and dropped (in a 480 fps stress simulation: 148 torn frames per
  minute before, 0 after); lost or duplicated interrupts are handled.
- **No duplicate bursts, no growing latency:** one frame used to fill all of
  an application's buffers, and the backlog grew to ~50 ms. The driver now
  delivers every frame once, oldest first, and skips to the newest when an
  application is short of buffers.
- **No conversion with interrupts disabled** (it used to block them for
  0.2–0.9 ms per buffer).
- **Accurate timing:** timestamps come from the frame's interrupt; sequence
  numbers are per application and always increase, so a gap is a real drop;
  the reported frame rate is the real input rate instead of a fixed 60.
- **Applications with few buffers** get every frame they have room for, and
  an application that queues a buffer while a frame is arriving still gets
  it; up to three applications per channel are served directly.
- **Faster converters** with byte-identical output: 1080p→720p 0.43→0.19 ms,
  720p→1080p 0.33→0.24 ms, YV12 1080p 0.77→0.53 ms, NV12 1080p 0.65→0.37 ms
  per frame (kernel compiler flags).
- **Mode changes** are noticed within 250 ms instead of up to 1 s; while there
  is no signal nothing is copied and the "no signal" picture comes at ~30 fps
  (it used to come once per second).

### Audio

- **No I/O error without a signal:** the card sends no audio then, so the
  ALSA stream stalled. The driver now delivers silence at the exact 48 kHz
  pace until audio returns.
- **No xruns:** periods match the card's 1024-frame packets, the BATCH flag is
  set, and no stale packet plays at start.
- Audio is delivered directly from the interrupt thread, without an extra
  wakeup per packet, and never waits behind video copies; buffers up to
  ~680 ms; the card's long name includes its PCI address.

### V4L2, controls and system integration

- **Picture controls** (brightness, contrast, saturation, hue) never reached
  the hardware; they now use the kernel's control framework (`v4l2-ctl`
  lists them, events and `LOG_STATUS` work, the hue default is right).
- **Format replies** are complete (colorimetry fields), the input reports
  "no signal", there is no fake analog TV standard, and `bus_info` is unique
  (`PCI:<address>`) so tools can tell two cards apart.
- **Suspend/resume** was missing (the card came back unconfigured).
- **PCI setup:** explicit 32-bit DMA mask, PCI regions requested, bus
  mastering only after the card reset, Relaxed Ordering no longer forced on
  root ports where the kernel disables it as broken.
- **Memory:** the 16 × 4 MB ring buffers no longer need physically contiguous
  memory (which failed after long uptime); applications can't allocate
  oversized buffers.
- **Scaler fixes:** U/V swapped and junk pixels when centring small inputs,
  stale bytes read past the NV12 frame, an unaligned read, width and height
  swapped for portrait output on 4:2:0 boards.
- **32-bit builds** work (link errors, torn 64-bit timestamps); deprecated
  tasklets are replaced by threaded interrupts.

### Code quality and testing

- No compiler warnings with `W=1` on all supported kernels, `sparse`-clean
  (correct `__iomem`/`__user` annotations), no dead vendor code, messages
  with the card's PCI address, log flooding rate-limited.
- **`tools/driver-tests`:** runs the driver's own code in userspace: every
  converter against guard pages with AddressSanitizer/UBSan, and a simulation
  of the capture core (simulated card at real interrupt timing, several
  applications that keep restarting, signal loss, mode changes, lost and
  duplicate interrupts, hostile hardware returning garbage) that checks every
  delivered frame, plus ThreadSanitizer.
- **`tools/card-check.sh`, `tools/leak-check.sh`, `tools/test-on-card.sh`** for the real card (see
  [Testing on your card](#testing-on-your-card)).

## For developers

- `src/`: driver source (`hws_video.c`, `hws.h`, `hws_reg.h`) and the DKMS
  config.
- `tools/driver-tests/`: the userspace test suite; run
  `tools/driver-tests/verify.sh src` (about 30 minutes, prints `RESULT: PASS`
  or `FAIL`); see its README.
- `tools/card-check.sh`, `tools/leak-check.sh`, `tools/test-on-card.sh`:
  checks on the real card.
- `tools/stutter-monitor.py`: frame pacing and system monitor.
- `doc/`: the vendor's original user guide.

Licensed under the GPL (GPL-2.0-or-later, see the SPDX tags in the source).
