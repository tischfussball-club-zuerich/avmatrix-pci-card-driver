# Driver tests (no hardware needed)

Userspace tests that run the driver's own code, extracted verbatim from
`src/hws_video.c`, so they always test the current source.

```
tools/driver-tests/verify.sh src     # from the repo root; takes ~25 min
```

Prints `RESULT: PASS` or `RESULT: FAIL`. Needs gcc with AddressSanitizer,
UndefinedBehaviorSanitizer and ThreadSanitizer, and python3.

## What it checks

- **conv/**: every frame converter and scaler (YUYV/YV12/NV12, scaling,
  deinterlacing, rotation) for all input sizes the driver accepts against all
  output formats. Each buffer sits next to an unmapped guard page, so any
  out-of-bounds read or write faults immediately; runs under ASan/UBSan.
  `harness hash` prints a hash per output to compare two versions
  byte-for-byte; `timek` times the converters with kernel compiler flags.
  `doubling.c` checks that the direct path's line doubling for interlaced
  input, with the frame split into halves at any byte, matches the worker's
  deinterlacer byte for byte.
- **sim/**: the capture core (hard IRQ handler, IRQ thread, half-frame
  copies, ring, worker, direct path, fan-out, stream start/stop, format
  changes) on a small kernel-API shim (`shim.h`), driven by a simulated card
  at real interrupt timing with 2 channels and 5 apps that keep stopping and
  restarting (one of them a display-style app with 2 buffers that keeps its
  frame until the next one arrives; at 60 fps it must get at least 75% of
  the frames), plus signal loss and resolution changes. Every delivered
  buffer is checked: no torn frame, no duplicate, frames in order,
  sequence/timestamp increasing, every buffer back after STREAMOFF and none
  completed after it.
  Modes (`verify.sh` runs all of them at 60/240/480 fps):
  - default: like real hardware, the card may overwrite a half the driver is
    still copying
  - `SIM_DMA_LOCK=1`: removes that race (driver logic only)
  - `SIM_NO_DIRECT=1`: ring + worker path only
  - `SIM_LOSE_IRQ=N` / `SIM_DUP_IRQ=N`: lose one in N second-half interrupts /
    report one in N first halves twice
  - `SIM_HOSTILE=1` (60/240 fps): alternates 1 s of broken hardware (status
    and format registers return garbage, 0 or all-ones; an interrupt storm
    with random status bits; garbage written into DMA memory; ALSA streams
    opened, closed and resized at random) with 1 s of sane hardware. Checks
    that nothing crashes or trips a sanitizer, that every buffer comes back on
    STREAMOFF, and that the fuzz really reaches the video, audio and ALSA copy
    paths. Frame content isn't checked in this mode.
- **ThreadSanitizer** run of the simulator. The remaining reports are the
  driver's intentional lock-free flags (pending bits, counters, busy flags)
  and the simulated card's own DMA writes; none involve frames, buffers or
  lists.

These tests cannot replace a run on the card. On the card, load the module
with `timing_debug=1` and run `tools/stutter-monitor.py --device /dev/videoN`.
