#!/usr/bin/env python3
"""
stutter-monitor: watch the whole capture pipeline for causes of stutter.

Samples once per interval and flags anything that can make video stutter:
  - driver:  what the driver itself sees on every channel, from
             /sys/class/video4linux/videoN/stats (this driver; the stock one
             has no such file): frames the card produced vs. frames the
             driver took in and delivered, its drop counters, applications
             without a free buffer, signal and format changes, audio from the
             card vs. silence filled in. Costs nothing on the frame path: a
             few counters read once per interval. This is the part to leave
             running while streaming.
  - frames:  with --device, reads frames from the V4L2 device as an extra
             consumer and checks frame intervals, gaps, duplicate bursts and
             delivery latency (the driver supports several readers per device)
  - IRQ:     the card's interrupt rate and which CPU handles it
  - CPU:     saturated cores, frequency drops, thermal throttling
  - PSI:     CPU / memory / IO pressure stalls (kernel's own "something waited")
  - app:     threads of OBS/ffmpeg/GStreamer pinned at 100% of a core
  - kernel:  PCIe AER errors and other relevant kernel log messages

The frame probe never changes the capture format. It does add one extra frame
copy per buffer inside the driver, so watch one channel at a time if the
system is already at its limit.

Ctrl+C (or SIGTERM, or the terminal closing) prints a summary that correlates
frame problems with system events; a second Ctrl+C quits at once.

Examples:
  sudo ./stutter-monitor.py                  # driver + system, no extra load
  sudo ./stutter-monitor.py --quiet --log run.jsonl   # only seconds with problems
  sudo ./stutter-monitor.py --device /dev/video0
  sudo ./stutter-monitor.py --device /dev/video0 --fps 50 --quiet --log run.jsonl
"""

import argparse
import collections
import ctypes
import errno
import fcntl
import glob
import json
import mmap
import os
import re
import select
import signal
import statistics
import struct
import sys
import threading
import time
import zlib

DRIVER = "HwsCapture"
CLK_TCK = os.sysconf("SC_CLK_TCK")
KMSG_INTEREST = re.compile(r"HwsCapture|AER|pcieport|interrupt took|hung_task|soft lockup|rcu.*stall", re.I)
# the driver's own timing lines (timing_debug=1/2): already covered by its stats file
KMSG_TIMING = re.compile(r"^HwsCapture ch\d+: ")

# Flags that mean "the frames themselves were bad" - correlated against the rest
FRAME_FLAGS = frozenset(("FRAME-GAP", "DUP-FRAMES", "LOW-FPS", "JITTER", "DELIVERY-LATE",
                         "DRV-LOST", "DRV-DROP", "CARD-SLOW", "NO-APP-BUF"))

# counters in the driver's stats file (totals since it loaded; the monitor shows differences)
DRV_COUNTERS = ("card_frames", "frames_in", "frames_out", "direct", "dma_overrun", "stale_half", "unpaired",
                "aborted", "copy_missed", "ring_overwrite", "ring_skip", "no_app_buf",
                "audio_card_packets", "audio_silence_packets")
# every frame the card completes ends in frames_in or exactly one of these (newer drivers;
# a counter the driver doesn't have reads as 0)
DRV_INTAKE_DROPS = ("dma_overrun", "stale_half", "unpaired", "aborted", "copy_missed")
# dropped later, after the driver took the frame in
DRV_DROPS = DRV_INTAKE_DROPS + ("ring_overwrite", "ring_skip")
# 32-bit in the driver: these wrap around, the others are 64-bit and only go down on a reload
DRV_U32 = ("card_frames", "audio_card_packets", "audio_silence_packets")


def num(v, default=0):
    try:
        return int(str(v).strip())
    except (TypeError, ValueError):
        return default


def read_driver_stats():
    """{"video0": {key: value}} for every channel of this driver that has a stats file."""
    out = {}
    for path in sorted(glob.glob("/sys/class/video4linux/video*/stats")):
        try:
            ino = os.stat(path).st_ino  # sysfs makes a new file on every driver load
        except OSError:
            continue
        txt = read_file(path)
        if not txt:
            continue
        d = {"_id": ino, "_t": time.monotonic()}
        for line in txt.splitlines():
            k, _, v = line.partition(" ")
            d[k] = v.strip()
        if "card_frames" in d:
            out[os.path.basename(os.path.dirname(path))] = d
    return out


def frame_rate(d):
    """Measured input rate (frame_interval "1001/60000"), else what the card reports."""
    num_, _, den = d.get("frame_interval", "").partition("/")
    n, m = num(num_), num(den)
    return m / n if n > 0 and m > 0 else float(num(d.get("input_fps")))


def frame_rate_measured(d):
    n, _, m = d.get("frame_interval", "").partition("/")
    return num(n) > 0 and num(m) > 0


class DriverTracker:
    """Turns the driver's running totals into per-sample differences, per channel.

    A sample right after a stream start, a signal or format change is only partly
    representative (the card resyncs, the driver restarts its measurements), so
    such samples are marked "settling" and rate/loss checks skip them.
    Frames the card completed but the driver hasn't counted yet are carried as a
    balance: the driver counts them a moment later, in its interrupt thread, so a
    sample can fall in between. Only a balance that stays is reported as lost.
    """

    def __init__(self):
        self.prev = {}
        self.settle = {}
        self.bal = {}
        self.over = {}
        self.no_audio = {}

    def update(self, cur):
        res = {}
        for name, d in cur.items():
            p = self.prev.get(name)
            e = {"streaming": d.get("streaming") == "1", "signal": d.get("signal") == "1",
                 "input": d.get("input", "?"), "output": d.get("output", "?"),
                 "apps": num(d.get("apps_streaming")), "card_lost": d.get("card_lost") == "1",
                 "irq_paused": d.get("irq_paused") == "1", "audio_running": d.get("audio_running") == "1",
                 "fps": frame_rate(d), "measured": frame_rate_measured(d)}
            # an application uses the channel (an unused one's source may sleep and wake)
            e["in_use"] = e["streaming"] or e["audio_running"] or num(d.get("apps_open")) > 0
            res[name] = e
            dt = d["_t"] - p["_t"] if p else 0
            if (p is None or p["_id"] != d["_id"] or p.get("channel") != d.get("channel")
                    or any(num(d.get(k)) < num(p.get(k)) for k in DRV_COUNTERS if k not in DRV_U32)
                    # more frames than any input could deliver: not the same counter any more
                    or (num(d.get("card_frames")) - num(p.get("card_frames"))) & 0xFFFFFFFF > 500 * dt + 100):
                # new channel, or the driver was reloaded: this sample is the new baseline
                e["baseline"] = True
                self.settle[name], self.bal[name], self.over[name] = 1, 0, False
                continue
            e["dt"] = dt
            for k in DRV_COUNTERS:
                diff = num(d.get(k)) - num(p.get(k))
                e[k] = diff & 0xFFFFFFFF if k in DRV_U32 else diff
            e["changed"] = [k for k in ("signal", "input") if p.get(k) != d.get(k)]
            e["was_streaming"] = p.get("streaming") == "1"
            e["audio_was_running"] = p.get("audio_running") == "1"
            # capture (re)started in between: also a stop and start within one sample
            restarted = d.get("capture_starts") != p.get("capture_starts")

            if e["changed"]:
                self.settle[name] = 2   # this sample and the next
            elif restarted or (e["streaming"] and not e["was_streaming"]):
                self.settle[name] = 1
            e["settling"] = self.settle.get(name, 0) > 0 or e["dt"] <= 0
            self.settle[name] = max(0, self.settle.get(name, 0) - 1)

            e["lost"] = 0
            if e["settling"] or not (e["streaming"] and e["was_streaming"] and e["signal"]):
                self.bal[name], self.over[name] = 0, False
            else:
                lost = e["card_frames"] - e["frames_in"] - sum(e[k] for k in DRV_INTAKE_DROPS)
                bal = max(0, self.bal.get(name, 0) + lost)
                if bal > 1 and self.over.get(name):
                    e["lost"], bal = bal, 0
                self.over[name] = bal > 1
                self.bal[name] = bal

            # audio: silence filled in while the card still sends some = a stall; a source
            # without audio (nothing from the card at all) is reported once, not every second
            e["audio_stall"] = e["audio_none"] = False
            if e["audio_running"] and e["audio_was_running"] and e["signal"] and not e["settling"]:
                if e["audio_card_packets"] == 0 and e["audio_silence_packets"]:
                    e["audio_none"] = not self.no_audio.get(name)
                    self.no_audio[name] = True
                elif e["audio_card_packets"]:
                    e["audio_stall"] = e["audio_silence_packets"] > 0
                    self.no_audio[name] = False
            elif not e["audio_running"] or e["changed"]:
                self.no_audio[name] = False
        self.prev = cur
        return res


def read_file(path, default=None):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


# ---------------------------------------------------------------- V4L2 frame probe

def _ioc(direction, nr, size):
    return (direction << 30) | (size << 16) | (ord("V") << 8) | nr


V4L2_BUF_SIZE = 88 if ctypes.sizeof(ctypes.c_long) == 8 else 68
VIDIOC_REQBUFS = _ioc(3, 8, 20)
VIDIOC_QUERYBUF = _ioc(3, 9, V4L2_BUF_SIZE)
VIDIOC_QBUF = _ioc(3, 15, V4L2_BUF_SIZE)
VIDIOC_DQBUF = _ioc(3, 17, V4L2_BUF_SIZE)
VIDIOC_STREAMON = _ioc(1, 18, 4)
VIDIOC_STREAMOFF = _ioc(1, 19, 4)
BUF_TYPE_VIDEO_CAPTURE = 1
MEMORY_MMAP = 1

if V4L2_BUF_SIZE == 88:   # 64-bit struct v4l2_buffer layout
    OFF_TS, TS_FMT, OFF_SEQ, OFF_MOFFSET, OFF_LENGTH = 24, "qq", 56, 64, 72
else:                     # 32-bit
    OFF_TS, TS_FMT, OFF_SEQ, OFF_MOFFSET, OFF_LENGTH = 20, "ll", 44, 52, 56


class FrameProbe(threading.Thread):
    """Extra V4L2 consumer: dequeues frames and records timestamp, arrival and content checksum."""

    def __init__(self, device, queued=2):
        super().__init__(daemon=True)
        self.device = device
        self.queued = queued
        self.lock = threading.Lock()
        self.events = []          # (buffer_ts, arrival, sequence, crc)
        self.error = None
        self.running = True
        self.fd = None
        self.maps = []

    def _buf(self, index=0):
        b = bytearray(V4L2_BUF_SIZE)
        struct.pack_into("II", b, 0, index, BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into("I", b, OFF_SEQ + 4, MEMORY_MMAP)
        return b

    def _setup(self):
        self.fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
        req = bytearray(20)
        struct.pack_into("III", req, 0, self.queued, BUF_TYPE_VIDEO_CAPTURE, MEMORY_MMAP)
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req, True)
        # vb2 may allocate more than requested; only keep `queued` in flight
        count = min(struct.unpack_from("I", req, 0)[0], self.queued)
        for i in range(count):
            b = self._buf(i)
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, b, True)
            offset = struct.unpack_from("I", b, OFF_MOFFSET)[0]
            length = struct.unpack_from("I", b, OFF_LENGTH)[0]
            self.maps.append(mmap.mmap(self.fd, length, mmap.MAP_SHARED, mmap.PROT_READ, offset=offset))
            fcntl.ioctl(self.fd, VIDIOC_QBUF, b, True)
        fcntl.ioctl(self.fd, VIDIOC_STREAMON, struct.pack("I", BUF_TYPE_VIDEO_CAPTURE))

    def _crc(self, m):
        # checksum a few slices - enough to tell identical frames apart cheaply
        n = len(m)
        crc = 0
        for pos in (n // 8, n // 3, n // 2, 2 * n // 3, 7 * n // 8):
            crc = zlib.crc32(m[pos:pos + 4096], crc)
        return crc

    def run(self):
        try:
            self._setup()
            while self.running:
                r, _, _ = select.select([self.fd], [], [], 0.5)
                if not r:
                    continue
                b = self._buf()
                try:
                    fcntl.ioctl(self.fd, VIDIOC_DQBUF, b, True)
                except OSError as e:
                    if e.errno == errno.EAGAIN:
                        continue
                    raise
                arrival = time.clock_gettime(time.CLOCK_MONOTONIC)
                sec, usec = struct.unpack_from(TS_FMT, b, OFF_TS)
                seq = struct.unpack_from("I", b, OFF_SEQ)[0]
                index = struct.unpack_from("I", b, 0)[0]
                crc = self._crc(self.maps[index])
                with self.lock:
                    self.events.append((sec + usec / 1e6, arrival, seq, crc))
                fcntl.ioctl(self.fd, VIDIOC_QBUF, b, True)
        except Exception as e:  # noqa: BLE001 - any failure ends this probe; the main loop restarts it
            self.error = f"{self.device}: {e}"
        finally:
            self._teardown()

    def _teardown(self):
        if self.fd is None:
            return
        try:
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, struct.pack("I", BUF_TYPE_VIDEO_CAPTURE))
        except OSError:
            pass
        for m in self.maps:
            m.close()
        self.maps = []
        try:
            os.close(self.fd)
        except OSError:
            pass
        self.fd = None

    def take(self):
        with self.lock:
            ev, self.events = self.events, []
        return ev


class FrameStats:
    """Turns probe events into per-interval frame metrics."""

    def __init__(self, fps):
        self.fixed_period = 1.0 / fps if fps else None
        self.intervals = collections.deque(maxlen=300)
        self.last_ts = None
        self.last_crc = None

    def reset(self):
        self.intervals.clear()
        self.last_ts = None
        self.last_crc = None

    def period(self):
        if self.fixed_period:
            return self.fixed_period
        if len(self.intervals) >= 20:
            return statistics.median(self.intervals)
        return None

    def update(self, events, dt):
        # Buffers whose timestamps are much closer than a frame period belong to
        # the same hardware frame: the driver handed one frame to several buffers.
        burst_gap = (self.period() or 1 / 60) / 4
        frames = 0
        dups = 0
        same_content = 0
        ivs = []
        deliv = []
        for ts, arrival, _seq, crc in events:
            deliv.append(arrival - ts)
            if self.last_ts is not None and ts - self.last_ts < burst_gap:
                dups += 1
                if crc == self.last_crc:
                    same_content += 1
                continue
            if self.last_ts is not None:
                iv = ts - self.last_ts
                ivs.append(iv)
                self.intervals.append(iv)
            self.last_ts = ts
            self.last_crc = crc
            frames += 1

        period = self.period()
        missed = 0
        if period:
            missed = sum(max(0, round(iv / period) - 1) for iv in ivs)
        return {
            "buffers": len(events), "frames": frames, "fps": frames / dt if dt else 0,
            "dups": dups, "dups_same_content": same_content, "missed": missed,
            "iv_min_ms": min(ivs) * 1000 if ivs else 0, "iv_max_ms": max(ivs) * 1000 if ivs else 0,
            "deliv_max_ms": max(deliv) * 1000 if deliv else 0,
            "period_ms": period * 1000 if period else 0,
        }


# ---------------------------------------------------------------- system samplers

def card_pci_addrs():
    return [os.path.basename(p) for p in glob.glob(f"/sys/bus/pci/drivers/{DRIVER}/0000:*")]


def read_cpu_times():
    """{cpu_index: (busy_ticks, total_ticks)}"""
    out = {}
    for line in (read_file("/proc/stat") or "").splitlines():
        m = re.match(r"cpu(\d+) (.*)", line)
        if not m:
            continue
        v = [int(x) for x in m.group(2).split()]
        idle = v[3] + (v[4] if len(v) > 4 else 0)  # idle + iowait
        total = sum(v[:8])
        out[int(m.group(1))] = (total - idle, total)
    return out


def read_irq_counts(names):
    """Per-CPU interrupt counts for /proc/interrupts lines matching any of names."""
    text = read_file("/proc/interrupts") or ""
    lines = text.splitlines()
    if not lines or not names:
        return {}
    cpus = [int(c[3:]) for c in lines[0].split()]
    counts = collections.Counter()
    for line in lines[1:]:
        if not any(n in line for n in names):
            continue
        fields = line.split()[1:1 + len(cpus)]
        for cpu, val in zip(cpus, fields):
            if val.isdigit():
                counts[cpu] += int(val)
    return counts


def read_psi():
    """{resource: some_total_us}"""
    out = {}
    for res in ("cpu", "memory", "io"):
        text = read_file(f"/proc/pressure/{res}")
        if text:
            m = re.search(r"^some .*total=(\d+)", text, re.M)
            if m:
                out[res] = int(m.group(1))
    return out


def read_freqs():
    """{cpu: (cur_khz, max_khz)}"""
    out = {}
    for d in glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq"):
        cpu = int(re.search(r"cpu(\d+)/cpufreq", d).group(1))
        cur = read_file(f"{d}/scaling_cur_freq")
        mx = read_file(f"{d}/cpuinfo_max_freq")
        if cur and mx:
            out[cpu] = (int(cur), int(mx))
    return out


def read_throttle_total():
    total = 0
    for f in glob.glob("/sys/devices/system/cpu/cpu[0-9]*/thermal_throttle/*_throttle_count"):
        v = read_file(f)
        if v and v.strip().isdigit():
            total += int(v)
    return total


def find_app_pids(names):
    me = os.getpid()
    pids = []
    for d in glob.glob("/proc/[0-9]*"):
        pid = int(os.path.basename(d))
        comm = (read_file(f"{d}/comm") or "").strip()
        if pid != me and any(comm.startswith(n) for n in names):
            pids.append((pid, comm))
    return pids


def read_thread_ticks(pid):
    """{tid: (thread_name, utime+stime)}"""
    out = {}
    for t in glob.glob(f"/proc/{pid}/task/[0-9]*"):
        stat = read_file(f"{t}/stat")
        if not stat:
            continue
        # comm may contain spaces; it is wrapped in the last parentheses pair
        rpar = stat.rfind(")")
        name = stat[stat.find("(") + 1:rpar]
        fields = stat[rpar + 2:].split()
        out[int(os.path.basename(t))] = (name, int(fields[11]) + int(fields[12]))
    return out


class KmsgReader:
    """Non-blocking tail of /dev/kmsg (needs root when dmesg_restrict=1)."""

    def __init__(self):
        self.fd = None
        self.error = None
        try:
            self.fd = os.open("/dev/kmsg", os.O_RDONLY | os.O_NONBLOCK)
            os.lseek(self.fd, 0, os.SEEK_END)
        except OSError as e:
            self.error = str(e)

    def read(self):
        """(warnings, info): relevant messages at warning level or worse, and the rest of them."""
        warn, info = [], []
        while self.fd is not None:
            try:
                rec = os.read(self.fd, 8192).decode(errors="replace")
            except BlockingIOError:
                break
            except OSError as e:
                if e.errno == errno.EPIPE:  # records overwritten, just continue
                    continue
                break
            head, _, rest = rec.partition(";")
            msg = rest.split("\n", 1)[0]
            if not KMSG_INTEREST.search(msg):
                continue
            level = num(head.split(",", 1)[0], 6) & 7
            (warn if level <= 4 and not KMSG_TIMING.match(msg) else info).append(msg)
        return warn, info


# ---------------------------------------------------------------- evaluation

def evaluate(s, irq_cpus, irq_median):
    """Turn one sample into a list of flag strings."""
    flags = []

    drv = sorted(s["driver"].items())
    # device-wide states: once per sample, naming the channels
    for key, flag in (("card_lost", "CARD-LOST"), ("irq_paused", "IRQ-PAUSED")):
        names = [name for name, e in drv if e[key]]
        if names:
            flags.append(f"{flag} {','.join(names)}" + (" (interrupt storm)" if key == "irq_paused" else ""))

    for name, e in drv:
        if e.get("baseline"):
            continue  # first sample of this channel, or the driver was just reloaded
        if not e["in_use"]:
            continue  # nobody uses this channel: its source coming and going doesn't matter
        if "signal" in e["changed"]:
            flags.append(f"SIGNAL {name} {'back' if e['signal'] else 'lost'}")
        elif "input" in e["changed"] and e["signal"]:
            flags.append(f"FORMAT {name} input now {e['input']}")
        if e["settling"]:
            continue  # stream start, signal or format change: partly representative
        # audio can run without a video stream (an app that only records sound)
        if e["audio_stall"]:
            flags.append(f"AUDIO-SILENCE {name} {e['audio_silence_packets']} packets filled")
        if e["audio_none"]:
            flags.append(f"AUDIO-NONE {name} the source sends no audio (silence from now on)")
        if not (e["streaming"] and e["was_streaming"] and e["signal"]):
            continue  # no signal: the driver shows its "no signal" picture, nothing to judge
        if e["lost"]:
            flags.append(f"DRV-LOST {name} {e['lost']} frames the card completed never reached the driver")
        drops = [f"{k} {e[k]}" for k in DRV_DROPS if e.get(k)]
        if drops:
            flags.append(f"DRV-DROP {name} " + " ".join(drops))
        # whole frames: one fewer than expected is just where the sample fell. Interlaced
        # input before the exact rate is measured: the card's figure may be the field rate
        expect = e["fps"] / 2 if e["input"].endswith("i") and not e["measured"] else e["fps"]
        if e["signal"] and expect and e["card_frames"] < 0.9 * expect * e["dt"] - 1:
            flags.append(f"CARD-SLOW {name} {e['card_frames'] / e['dt']:.1f}fps<{e['fps']:.2f}")
        if e["no_app_buf"]:
            flags.append(f"NO-APP-BUF {name} x{e['no_app_buf']}")

    for dev, f in sorted(s["frames"].items()):
        name = os.path.basename(dev)
        if "error" in f:
            if f.get("new"):  # once per error, not every second until the probe is back
                flags.append(f"PROBE-ERROR {name} {f['error']}")
            continue
        if f.get("skip"):
            continue  # no signal ("no signal" picture at its own rate) or the input just changed
        period = f["period_ms"]
        if f["missed"]:
            flags.append(f"FRAME-GAP {name} missed{f['missed']} max{f['iv_max_ms']:.0f}ms")
        if f["dups"]:
            flags.append(f"DUP-FRAMES {name} x{f['dups']} (same content {f['dups_same_content']})")
        if period and f["fps"] < 0.9 * 1000 / period:
            flags.append(f"LOW-FPS {name} {f['fps']:.1f}<{1000 / period:.0f}")
        if period and not f["missed"] and f["iv_max_ms"] > 1.25 * period:
            flags.append(f"JITTER {name} {f['iv_min_ms']:.0f}-{f['iv_max_ms']:.0f}ms")
        if period and f["deliv_max_ms"] > period:
            flags.append(f"DELIVERY-LATE {name} {f['deliv_max_ms']:.0f}ms")

    irq_total = sum(s["irq_rate"].values())
    if irq_median and abs(irq_total - irq_median) > 0.1 * irq_median:
        flags.append(f"IRQ-RATE {irq_total}/s (usual {irq_median:.0f}/s)")

    for cpu, busy in s["cpu_busy"].items():
        if busy > 95:
            flags.append(f"CPU-SAT cpu{cpu}")
    for cpu in irq_cpus:
        if s["cpu_busy"].get(cpu, 0) > 85:
            flags.append(f"IRQ-CPU-BUSY cpu{cpu} {s['cpu_busy'][cpu]:.0f}%")

    psi = s["psi_pct"]
    if psi.get("cpu", 0) > 10:
        flags.append(f"CPU-PRESSURE {psi['cpu']:.0f}%")
    if psi.get("memory", 0) > 1:
        flags.append(f"MEM-PRESSURE {psi['memory']:.0f}%")
    if psi.get("io", 0) > 10:
        flags.append(f"IO-PRESSURE {psi['io']:.0f}%")

    for cpu, ratio in s["freq_ratio"].items():
        if s["cpu_busy"].get(cpu, 0) > 70 and ratio < 0.6:
            flags.append(f"LOW-FREQ cpu{cpu} {ratio * 100:.0f}%")
    if s["throttle_delta"] > 0:
        flags.append(f"THERMAL-THROTTLE +{s['throttle_delta']}")

    for name, pct in s["hot_threads"]:
        flags.append(f"APP-THREAD-SAT {name} {pct:.0f}%")

    for msg in s["kmsg"]:
        flags.append(f"KERNEL {msg[:60]}")

    return flags


def fmt_line(s, flags, color):
    ts = time.strftime("%H:%M:%S", time.localtime(s["time"]))
    parts = []
    for name, e in sorted(s["driver"].items()):
        if e["streaming"] and "dt" in e and e["dt"] > 0:
            dt = e["dt"]
            parts.append(f"{name} {e['input'] if e['signal'] else 'no signal'}->{e['output']} "
                         f"card {e['card_frames'] / dt:.0f} in {e['frames_in'] / dt:.0f} "
                         f"out {e['frames_out'] / dt:.0f}/s apps {e['apps']}")
    for dev, f in sorted(s["frames"].items()):
        if "error" not in f:
            parts.append(f"{os.path.basename(dev)} {f['fps']:.1f}fps iv{f['iv_min_ms']:.0f}-{f['iv_max_ms']:.0f}ms "
                         f"lat{f['deliv_max_ms']:.0f}ms")
    busiest = max(s["cpu_busy"].items(), key=lambda kv: kv[1], default=(None, 0))
    if busiest[0] is not None:
        parts.append(f"cpu max {busiest[1]:.0f}%(cpu{busiest[0]})")
    if s["irq_rate"]:
        parts.append("irq " + ",".join(f"cpu{c}:{r}/s" for c, r in sorted(s["irq_rate"].items()) if r))
    psi = s["psi_pct"]
    if psi:
        parts.append("psi " + " ".join(f"{k[:3]}{v:.0f}%" for k, v in psi.items()))
    line = f"{ts}  " + " | ".join(parts)
    if flags:
        tag = "  !! " + "; ".join(flags)
        line += f"\033[31m{tag}\033[0m" if color else tag
    return line


# ---------------------------------------------------------------- main

def interval_arg(v):
    x = float(v)
    if not x >= 0.1:
        raise argparse.ArgumentTypeError("must be at least 0.1 seconds")
    return x


def fps_arg(v):
    x = float(v)
    if not 0 < x < 1000:   # also rejects nan and inf
        raise argparse.ArgumentTypeError("must be a frame rate between 0 and 1000")
    return x


class Output:
    """stdout that survives a closed pipe (| head) or a vanished terminal (ssh dropped)."""

    def __init__(self):
        self.closed = False

    def write(self, line):
        if self.closed:
            return False
        try:
            print(line, flush=True)
        except BrokenPipeError:
            # reader went away: silence stdout for good, keep the log going until told to stop
            self.closed = True
            try:
                os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
            except OSError:
                pass
        except OSError:
            self.closed = True
        return not self.closed


class Summary:
    """Running counts for the end-of-run summary (no per-sample history kept)."""

    def __init__(self):
        self.n = 0
        self.counts = collections.Counter()
        self.bad = 0
        self.cooc = collections.Counter()

    def add(self, flags):
        kinds = {f.split()[0] for f in flags}
        self.n += 1
        self.counts.update(kinds)
        if kinds & FRAME_FLAGS:
            self.bad += 1
            self.cooc.update(k for k in kinds if k not in FRAME_FLAGS)

    def as_dict(self):
        return {"samples": self.n, "frame_problem_samples": self.bad, "flags": dict(self.counts),
                "with_frame_problems": dict(self.cooc)}


class Guard:
    """Runs one sampler; an unexpected error costs that reading, not the whole run."""

    def __init__(self):
        self.errors = []
        self.seen = set()

    def __call__(self, what, fn, *args, default=None):
        try:
            return fn(*args)
        except Exception as e:  # noqa: BLE001
            msg = f"{what}: {type(e).__name__}: {e}"
            if msg not in self.seen:  # report each distinct error once
                self.seen.add(msg)
                self.errors.append(msg)
            return default

    def take(self):
        errs, self.errors = self.errors, []
        return errs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", action="append", default=[],
                    help="V4L2 device to probe for frame timing (repeatable), e.g. /dev/video0")
    ap.add_argument("--fps", type=fps_arg,
                    help="expected frame rate of the source (default: learn from the measured intervals)")
    ap.add_argument("--queued", type=int, default=2,
                    help="buffers the probe keeps queued (default 2). The stock driver fills every queued "
                         "buffer with the same frame, so >1 reproduces the duplicate bursts apps see; "
                         "1 adds the least load but may report false gaps if the probe is slow")
    ap.add_argument("--interval", type=interval_arg, default=1.0, help="sample interval in seconds (default 1)")
    ap.add_argument("--app", default="obs,ffmpeg,gst-launch,vlc,mpv",
                    help="comma-separated process name prefixes to watch (default: %(default)s)")
    ap.add_argument("--log", help="write every sample as JSON lines to this file")
    ap.add_argument("--quiet", action="store_true", help="only print seconds with flags")
    ap.add_argument("--no-color", action="store_true")
    args = ap.parse_args()
    color = sys.stdout.isatty() and not args.no_color
    out = Output()

    try:
        log = open(args.log, "a") if args.log else None
    except OSError as e:
        ap.error(f"cannot open log file: {e}")

    pci = card_pci_addrs()
    if not pci:
        print(f"warning: no PCI device bound to {DRIVER} yet; IRQ tracking starts when it appears",
              file=sys.stderr)
    kmsg = KmsgReader()
    if kmsg.error:
        print(f"warning: cannot read /dev/kmsg ({kmsg.error}); kernel errors not tracked (run as root)",
              file=sys.stderr)
    tracker = DriverTracker()
    first = read_driver_stats()
    tracker.update(first)
    if not first:
        print(f"note: no driver stats files (/sys/class/video4linux/video*/stats) - "
              f"needs this repository's {DRIVER} driver", file=sys.stderr)

    probes, fstats, retry = {}, {}, {}

    def start_probe(dev):
        probes[dev] = FrameProbe(dev, args.queued)
        fstats[dev] = FrameStats(args.fps)
        probes[dev].start()

    for dev in args.device:
        start_probe(dev)
        retry[dev] = {"at": None, "wait": 5.0, "reported": None}

    app_names = [n for n in args.app.split(",") if n]

    out.write(f"card: {', '.join(pci) or '-'}   driver stats: {', '.join(first) or '-'}   "
              f"probing: {', '.join(args.device) or '-'}   "
              f"watching apps: {', '.join(app_names)}   Ctrl+C for summary\n")

    guard = Guard()
    prev_cpu = guard("cpu", read_cpu_times, default={})
    prev_irq = guard("irq", read_irq_counts, pci, default={})
    prev_psi = guard("psi", read_psi, default={})
    prev_throttle = guard("thermal", read_throttle_total, default=0)
    prev_threads = {}
    prev_t = time.monotonic()
    irq_hist = collections.deque(maxlen=60)
    irq_state = None
    summary = Summary()
    stop = threading.Event()

    def on_signal(sig, _frame):
        if stop.is_set():  # second Ctrl+C: leave now, even if a read hangs
            signal.signal(sig, signal.SIG_DFL)
            os.kill(os.getpid(), sig)
        stop.set()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        if signal.getsignal(sig) is not signal.SIG_IGN:   # nohup: keep running when the terminal closes
            signal.signal(sig, on_signal)

    n = 0
    next_t = time.monotonic()
    try:
        while not stop.is_set():
            # fixed schedule: work time doesn't add up to drift over hours
            next_t += args.interval
            delay = next_t - time.monotonic()
            if delay < -args.interval:   # fell far behind (stalled system): start counting afresh
                next_t = time.monotonic()
            elif delay > 0:
                stop.wait(delay)
            if stop.is_set():
                break
            n += 1

            # driver counters first, right after waking: they carry their own timestamps
            drv = guard("driver stats", read_driver_stats, default={})
            driver = guard("driver stats", tracker.update, drv, default={})

            now = time.monotonic()
            dt = now - prev_t
            prev_t = now

            frames = {}
            for dev in args.device:
                probe, r = probes[dev], retry[dev]
                if probe.error:
                    frames[dev] = {"error": probe.error, "new": probe.error != r["reported"]}
                    r["reported"] = probe.error
                    if r["at"] is None:
                        r["at"] = now + r["wait"]
                    elif now >= r["at"] and not probe.is_alive():
                        # try again (device came back, driver reloaded), waiting longer each time
                        start_probe(dev)
                        r["at"], r["wait"] = None, min(r["wait"] * 2, 60.0)
                    continue
                events = probe.take()
                if events:   # working again: the next error is news, retry quickly
                    r["wait"], r["reported"] = 5.0, None
                f = guard("probe", fstats[dev].update, events, dt)
                if f:
                    # the driver's view of this channel (/dev/v4l/by-id links resolve to videoN)
                    de = driver.get(os.path.basename(os.path.realpath(dev)))
                    if de and (de.get("baseline") or de.get("settling") or not de["signal"]):
                        f["skip"] = True
                        if de.get("baseline") or de.get("changed"):
                            fstats[dev].reset()   # new mode: learn its frame period afresh
                    frames[dev] = f

            cpu = guard("cpu", read_cpu_times, default={})
            cpu_busy = {}
            for c, (busy, total) in cpu.items():
                if c in prev_cpu and total > prev_cpu[c][1]:
                    cpu_busy[c] = 100.0 * (busy - prev_cpu[c][0]) / (total - prev_cpu[c][1])
            prev_cpu = cpu

            if n % 10 == 0:   # the driver may have been loaded or reloaded since
                now_pci = guard("pci", card_pci_addrs, default=pci)
                if now_pci != pci:
                    pci, prev_irq = now_pci, {}
                    irq_hist.clear()
            irq = guard("irq", read_irq_counts, pci, default={})
            fresh = not prev_irq or set(irq) != set(prev_irq) or any(irq[c] < prev_irq[c] for c in irq)
            irq_rate = {} if fresh else {c: int((irq[c] - prev_irq[c]) / dt) for c in irq}
            prev_irq = irq
            irq_cpus = [c for c, r in irq_rate.items() if r > 0]
            # the usual rate changes with what the card is doing: start the baseline over
            state = tuple(sorted((name, e["streaming"], e["audio_running"], e["signal"], e["input"],
                                  round(e["fps"], 1), e["card_lost"], e["irq_paused"])
                                 for name, e in driver.items() if e["in_use"] or e["card_lost"] or e["irq_paused"]))
            if state != irq_state or fresh:
                irq_hist.clear()
                irq_state = state
            irq_median = statistics.median(irq_hist) if len(irq_hist) >= 5 else None
            if irq_rate:
                irq_hist.append(sum(irq_rate.values()))

            psi = guard("psi", read_psi, default={})
            psi_pct = {k: 100.0 * max(0, psi[k] - prev_psi.get(k, psi[k])) / (dt * 1e6) for k in psi}
            prev_psi = psi

            freq_ratio = {c: cur / mx for c, (cur, mx) in guard("cpufreq", read_freqs, default={}).items() if mx}
            throttle = guard("thermal", read_throttle_total, default=prev_throttle)
            throttle_delta = max(0, throttle - prev_throttle)
            prev_throttle = throttle

            hot_threads = []
            threads_now = {}
            for pid, comm in guard("apps", find_app_pids, app_names, default=[]):
                for tid, (tname, ticks) in guard("apps", read_thread_ticks, pid, default={}).items():
                    threads_now[tid] = ticks
                    if tid in prev_threads:
                        pct = 100.0 * (ticks - prev_threads[tid]) / CLK_TCK / dt
                        if pct > 90:
                            hot_threads.append((f"{comm}/{tname}", pct))
            prev_threads = threads_now

            kwarn, kinfo = guard("kmsg", kmsg.read, default=([], []))
            sample = {
                "time": time.time(), "dt": dt, "driver": driver, "frames": frames, "cpu_busy": cpu_busy,
                "irq_rate": irq_rate, "psi_pct": psi_pct, "freq_ratio": freq_ratio,
                "throttle_delta": throttle_delta, "hot_threads": hot_threads, "kmsg": kwarn, "kmsg_info": kinfo,
            }
            errors = guard.take()
            flags = guard("evaluate", evaluate, sample, irq_cpus, irq_median, default=[])
            errors += guard.take()
            flags += [f"MONITOR-ERROR {m[:80]}" for m in errors]
            sample["monitor_errors"] = errors
            sample["flags"] = flags
            summary.add(flags)

            if flags or not args.quiet:
                line = guard("output", fmt_line, sample, flags, color, default=None)
                out.write(line if line is not None else f"{time.strftime('%H:%M:%S')}  !! " + "; ".join(flags))
            if log:
                try:
                    log.write(json.dumps(sample, default=str) + "\n")
                    log.flush()
                except OSError as e:
                    print(f"warning: log write failed ({e}); logging stopped", file=sys.stderr)
                    log = None
            if out.closed and not log:
                break   # nobody is listening any more
    finally:
        for probe in probes.values():
            probe.running = False
        for probe in probes.values():
            probe.join(timeout=2)
        print_summary(summary, out)
        if log:
            try:
                log.write(json.dumps({"time": time.time(), "summary": summary.as_dict()}) + "\n")
                log.close()
            except OSError:
                pass


def print_summary(sm, out):
    if not sm.n:
        return
    w = out.write
    w(f"\n==== summary: {sm.n} samples, {sm.bad} with frame problems ====")
    if not sm.counts:
        w("no problems flagged")
        return
    w("flag counts (samples):")
    for k, v in sm.counts.most_common():
        w(f"  {k:<18} {v:>5}  ({100.0 * v / sm.n:.1f}%)")
    if sm.bad:
        w("system events at the same time as frame problems:")
        if sm.cooc:
            for k, v in sm.cooc.most_common():
                w(f"  {k:<18} {v:>5} of {sm.bad}")
        else:
            w("  none - frame problems happen without visible system load (driver/hardware/source)")

    w("\nhints:")
    hints = {
        "FRAME-GAP": "frames missing between two delivered frames - dropped in source, card or driver",
        "DUP-FRAMES": "one frame delivered to several buffers in a burst - driver fan-out hands each frame "
                      "to every queued buffer; apps see repeats then gaps",
        "LOW-FPS": "fewer frames than the source rate (check --fps / camera setting)",
        "JITTER": "uneven frame spacing without loss - late driver worker or busy CPU",
        "DELIVERY-LATE": "frame reached user space more than a frame period after the driver stamped it",
        "IRQ-RATE": "card interrupt rate deviates from normal while nothing changed on the driver side - "
                    "source/cable/card instability",
        "PROBE-ERROR": "the frame probe could not read the device (retried with growing pauses)",
        "MONITOR-ERROR": "the monitor itself failed to read something - that reading is missing, the run went on",
        "CPU-SAT": "a core is saturated",
        "IRQ-CPU-BUSY": "the core handling the card's IRQ is busy - try moving it via "
                        "/proc/irq/<n>/smp_affinity_list",
        "CPU-PRESSURE": "tasks wait for CPU - system overloaded",
        "MEM-PRESSURE": "memory reclaim stalls", "IO-PRESSURE": "disk IO stalls (recording to slow disk?)",
        "LOW-FREQ": "busy core running at low clock - set cpufreq governor to performance",
        "THERMAL-THROTTLE": "CPU throttled by temperature",
        "APP-THREAD-SAT": "an app thread is pinned at 100% of a core - that thread is the bottleneck",
        "KERNEL": "relevant kernel messages - check dmesg",
        "DRV-LOST": "the card completed frames the driver never took in (and they stayed missing) - "
                    "a driver problem, please report",
        "DRV-DROP": "the driver dropped frames (DMA overrun / stale / unpaired halves, copy_missed, ring "
                    "overwritten or skipped) - usually its copy thread ran too late: look at CPU and IRQ flags "
                    "in the same seconds. aborted: given up during a format change or while apps stopped",
        "CARD-SLOW": "the card itself delivered fewer frames than the input rate - source, cable or card "
                     "(not the driver)",
        "NO-APP-BUF": "a frame arrived while the application had no free buffer - the application is too slow "
                      "or queues too few buffers",
        "SIGNAL": "the input signal was lost or came back",
        "FORMAT": "the input resolution changed",
        "AUDIO-SILENCE": "audio from the card stalled and the driver filled the gap with silence",
        "AUDIO-NONE": "the source sends no audio at all (check its audio output / HDMI audio setting)",
        "CARD-LOST": "the card stopped responding on the PCIe bus",
        "IRQ-PAUSED": "the driver paused the card's interrupts after an interrupt storm (it retries every second)",
    }
    for k, _ in sm.counts.most_common():
        if k in hints:
            w(f"  {k}: {hints[k]}")


if __name__ == "__main__":
    main()
