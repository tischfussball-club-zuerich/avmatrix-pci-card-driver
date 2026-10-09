#!/usr/bin/env python3
"""
stutter-monitor: watch the whole capture pipeline for causes of stutter.
Works with the stock HwsCapture driver - no driver changes or debug logs needed.

Samples once per interval and flags anything that can make video stutter:
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

Ctrl+C prints a summary that correlates frame problems with system events.

Examples:
  sudo ./stutter-monitor.py --device /dev/video0
  sudo ./stutter-monitor.py --device /dev/video0 --fps 50 --quiet --log run.jsonl
  ./stutter-monitor.py                       # system only, no frame probe
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

# Flags that mean "the frames themselves were bad" - correlated against the rest
FRAME_FLAGS = ("FRAME-GAP", "DUP-FRAMES", "LOW-FPS", "JITTER", "DELIVERY-LATE", "IRQ-RATE", "PROBE-ERROR")


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
        except OSError as e:
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
        os.close(self.fd)
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
        msgs = []
        while self.fd is not None:
            try:
                rec = os.read(self.fd, 8192).decode(errors="replace")
            except BlockingIOError:
                break
            except OSError as e:
                if e.errno == errno.EPIPE:  # records overwritten, just continue
                    continue
                break
            msg = rec.split(";", 1)[-1].split("\n", 1)[0]
            if KMSG_INTEREST.search(msg):
                msgs.append(msg)
        return msgs


# ---------------------------------------------------------------- evaluation

def evaluate(s, irq_cpus, irq_median):
    """Turn one sample into a list of flag strings."""
    flags = []

    for dev, f in sorted(s["frames"].items()):
        name = os.path.basename(dev)
        if "error" in f:
            flags.append(f"PROBE-ERROR {name} {f['error']}")
            continue
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

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", action="append", default=[],
                    help="V4L2 device to probe for frame timing (repeatable), e.g. /dev/video0")
    ap.add_argument("--fps", type=float,
                    help="expected frame rate of the source (default: learn from the measured intervals)")
    ap.add_argument("--queued", type=int, default=2,
                    help="buffers the probe keeps queued (default 2). The stock driver fills every queued "
                         "buffer with the same frame, so >1 reproduces the duplicate bursts apps see; "
                         "1 adds the least load but may report false gaps if the probe is slow")
    ap.add_argument("--interval", type=float, default=1.0, help="sample interval in seconds (default 1)")
    ap.add_argument("--app", default="obs,ffmpeg,gst-launch,vlc,mpv",
                    help="comma-separated process name prefixes to watch (default: %(default)s)")
    ap.add_argument("--log", help="write every sample as JSON lines to this file")
    ap.add_argument("--quiet", action="store_true", help="only print seconds with flags")
    ap.add_argument("--no-color", action="store_true")
    args = ap.parse_args()
    color = sys.stdout.isatty() and not args.no_color

    pci = card_pci_addrs()
    if not pci:
        print(f"warning: no PCI device bound to {DRIVER}; IRQ tracking disabled", file=sys.stderr)
    kmsg = KmsgReader()
    if kmsg.error:
        print(f"warning: cannot read /dev/kmsg ({kmsg.error}); kernel errors not tracked (run as root)",
              file=sys.stderr)
    if not args.device:
        print("note: no --device given - frame timing is not measured, only the system", file=sys.stderr)

    probes = {}
    stats = {}
    for dev in args.device:
        probes[dev] = FrameProbe(dev, args.queued)
        stats[dev] = FrameStats(args.fps)
        probes[dev].start()

    app_names = [n for n in args.app.split(",") if n]
    log = open(args.log, "a") if args.log else None

    print(f"card: {', '.join(pci) or '-'}   probing: {', '.join(args.device) or '-'}   "
          f"watching apps: {', '.join(app_names)}   Ctrl+C for summary\n")

    prev_cpu = read_cpu_times()
    prev_irq = read_irq_counts(pci)
    prev_psi = read_psi()
    prev_throttle = read_throttle_total()
    prev_threads = {}
    prev_t = time.monotonic()
    irq_hist = collections.deque(maxlen=60)

    history = []
    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    while not stop:
        time.sleep(args.interval)
        now = time.monotonic()
        dt = now - prev_t
        prev_t = now

        frames = {}
        for dev, probe in probes.items():
            if probe.error:
                frames[dev] = {"error": probe.error}
            else:
                frames[dev] = stats[dev].update(probe.take(), dt)

        cpu = read_cpu_times()
        cpu_busy = {}
        for c, (busy, total) in cpu.items():
            if c in prev_cpu and total > prev_cpu[c][1]:
                cpu_busy[c] = 100.0 * (busy - prev_cpu[c][0]) / (total - prev_cpu[c][1])
        prev_cpu = cpu

        irq = read_irq_counts(pci)
        irq_rate = {c: int((irq[c] - prev_irq.get(c, 0)) / dt) for c in irq}
        prev_irq = irq
        irq_cpus = [c for c, r in irq_rate.items() if r > 0]
        irq_median = statistics.median(irq_hist) if len(irq_hist) >= 5 else None
        if irq_rate:
            irq_hist.append(sum(irq_rate.values()))

        psi = read_psi()
        psi_pct = {k: 100.0 * (psi[k] - prev_psi.get(k, psi[k])) / (dt * 1e6) for k in psi}
        prev_psi = psi

        freq_ratio = {c: cur / mx for c, (cur, mx) in read_freqs().items() if mx}
        throttle = read_throttle_total()
        throttle_delta = throttle - prev_throttle
        prev_throttle = throttle

        hot_threads = []
        threads_now = {}
        for pid, comm in find_app_pids(app_names):
            for tid, (tname, ticks) in read_thread_ticks(pid).items():
                threads_now[tid] = ticks
                if tid in prev_threads:
                    pct = 100.0 * (ticks - prev_threads[tid]) / CLK_TCK / dt
                    if pct > 90:
                        hot_threads.append((f"{comm}/{tname}", pct))
        prev_threads = threads_now

        sample = {
            "time": time.time(), "frames": frames, "cpu_busy": cpu_busy, "irq_rate": irq_rate,
            "psi_pct": psi_pct, "freq_ratio": freq_ratio, "throttle_delta": throttle_delta,
            "hot_threads": hot_threads, "kmsg": kmsg.read(),
        }
        flags = evaluate(sample, irq_cpus, irq_median)
        sample["flags"] = flags
        history.append(flags)

        if flags or not args.quiet:
            print(fmt_line(sample, flags, color), flush=True)
        if log:
            log.write(json.dumps(sample) + "\n")
            log.flush()

    for probe in probes.values():
        probe.running = False
    for probe in probes.values():
        probe.join(timeout=2)

    print_summary(history)


def print_summary(history):
    n = len(history)
    if not n:
        return
    kind = lambda f: f.split()[0]
    counts = collections.Counter(kind(f) for flags in history for f in set(flags))
    bad_secs = [flags for flags in history if any(kind(f) in FRAME_FLAGS for f in flags)]
    cooc = collections.Counter(kind(f) for flags in bad_secs for f in set(flags) if kind(f) not in FRAME_FLAGS)

    print(f"\n==== summary: {n} samples, {len(bad_secs)} with frame problems ====")
    if not counts:
        print("no problems flagged")
        return
    print("flag counts (samples):")
    for k, v in counts.most_common():
        print(f"  {k:<18} {v:>5}  ({100.0 * v / n:.1f}%)")
    if bad_secs:
        print("system events at the same time as frame problems:")
        if cooc:
            for k, v in cooc.most_common():
                print(f"  {k:<18} {v:>5} of {len(bad_secs)}")
        else:
            print("  none - frame problems happen without visible system load (driver/hardware/source)")

    print("\nhints:")
    hints = {
        "FRAME-GAP": "frames missing between two delivered frames - dropped in source, card or driver",
        "DUP-FRAMES": "one frame delivered to several buffers in a burst - driver fan-out hands each frame "
                      "to every queued buffer; apps see repeats then gaps",
        "LOW-FPS": "fewer frames than the source rate (check --fps / camera setting)",
        "JITTER": "uneven frame spacing without loss - late driver worker or busy CPU",
        "DELIVERY-LATE": "frame reached user space more than a frame period after the driver stamped it",
        "IRQ-RATE": "card interrupt rate deviates from normal - source/cable/card instability",
        "PROBE-ERROR": "the frame probe could not read the device",
        "CPU-SAT": "a core is saturated",
        "IRQ-CPU-BUSY": "the core handling the card's IRQ is busy - try moving it via "
                        "/proc/irq/<n>/smp_affinity_list",
        "CPU-PRESSURE": "tasks wait for CPU - system overloaded",
        "MEM-PRESSURE": "memory reclaim stalls", "IO-PRESSURE": "disk IO stalls (recording to slow disk?)",
        "LOW-FREQ": "busy core running at low clock - set cpufreq governor to performance",
        "THERMAL-THROTTLE": "CPU throttled by temperature",
        "APP-THREAD-SAT": "an app thread is pinned at 100% of a core - that thread is the bottleneck",
        "KERNEL": "relevant kernel messages - check dmesg",
    }
    for k, _ in counts.most_common():
        if k in hints:
            print(f"  {k}: {hints[k]}")


if __name__ == "__main__":
    main()
