#!/bin/bash
# Check the HwsCapture driver on the real card: V4L2 conformance, frame pacing
# and drops per channel, audio capture, and kernel messages. v4l2-compliance
# changes format and picture controls while it tests; the script puts them back
# afterwards. Connect a source to the inputs you want checked; channels without
# a signal are reported as such.
#
#   tools/card-check.sh            # all channels, 10 s capture each
#   DUR=30 tools/card-check.sh     # longer captures
#
# Needs v4l-utils (v4l2-ctl, v4l2-compliance) and alsa-utils (arecord).
set -u
DUR=${DUR:-10}
# only kernel messages from this run count
dmesg_ok=1; dmesg_start=$(dmesg 2>/dev/null | wc -l); dmesg >/dev/null 2>&1 || dmesg_ok=0
fail=0
say() { printf '%s\n' "$*"; }
bad() { printf '  !! %s\n' "$*"; fail=1; }

for t in v4l2-ctl v4l2-compliance arecord; do
	command -v $t >/dev/null || { say "missing $t (install v4l-utils / alsa-utils)"; exit 2; }
done
lsmod | grep -q '^HwsCapture' || { say "HwsCapture module not loaded"; exit 2; }
say "== module: $(modinfo -F version HwsCapture 2>/dev/null) params: direct_io=$(cat /sys/module/HwsCapture/parameters/direct_io 2>/dev/null) nt_copy=$(cat /sys/module/HwsCapture/parameters/nt_copy 2>/dev/null)"

nodes=()
for d in /dev/video*; do
	v4l2-ctl -d "$d" --info 2>/dev/null | grep -q 'Driver name *: *HwsCapture' && nodes+=("$d")
done
[ ${#nodes[@]} -gt 0 ] || { say "no HwsCapture video devices found"; exit 2; }
say "== ${#nodes[@]} video channel(s): ${nodes[*]}"

for d in "${nodes[@]}"; do
	say ""
	say "== $d ($(v4l2-ctl -d "$d" --info | sed -n 's/.*Card type *: *//p'))"
	nosig=$(v4l2-ctl -d "$d" --get-input --list-inputs 2>/dev/null | grep -c 'no signal')
	fmt=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *//p')
	fps=$(v4l2-ctl -d "$d" --get-parm | sed -n 's/.*Frames per second: *\([0-9.]*\).*/\1/p')
	say "   format $fmt @ ${fps:-?} fps$([ "$nosig" -gt 0 ] && echo ', NO SIGNAL')"

	# conformance (the per-app queues make the "second open can't stream" tests
	# fail by design: every app gets its own copy of every frame)
	saved_fmt=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *\([0-9]*\)\/\([0-9]*\).*/width=\1,height=\2/p')
	saved_ctrls=$(v4l2-ctl -d "$d" --list-ctrls | sed -n 's/^ *\([a-z_]*\) .*value=\(-*[0-9]*\).*/\1=\2/p' | paste -sd, -)
	comp=$(v4l2-compliance -d "$d" -s 30 2>&1)
	[ -n "$saved_fmt" ] && v4l2-ctl -d "$d" --set-fmt-video="$saved_fmt,pixelformat=YUYV" 2>/dev/null
	[ -n "$saved_ctrls" ] && v4l2-ctl -d "$d" --set-ctrl="$saved_ctrls" 2>/dev/null
	sum=$(printf '%s\n' "$comp" | grep -E '^Total for' | tail -1)
	say "   v4l2-compliance: ${sum:-no summary}"
	printf '%s\n' "$comp" | grep -E 'fail:' | sed 's/^/      /' | head -12

	# capture: sequence gaps = frames the app lost, timestamp gaps = pacing
	[ "$nosig" -gt 0 ] && { say "   capture skipped: no signal"; continue; }
	n=$(awk -v f="${fps:-60}" -v t="$DUR" 'BEGIN { printf "%d", f * t }')
	v4l2-ctl -d "$d" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose 2>&1 |
	awk -v want="$n" -v fps="${fps:-60}" '
		/seq:/ { for (i = 1; i <= NF; i++) { if ($i == "seq:") s = $(i+1); if ($i == "ts:") t = $(i+1) }
			 if (cnt++ && s != ps + 1) lost += s - ps - 1
			 if (cnt > 1) { dt = (t - pt) * 1000; if (dt > max) max = dt; if (dt > 1500 / fps) late++ }
			 ps = s; pt = t }
		END { printf "   capture: %d frames, %d lost, %d late (>1.5 frame periods), max gap %.1f ms\n",
			  cnt, lost, late, max
		      if (cnt < want * 0.95 || lost > 0) exit 1 }' || bad "frames lost or capture incomplete on $d"
done

say ""
cards=$(arecord -l 2>/dev/null | sed -n 's/^card [0-9]*: \(HAudio[0-9]*\) .*/\1/p')
for c in $cards; do
	out=$(arecord -q -D "hw:CARD=$c" -f S16_LE -r 48000 -c 2 -d 5 /dev/null 2>&1); rc=$?
	if [ $rc -ne 0 ]; then bad "$c: arecord failed: $out"
	elif echo "$out" | grep -qi overrun; then bad "$c: overruns: $out"
	else say "== $c: 5 s audio capture ok"; fi
done

say ""
if [ $dmesg_ok = 0 ]; then
	say "== kernel messages: not readable (run as root, or: sudo dmesg | grep -i hwscapture)"
else
	new=$(dmesg | tail -n +$((dmesg_start + 1)))
	say "== kernel messages during this run (HwsCapture)"
	printf '%s\n' "$new" | grep -i hwscapture | tail -20 | sed 's/^/   /'
	# a crash in any module while we drove the card counts; older messages don't
	probs=$(printf '%s\n' "$new" | grep -iE 'hwscapture.*(stuck|not respond|stopped|failed)|BUG:|Oops|WARNING: CPU' | head)
	[ -n "$probs" ] && { say "$probs" | sed 's/^/   !! /'; fail=1; }
fi

say ""
say "RESULT: $([ $fail = 0 ] && echo OK || echo 'PROBLEMS FOUND (see !! above)')"
say "For frame pacing over time: tools/stutter-monitor.py --device ${nodes[0]}"
exit $fail
