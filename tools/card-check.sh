#!/bin/bash
# Check the HwsCapture driver on the real card. Connect a source to the inputs
# you want checked; channels without a signal are reported as such.
#
#   1. per channel: V4L2 conformance (v4l2-compliance), a capture at the input
#      size (direct path) and one at a different, scaled size (ring + conversion)
#   2. all channels with a signal at once, plus audio on every card at the same
#      time - the real streaming load - with CPU use and the driver's threads
#   3. the driver's own drop counters and problem messages from the kernel log
#
#   tools/card-check.sh                  # 10 s captures
#   DUR=60 tools/card-check.sh           # longer captures (each phase)
#   SKIP_COMPLIANCE=1 tools/card-check.sh   # e.g. to compare with another driver
#
# v4l2-compliance changes format and picture controls while it tests, the scaled
# capture changes the format; the script puts them back afterwards.
# Needs v4l-utils (v4l2-ctl, v4l2-compliance) and alsa-utils (arecord).
set -u
DUR=${DUR:-10}
SCALE_DUR=${SCALE_DUR:-$(( DUR < 10 ? DUR : 10 ))}
SKIP_COMPLIANCE=${SKIP_COMPLIANCE:-0}
TMP=$(mktemp -d /tmp/hws-check.XXXXXX)
trap 'rm -rf "$TMP"' EXIT
# kernel messages after a point in time. Timestamps, not line counts: once the
# kernel's log buffer is full, old lines drop out as new ones arrive.
dmesg_now() { dmesg 2>/dev/null | tail -1 | sed -n 's/^\[ *\([0-9.]*\)\].*/\1/p'; }
dmesg_since() { dmesg 2>/dev/null | awk -v t="${1:-0}" 'match($0, /^\[ *[0-9.]+\]/) { v = substr($0, RSTART + 1, RLENGTH - 2); gsub(/ /, "", v); if (v + 0 > t + 0) print }'; }
# only kernel messages from this run count
dmesg_ok=1; dmesg >/dev/null 2>&1 || dmesg_ok=0; dmesg_start=$(dmesg_now)
fail=0
say() { printf '%s\n' "$*"; }
bad() { printf '  !! %s\n' "$*"; fail=1; }

# analyze <v4l2-ctl --verbose output> <frames wanted> <fps> <expected bytesused> <label>
# sequence gaps = frames the app lost, timestamp gaps = pacing
analyze() {
	awk -v want="$2" -v fps="$3" -v size="$4" -v label="$5" '
		/seq:/ { for (i = 1; i <= NF; i++) {
				if ($i == "seq:") s = $(i+1)
				if ($i == "ts:") t = $(i+1)
				if ($i == "bytesused:") b = $(i+1) }
			 if (cnt++ && s != ps + 1) lost += s - ps - 1
			 if (cnt > 1) { dt = (t - pt) * 1000; if (dt > max) max = dt; if (dt > 1500 / fps) late++ }
			 if (size && b != size) wrong++
			 ps = s; pt = t }
		END { printf "   %s: %d frames, %d lost, %d late (>1.5 frame periods), max gap %.1f ms%s\n",
			  label, cnt, lost, late, max, wrong ? sprintf(", %d with wrong size", wrong) : ""
		      if (cnt < want * 0.95 || lost > 0 || wrong) exit 1 }' "$1"
}

for t in v4l2-ctl arecord; do
	command -v $t >/dev/null || { say "missing $t (install v4l-utils / alsa-utils)"; exit 2; }
done
[ "$SKIP_COMPLIANCE" = 1 ] || command -v v4l2-compliance >/dev/null || { say "missing v4l2-compliance (install v4l-utils)"; exit 2; }
lsmod | grep -q '^HwsCapture' || { say "HwsCapture module not loaded"; exit 2; }
say "== module: $(modinfo -F version HwsCapture 2>/dev/null) params: direct_io=$(cat /sys/module/HwsCapture/parameters/direct_io 2>/dev/null) nt_copy=$(cat /sys/module/HwsCapture/parameters/nt_copy 2>/dev/null) timing_debug=$(cat /sys/module/HwsCapture/parameters/timing_debug 2>/dev/null)"

nodes=()
for d in /dev/video*; do
	v4l2-ctl -d "$d" --info 2>/dev/null | grep -q 'Driver name *: *HwsCapture' && nodes+=("$d")
done
[ ${#nodes[@]} -gt 0 ] || { say "no HwsCapture video devices found"; exit 2; }
say "== ${#nodes[@]} video channel(s): ${nodes[*]}"

# ---------------------------------------------------------------- 1. per channel
sig_nodes=()
declare -A FPS
for d in "${nodes[@]}"; do
	say ""
	say "== $d ($(v4l2-ctl -d "$d" --info | sed -n 's/.*Card type *: *//p'))"
	nosig=$(v4l2-ctl -d "$d" --get-input --list-inputs 2>/dev/null | grep -c 'no signal')
	w=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *\([0-9]*\)\/.*/\1/p')
	h=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *[0-9]*\/\([0-9]*\).*/\1/p')
	fps=$(v4l2-ctl -d "$d" --get-parm | sed -n 's/.*Frames per second: *\([0-9.]*\).*/\1/p')
	fps=${fps:-60}; FPS[$d]=$fps
	say "   format ${w}x${h} @ $fps fps$([ "$nosig" -gt 0 ] && echo ', NO SIGNAL')"
	saved_fmt="width=$w,height=$h,pixelformat=YUYV"
	saved_ctrls=$(v4l2-ctl -d "$d" --list-ctrls | sed -n 's/^ *\([a-z_]*\) .*value=\(-*[0-9]*\).*/\1=\2/p' | paste -sd, -)

	# conformance (the per-app queues make the "second open can't stream" tests
	# fail by design: every app gets its own copy of every frame)
	if [ "$SKIP_COMPLIANCE" = 1 ]; then
		say "   v4l2-compliance: skipped"
	else
		comp=$(v4l2-compliance -d "$d" -s 30 2>&1)
		v4l2-ctl -d "$d" --set-fmt-video="$saved_fmt" 2>/dev/null
		[ -n "$saved_ctrls" ] && v4l2-ctl -d "$d" --set-ctrl="$saved_ctrls" 2>/dev/null
		sum=$(printf '%s\n' "$comp" | grep -E '^Total for' | tail -1)
		say "   v4l2-compliance: ${sum:-no summary}"
		printf '%s\n' "$comp" | grep -E 'fail:' | sed 's/^/      /' | head -12
	fi

	[ "$nosig" -gt 0 ] && { say "   captures skipped: no signal"; continue; }
	sig_nodes+=("$d")

	# input size: straight from DMA into the app's buffer
	n=$(awk -v f="$fps" -v t="$DUR" 'BEGIN { printf "%d", f * t }')
	v4l2-ctl -d "$d" --set-fmt-video="$saved_fmt" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/seq" 2>&1
	analyze "$TMP/seq" "$n" "$fps" $((w * h * 2)) "capture ${w}x${h}, ${DUR} s" || bad "frames lost or capture incomplete on $d"

	# another size: through the ring and the conversion thread
	if [ "$w" = 1920 ]; then sw=1280; sh=720; else sw=1920; sh=1080; fi
	n=$(awk -v f="$fps" -v t="$SCALE_DUR" 'BEGIN { printf "%d", f * t }')
	v4l2-ctl -d "$d" --set-fmt-video="width=$sw,height=$sh,pixelformat=YUYV" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/scaled" 2>&1
	analyze "$TMP/scaled" "$n" "$fps" $((sw * sh * 2)) "scaled to ${sw}x${sh}, ${SCALE_DUR} s" || bad "scaled capture failed on $d"
	# one scaled frame must look like a picture, not a blank or garbage buffer
	v4l2-ctl -d "$d" --set-fmt-video="width=$sw,height=$sh,pixelformat=YUYV" --stream-mmap=4 --stream-skip=10 --stream-count=1 --stream-to="$TMP/frame.yuv" >/dev/null 2>&1
	size=$(stat -c %s "$TMP/frame.yuv" 2>/dev/null || echo 0)
	vals=$(head -c 2000000 "$TMP/frame.yuv" 2>/dev/null | od -An -tu1 -v | tr -s ' ' '\n' | sort -un | wc -l)
	if [ "$size" -ne $((sw * sh * 2)) ]; then bad "scaled frame has $size bytes, expected $((sw * sh * 2))"
	elif [ "$vals" -lt 16 ]; then say "   scaled frame: only $vals different byte values (blank picture? check the source)"
	else say "   scaled frame: $size bytes, $vals different byte values"; fi
	v4l2-ctl -d "$d" --set-fmt-video="$saved_fmt" 2>/dev/null
done

# ---------------------------------------------------------------- 2. everything at once
say ""
cards=$(arecord -l 2>/dev/null | sed -n 's/^card [0-9]*: \(HAudio[0-9]*\) .*/\1/p')
if [ ${#sig_nodes[@]} -gt 0 ]; then
	say "== all at once for $DUR s: ${#sig_nodes[@]} video channel(s) with signal + audio on $(echo $cards | wc -w) card(s)"
	cpu() { awk '/^cpu / { idle = $5 + $6; tot = 0; for (i = 2; i <= NF; i++) tot += $i; print tot, idle }' /proc/stat; }
	thr() { local p t=0; for p in $(pgrep '^hws-(copy|conv)'); do
			t=$((t + $(awk '{ print $14 + $15 }' /proc/$p/stat 2>/dev/null || echo 0))); done; echo $t; }
	read -r tot0 idle0 < <(cpu); thr0=$(thr)
	par_start=$(dmesg_now)
	pids=()
	for d in "${sig_nodes[@]}"; do
		n=$(awk -v f="${FPS[$d]}" -v t="$DUR" 'BEGIN { printf "%d", f * t }')
		v4l2-ctl -d "$d" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/par.$(basename "$d")" 2>&1 &
		pids+=($!)
	done
	for c in $cards; do
		arecord -q -D "hw:CARD=$c" -f S16_LE -r 48000 -c 2 -d "$DUR" /dev/null > "$TMP/aud.$c" 2>&1 &
		pids+=($!)
	done
	wait "${pids[@]}"
	read -r tot1 idle1 < <(cpu); thr1=$(thr)
	for d in "${sig_nodes[@]}"; do
		n=$(awk -v f="${FPS[$d]}" -v t="$DUR" 'BEGIN { printf "%d", f * t }')
		w=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *\([0-9]*\)\/.*/\1/p')
		h=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *[0-9]*\/\([0-9]*\).*/\1/p')
		analyze "$TMP/par.$(basename "$d")" "$n" "${FPS[$d]}" $((w * h * 2)) "$d" || bad "frames lost on $d while all channels ran"
	done
	for c in $cards; do
		if grep -qiE 'error|overrun' "$TMP/aud.$c"; then bad "$c: $(head -c 200 "$TMP/aud.$c")"
		else say "   $c: audio ok"; fi
	done
	busy=$(awk -v t=$((tot1 - tot0)) -v i=$((idle1 - idle0)) 'BEGIN { printf "%.0f", t ? 100 * (t - i) / t : 0 }')
	drv=$(awk -v d=$((thr1 - thr0)) -v hz="$(getconf CLK_TCK)" -v s="$DUR" 'BEGIN { printf "%.1f", 100 * d / hz / s }')
	say "   CPU: whole system ${busy}% busy ($(nproc) cores); the driver's copy/conversion threads used ${drv}% of one core"
	# frames the card completed vs. frames the driver took in (timing_debug=1):
	# equal = a late frame never came from the card; more card frames = lost in the driver
	if [ $dmesg_ok = 1 ]; then
		cmp=$(dmesg_since "$par_start" | awk '
			/HwsCapture ch[0-9]+: in .*frames card [0-9]+ in [0-9]+/ {
				ch = $0; sub(/.*HwsCapture /, "", ch); sub(/:.*/, "", ch)
				m = $0; sub(/.*frames card /, "", m); split(m, v, /[ |]+/)
				card[ch] += v[1]; got[ch] += v[3]; seen[ch] = 1 }
			END { for (ch in seen) printf "%s %d %d\n", ch, card[ch], got[ch] }' | sort)
		while read -r ch c g; do
			[ -n "$ch" ] || continue
			if [ $((c - g)) -gt 2 ]; then bad "$ch: the card completed $c frames, the driver took in $g: $((c - g)) lost in the driver"
			else say "   $ch: the card completed $c frames, the driver took in $g (late frames came late from the card)"; fi
		done <<< "$cmp"
	fi
else
	say "== all at once: skipped, no channel has a signal"
	for c in $cards; do
		out=$(arecord -q -D "hw:CARD=$c" -f S16_LE -r 48000 -c 2 -d 5 /dev/null 2>&1); rc=$?
		if [ $rc -ne 0 ] || echo "$out" | grep -qi overrun; then bad "$c: arecord: $out"
		else say "   $c: 5 s audio capture ok"; fi
	done
fi

# ---------------------------------------------------------------- 3. kernel log
say ""
if [ $dmesg_ok = 0 ]; then
	say "== kernel messages: not readable (run as root, or: sudo dmesg | grep -i hwscapture)"
else
	new=$(dmesg_since "$dmesg_start")
	# per-second statistics of this driver (timing_debug=1): sum the drop counters
	stats=$(printf '%s\n' "$new" | awk '
		/HwsCapture ch[0-9]+: in / { ch = $0; sub(/.*HwsCapture /, "", ch); sub(/:.*/, "", ch)
			for (i = 1; i <= NF; i++) if ($i ~ /^(dma_overrun|unpaired|stale_half|overwrite|skip|no_app_buf)$/) c[ch, $i] += $(i+1)
			seen[ch] = 1 }
		END { for (ch in seen) printf "   %s: dma_overrun %d, unpaired %d, stale_half %d, overwrite %d, skip %d, no_app_buf %d\n",
			ch, c[ch,"dma_overrun"], c[ch,"unpaired"], c[ch,"stale_half"], c[ch,"overwrite"], c[ch,"skip"], c[ch,"no_app_buf"] }' | sort)
	if [ -n "$stats" ]; then
		say "== driver drop counters over the whole run (includes v4l2-compliance's own stream tests)"
		say "$stats"
	fi
	say "== kernel messages during this run (HwsCapture, last 10)"
	printf '%s\n' "$new" | grep -i hwscapture | grep -v ': in [0-9]* fps' | tail -10 | sed 's/^/   /'
	# a crash in any module while we drove the card counts; older messages don't
	probs=$(printf '%s\n' "$new" | grep -iE 'hwscapture.*(stuck|not respond|stopped|failed)|BUG:|Oops|WARNING: CPU' | head)
	[ -n "$probs" ] && { say "$probs" | sed 's/^/   !! /'; fail=1; }
fi

say ""
say "RESULT: $([ $fail = 0 ] && echo OK || echo 'PROBLEMS FOUND (see !! above)')"
say "For frame pacing over time: tools/stutter-monitor.py --device ${nodes[0]}"
exit $fail
