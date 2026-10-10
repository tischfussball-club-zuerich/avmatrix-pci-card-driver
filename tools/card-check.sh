#!/bin/bash
# Check the HwsCapture driver on the real card. Connect a source to the inputs
# you want checked; channels without a signal are reported as such.
#
#   1. per channel: V4L2 conformance (v4l2-compliance), a capture at the input
#      size (direct path) and one at a different, scaled size (ring + conversion)
#   2. all channels with a signal at once, plus audio on every card at the same
#      time - the real streaming load - with CPU use and the driver's threads
#   3. more behaviour (EXTRA=0 skips it): picture controls reach the picture,
#      the scaled picture matches the input, user-pointer buffers, two apps on
#      one channel, capture under full CPU load, apps killed at random moments
#      (no hang, no crash, the channel keeps working), the audio sample clock
#   4. the driver's own drop counters and problem messages from the kernel log
#
#   tools/card-check.sh                  # 10 s captures
#   DUR=60 tools/card-check.sh           # longer captures (each phase)
#   SKIP_COMPLIANCE=1 tools/card-check.sh   # e.g. to compare with another driver
#   RESULTS=file tools/card-check.sh     # also write the numbers to a file
#                                        # (tools/test-on-card.sh --compare uses it)
#
# v4l2-compliance changes format and picture controls while it tests, the scaled
# capture changes the format; the script puts them back afterwards.
# Needs v4l-utils (v4l2-ctl, v4l2-compliance) and alsa-utils (arecord).
set -u
DUR=${DUR:-10}
SCALE_DUR=${SCALE_DUR:-$(( DUR < 10 ? DUR : 10 ))}
SKIP_COMPLIANCE=${SKIP_COMPLIANCE:-0}
export LC_ALL=C	# awk/printf with a decimal point, whatever the desktop's language
EXTRA=${EXTRA:-1}
CHAOS=${CHAOS:-30}	# kill-test rounds
ADUR=${ADUR:-20}	# audio clock measurement, seconds
TMP=$(mktemp -d /tmp/hws-check.XXXXXX)
LOAD_PIDS=()
stop_load() { [ ${#LOAD_PIDS[@]} -eq 0 ] || { kill "${LOAD_PIDS[@]}"; wait "${LOAD_PIDS[@]}"; } 2>/dev/null; LOAD_PIDS=(); }
# background captures ignore Ctrl-C (non-interactive shell): end them too, or they
# keep the device busy and the driver can't be unloaded afterwards. Their children
# as well: timeout runs in its own process group, out of reach of the job's signal
end_jobs() { local p; for p in $(jobs -p); do pkill -TERM -P "$p"; kill "$p"; done 2>/dev/null; }
trap 'stop_load; end_jobs; rm -rf "$TMP"' EXIT
# kernel messages after a point in time. Timestamps, not line counts: once the
# kernel's log buffer is full, old lines drop out as new ones arrive.
dmesg_now() { dmesg 2>/dev/null | tail -1 | sed -n 's/^\[ *\([0-9.]*\)\].*/\1/p'; }
dmesg_since() { dmesg 2>/dev/null | awk -v t="${1:-0}" 'match($0, /^\[ *[0-9.]+\]/) { v = substr($0, RSTART + 1, RLENGTH - 2); gsub(/ /, "", v); if (v + 0 > t + 0) print }'; }
# only kernel messages from this run count
dmesg_ok=1; dmesg >/dev/null 2>&1 || dmesg_ok=0; dmesg_start=$(dmesg_now)
fail=0
say() { printf '%s\n' "$*"; }
bad() { printf '  !! %s\n' "$*"; fail=1; }
# machine-readable results: "<key> name=value ..." per line
RESULTS=${RESULTS:-}
res() { [ -z "$RESULTS" ] || printf '%s\n' "$*" >> "$RESULTS"; }
# a capture that hangs (e.g. a driver that stops delivering) must not hang the check
cap_limit() { echo $(( $1 * 2 + 15 )); }

# analyze <v4l2-ctl --verbose output> <frames wanted> <fps> <expected bytesused> <label> [<results key>]
# sequence gaps = frames the app lost, timestamp gaps = pacing
analyze() {
	awk -v want="$2" -v fps="$3" -v size="$4" -v label="$5" -v key="${6:-}" -v res="$RESULTS" '
		/seq:/ { for (i = 1; i <= NF; i++) {
				if ($i == "seq:") s = $(i+1)
				if ($i == "ts:") t = $(i+1)
				if ($i == "bytesused:") b = $(i+1) }
			 # a sequence number that does not increase: a duplicate or reordered frame
			 if (cnt++) { if (s > ps + 1) lost += s - ps - 1; else if (s <= ps) dup++ }
			 if (cnt > 1) { dt = (t - pt) * 1000; if (dt > max) max = dt; if (dt > 1500 / fps) late++ }
			 if (size && b != size) wrong++
			 if (cnt == 1) t0 = t
			 ps = s; pt = t }
		END { rate = (pt > t0) ? (cnt - 1 + late) / (pt - t0) : 0
		      printf "   %s: %d of %d frames, %d lost, %d late (>1.5 frame periods), max gap %.1f ms%s%s%s\n",
			  label, cnt, want, lost, late, max, dup ? sprintf(", %d duplicate/out of order", dup) : "",
			  wrong ? sprintf(", %d with wrong size", wrong) : "",
			  rate ? sprintf(", %.3f fps from the timestamps", rate) : ""
		      if (res != "" && key != "")
			  printf "%s frames=%d want=%d lost=%d late=%d maxgap=%.1f dup=%d wrong=%d fps=%.3f\n",
				 key, cnt, want, lost, late, max, dup, wrong, rate >> res
		      if (cnt < want * 0.95 || lost > 0 || dup > 0 || wrong) exit 1 }' "$1"
}

# grid <YUYV frame> <width> <height> [x0 y0 w h]: average Y, U, V of an 8 x 6 grid
# of cells over the frame (or a region of it), one cell per line. Every 4th row
# and every 4th pixel pair: plenty for averages, and fast enough in awk.
grid() {
	od -An -tu1 -v -w$(( $2 * 2 )) "$1" | awk -v W="$2" -v H="$3" -v X0="${4:-0}" -v Y0="${5:-0}" -v CW="${6:-$2}" -v CH="${7:-$3}" '
		(NR - 1) % 4 { next }
		{ r = NR - 1; if (r < Y0 || r >= Y0 + CH) next
		  gy = int((r - Y0) * 6 / CH)
		  for (x = X0; x < X0 + CW; x += 8) {
			i = x * 2 + 1; gx = int((x - X0) * 8 / CW); c = gy * 8 + gx
			y[c] += $i + $(i + 2); u[c] += $(i + 1); v[c] += $(i + 3); n[c]++ } }
		END { for (c = 0; c < 48; c++) if (n[c]) printf "%.2f %.2f %.2f\n", y[c] / n[c] / 2, u[c] / n[c], v[c] / n[c]; else print "0 0 0" }'
}
# corr <grid a> <grid b> <column 1-3>: correlation of the cells, and the spread
# (standard deviation) of grid a, so a uniform picture can be told apart
corr() {
	paste -d' ' "$1" "$2" | awk -v c="$3" '
		{ a = $c; b = $(c + 3); n++; sa += a; sb += b; saa += a * a; sbb += b * b; sab += a * b }
		END { va = saa / n - (sa / n) ^ 2; vb = sbb / n - (sb / n) ^ 2
		      r = 0; if (va > 0 && vb > 0) r = (sab / n - sa / n * sb / n) / sqrt(va * vb)
		      sd = 0; if (va > 0) sd = sqrt(va)
		      printf "%.3f %.1f\n", r, sd }'
}
# grab <device> <file> [<fmt>]: one frame, after the pipeline has settled
grab() {
	rm -f "$2"
	timeout -s INT 20 v4l2-ctl -d "$1" ${3:+--set-fmt-video=$3} --stream-mmap=4 --stream-skip=10 --stream-count=1 --stream-to="$2" >/dev/null 2>&1
}
# card name of channel N (HAudioN), if there is one
audio_of() { local c; for c in $cards; do [ "$c" = "HAudio$1" ] && echo "$c"; done; }

for t in v4l2-ctl arecord; do
	command -v $t >/dev/null || { say "missing $t (install v4l-utils / alsa-utils)"; exit 2; }
done
[ "$SKIP_COMPLIANCE" = 1 ] || command -v v4l2-compliance >/dev/null || { say "missing v4l2-compliance (install v4l-utils)"; exit 2; }
lsmod | grep -q '^HwsCapture' || { say "HwsCapture module not loaded"; exit 2; }
say "== module: $(modinfo -F version HwsCapture 2>/dev/null) params: direct_io=$(cat /sys/module/HwsCapture/parameters/direct_io 2>/dev/null) nt_copy=$(cat /sys/module/HwsCapture/parameters/nt_copy 2>/dev/null) timing_debug=$(cat /sys/module/HwsCapture/parameters/timing_debug 2>/dev/null)"

nodes=()
# by number: video10 after video9, so ch1, ch2... follow the card's channels
for d in $(printf '%s\n' /dev/video* | sort -V); do
	v4l2-ctl -d "$d" --info 2>/dev/null | grep -q 'Driver name *: *HwsCapture' && nodes+=("$d")
done
[ ${#nodes[@]} -gt 0 ] || { say "no HwsCapture video devices found"; exit 2; }
say "== ${#nodes[@]} video channel(s): ${nodes[*]}"
cards=$(arecord -l 2>/dev/null | sed -n 's/^card [0-9]*: \(HAudio[0-9]*\) .*/\1/p')
# the driver's counters per channel (this driver; the stock one has no stats file)
stats_snap() {
	local d f
	for d in "${nodes[@]}"; do
		f=/sys/class/video4linux/$(basename "$d")/stats
		[ -r "$f" ] && printf '%s %s\n' "$d" "$(awk '{ printf "%s=%s ", $1, $2 }' "$f")"
	done
}
stats_start=$(stats_snap)
# the check assumes the card counts no frames while nobody captures: look for a second
idle_moving=
if [ -n "$stats_start" ]; then
	sleep 1
	[ "$(stats_snap | sed 's/ capture_starts=.*//; s/.*card_frames=\([0-9]*\).*/\1/')" = \
	  "$(printf '%s\n' "$stats_start" | sed 's/ capture_starts=.*//; s/.*card_frames=\([0-9]*\).*/\1/')" ] || idle_moving=1
fi

# ---------------------------------------------------------------- 1. per channel
sig_nodes=()
declare -A FPS KEY
ci=0
for d in "${nodes[@]}"; do
	# results by channel number: the device names may differ between drivers
	ci=$((ci + 1)); KEY[$d]=ch$ci
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
		# "Total for HwsCapture device /dev/video0: 55, Succeeded: 54, Failed: 1, Warnings: 0"
		res "${KEY[$d]}.compliance $(printf '%s' "$sum" | sed -n 's/.*: \([0-9]*\), Succeeded: \([0-9]*\), Failed: \([0-9]*\), Warnings: \([0-9]*\).*/tests=\1 ok=\2 failed=\3 warnings=\4/p')"
		printf '%s\n' "$comp" | grep -E 'fail:' | sed 's/^/      /' | head -12
	fi

	res "${KEY[$d]}.info fmt=${w}x${h} fps=$fps signal=$([ "$nosig" -gt 0 ] && echo 0 || echo 1)"
	[ "$nosig" -gt 0 ] && { say "   captures skipped: no signal"; continue; }
	sig_nodes+=("$d")

	# input size: straight from DMA into the app's buffer
	n=$(awk -v f="$fps" -v t="$DUR" 'BEGIN { printf "%d", f * t }')
	timeout -s INT $(cap_limit $DUR) v4l2-ctl -d "$d" --set-fmt-video="$saved_fmt" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/seq" 2>&1
	analyze "$TMP/seq" "$n" "$fps" $((w * h * 2)) "capture ${w}x${h}, ${DUR} s" "${KEY[$d]}.input" || bad "frames lost or capture incomplete on $d"
	# measured by the driver during that capture (59.94 vs 60), what apps get from G_PARM
	rfps=$(v4l2-ctl -d "$d" --get-parm | sed -n 's/.*Frames per second: *//p')
	say "   frame rate reported to applications now: $rfps"
	res "${KEY[$d]}.rate reported=$(printf '%s' "${rfps:-?}" | sed 's/ .*//')"

	# another size: through the ring and the conversion thread
	if [ "$w" = 1920 ]; then sw=1280; sh=720; else sw=1920; sh=1080; fi
	n=$(awk -v f="$fps" -v t="$SCALE_DUR" 'BEGIN { printf "%d", f * t }')
	timeout -s INT $(cap_limit $SCALE_DUR) v4l2-ctl -d "$d" --set-fmt-video="width=$sw,height=$sh,pixelformat=YUYV" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/scaled" 2>&1
	analyze "$TMP/scaled" "$n" "$fps" $((sw * sh * 2)) "scaled to ${sw}x${sh}, ${SCALE_DUR} s" "${KEY[$d]}.scaled" || bad "scaled capture failed on $d"
	# one scaled frame must look like a picture, not a blank or garbage buffer
	rm -f "$TMP/frame.yuv"
	timeout -s INT 20 v4l2-ctl -d "$d" --set-fmt-video="width=$sw,height=$sh,pixelformat=YUYV" --stream-mmap=4 --stream-skip=10 --stream-count=1 --stream-to="$TMP/frame.yuv" >/dev/null 2>&1
	size=$(stat -c %s "$TMP/frame.yuv" 2>/dev/null || echo 0)
	vals=$(head -c 2000000 "$TMP/frame.yuv" 2>/dev/null | od -An -tu1 -v | tr -s ' ' '\n' | sort -un | wc -l)
	if [ "$size" -ne $((sw * sh * 2)) ]; then bad "scaled frame has $size bytes, expected $((sw * sh * 2))"
	elif [ "$vals" -lt 16 ]; then say "   scaled frame: only $vals different byte values (blank picture? check the source)"
	else say "   scaled frame: $size bytes, $vals different byte values"; fi
	res "${KEY[$d]}.picture values=$vals"

	if [ "$EXTRA" = 1 ]; then
		# the scaled picture must show the input: compare the brightness and colour
		# of an 8 x 6 grid (the part of the input the scaler shows: the centre,
		# cropped to the output's aspect ratio). A moving source lowers the match
		# a little, a broken scaler (squeezed, shifted, U/V swapped) a lot.
		if [ "$size" -eq $((sw * sh * 2)) ] && grab "$d" "$TMP/in.yuv" "$saved_fmt" &&
		   [ "$(stat -c %s "$TMP/in.yuv" 2>/dev/null || echo 0)" -eq $((w * h * 2)) ]; then
			read -r cx cy cw ch < <(awk -v w=$w -v h=$h -v sw=$sw -v sh=$sh 'BEGIN {
				if (w * sh > h * sw) { cw = int(h * sw / sh); ch = h } else { cw = w; ch = int(w * sh / sw) }
				printf "%d %d %d %d", int((w - cw) / 4) * 2, int((h - ch) / 2), cw, ch }')
			grid "$TMP/in.yuv" $w $h $cx $cy $cw $ch > "$TMP/g.in"
			grid "$TMP/frame.yuv" $sw $sh > "$TMP/g.sc"
			read -r cy_ sy < <(corr "$TMP/g.in" "$TMP/g.sc" 1)
			read -r cu su < <(corr "$TMP/g.in" "$TMP/g.sc" 2)
			read -r cv sv < <(corr "$TMP/g.in" "$TMP/g.sc" 3)
			say "   scaled vs. input picture: match brightness $cy_, colour U $cu V $cv (1 = same; spread of the input: $sy / $su / $sv)"
			res "${KEY[$d]}.scalecheck luma=$cy_ u=$cu v=$cv"
			awk -v c=$cy_ -v s=$sy 'BEGIN { exit !(s >= 4 && c < 0.5) }' && bad "the scaled picture doesn't match the input on $d (brightness match $cy_)"
			awk -v c=$cy_ -v s=$sy 'BEGIN { exit !(s >= 4 && c >= 0.5 && c < 0.95) }' &&
				say "   note: brightness match below 0.95: a moving picture, or a scaler that squeezes or shifts it"
			awk -v c=$cy_ -v s=$sy 'BEGIN { exit !(s < 4) }' && say "   (the input is nearly uniform: show something with contrast for a meaningful scaler check)"
			for x in "U $cu $su" "V $cv $sv"; do read -r nm c sp <<< "$x"
				awk -v c=$c -v s=$sp 'BEGIN { exit !(s >= 3 && c < 0.5) }' && bad "the scaled picture's colour ($nm) doesn't match the input on $d (match $c)"
			done
		else
			say "   scaled vs. input picture: could not grab both frames"
		fi

		# picture controls must change the picture (in the original driver they
		# never reached the card): brightness at its minimum and its maximum
		bline=$(v4l2-ctl -d "$d" --list-ctrls 2>/dev/null | grep -E '^ *brightness ')
		bmin=$(printf '%s' "$bline" | sed -n 's/.*min=\(-*[0-9]*\).*/\1/p')
		bmax=$(printf '%s' "$bline" | sed -n 's/.*max=\(-*[0-9]*\).*/\1/p')
		bval=$(printf '%s' "$bline" | sed -n 's/.*value=\(-*[0-9]*\).*/\1/p')
		if [ -n "$bmin" ] && [ -n "$bmax" ] && [ -n "$bval" ]; then
			ylo=-1; yhi=-1
			v4l2-ctl -d "$d" --set-ctrl=brightness=$bmin 2>/dev/null && grab "$d" "$TMP/b.yuv" "$saved_fmt" &&
				ylo=$(grid "$TMP/b.yuv" $w $h | awk '{ s += $1 } END { printf "%.1f", s / NR }')
			v4l2-ctl -d "$d" --set-ctrl=brightness=$bmax 2>/dev/null && grab "$d" "$TMP/b.yuv" "$saved_fmt" &&
				yhi=$(grid "$TMP/b.yuv" $w $h | awk '{ s += $1 } END { printf "%.1f", s / NR }')
			v4l2-ctl -d "$d" --set-ctrl=brightness=$bval 2>/dev/null
			bd=$(awk -v a=$ylo -v b=$yhi 'BEGIN { printf "%.1f", (a < 0 || b < 0) ? -1 : b - a }')
			say "   brightness control: average brightness $ylo at the minimum, $yhi at the maximum"
			res "${KEY[$d]}.controls low=$ylo high=$yhi diff=$bd"
			awk -v x=$bd 'BEGIN { exit !(x < 8) }' && bad "the brightness control doesn't change the picture on $d"
		else
			say "   brightness control: not found"
			res "${KEY[$d]}.controls diff=-1"
		fi

		# user-pointer buffers (the app's own memory instead of the driver's)
		n=$(awk -v f="$fps" -v t="$SCALE_DUR" 'BEGIN { printf "%d", f * t }')
		timeout -s INT $(cap_limit $SCALE_DUR) v4l2-ctl -d "$d" --set-fmt-video="$saved_fmt" --stream-user=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/user" 2>&1
		analyze "$TMP/user" "$n" "$fps" $((w * h * 2)) "user-pointer buffers, ${SCALE_DUR} s" "${KEY[$d]}.userptr" || bad "capture with user-pointer buffers failed on $d"
	fi
	v4l2-ctl -d "$d" --set-fmt-video="$saved_fmt" 2>/dev/null
done

# ---------------------------------------------------------------- 2. everything at once
say ""
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
		timeout -s INT $(cap_limit $DUR) v4l2-ctl -d "$d" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/par.$(basename "$d")" 2>&1 &
		pids+=($!)
	done
	for c in $cards; do
		{ timeout -s INT $(cap_limit $DUR) arecord -q -D "hw:CARD=$c" -f S16_LE -r 48000 -c 2 -d "$DUR" /dev/null > "$TMP/aud.$c" 2>&1 ||
			echo "error: arecord failed or hung (exit $?)" >> "$TMP/aud.$c"; } &
		pids+=($!)
	done
	wait "${pids[@]}"
	read -r tot1 idle1 < <(cpu); thr1=$(thr)
	for d in "${sig_nodes[@]}"; do
		n=$(awk -v f="${FPS[$d]}" -v t="$DUR" 'BEGIN { printf "%d", f * t }')
		w=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *\([0-9]*\)\/.*/\1/p')
		h=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *[0-9]*\/\([0-9]*\).*/\1/p')
		analyze "$TMP/par.$(basename "$d")" "$n" "${FPS[$d]}" $((w * h * 2)) "$d" "${KEY[$d]}.together" || bad "frames lost on $d while all channels ran"
	done
	for c in $cards; do
		if grep -qiE 'error|overrun' "$TMP/aud.$c"; then bad "$c: $(head -c 200 "$TMP/aud.$c")"; res "audio.$c ok=0"
		else say "   $c: audio ok"; res "audio.$c ok=1"; fi
	done
	busy=$(awk -v t=$((tot1 - tot0)) -v i=$((idle1 - idle0)) 'BEGIN { printf "%.0f", t ? 100 * (t - i) / t : 0 }')
	drv=$(awk -v d=$((thr1 - thr0)) -v hz="$(getconf CLK_TCK)" -v s="$DUR" 'BEGIN { printf "%.1f", 100 * d / hz / s }')
	say "   CPU: whole system ${busy}% busy ($(nproc) cores); the driver's copy/conversion threads used ${drv}% of one core"
	res "cpu busy=$busy"
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
		out=$(timeout -s INT 30 arecord -q -D "hw:CARD=$c" -f S16_LE -r 48000 -c 2 -d 5 /dev/null 2>&1); rc=$?
		if [ $rc -ne 0 ] || echo "$out" | grep -qi overrun; then bad "$c: arecord: $out (exit $rc)"; res "audio.$c ok=0"
		else say "   $c: 5 s audio capture ok"; res "audio.$c ok=1"; fi
	done
fi

# ---------------------------------------------------------------- 3. more behaviour
# frames wanted for <seconds> on <device>
alive() { local st; st=$(awk '{ print $3 }' "/proc/$1/stat" 2>/dev/null); [ -n "$st" ] && [ "$st" != Z ]; }
want_for() { awk -v f="${FPS[$1]}" -v t="$2" 'BEGIN { printf "%d", f * t }'; }
if [ "$EXTRA" = 1 ] && [ ${#sig_nodes[@]} -gt 0 ]; then
	d=${sig_nodes[0]}
	dn=$(basename "$d")
	w=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *\([0-9]*\)\/.*/\1/p')
	h=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *[0-9]*\/\([0-9]*\).*/\1/p')

	# two applications on the same channel: each must get every frame
	say ""
	say "== two applications on $d at once, $SCALE_DUR s"
	n=$(want_for "$d" "$SCALE_DUR")
	timeout -s INT $(cap_limit $SCALE_DUR) v4l2-ctl -d "$d" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/app1" 2>&1 &
	p1=$!
	timeout -s INT $(cap_limit $SCALE_DUR) v4l2-ctl -d "$d" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/app2" 2>&1 &
	p2=$!
	wait $p1 $p2
	analyze "$TMP/app1" "$n" "${FPS[$d]}" $((w * h * 2)) "application 1" "${KEY[$d]}.app1" || bad "application 1 of 2 lost frames on $d"
	analyze "$TMP/app2" "$n" "${FPS[$d]}" $((w * h * 2)) "application 2" "${KEY[$d]}.app2" || bad "application 2 of 2 lost frames on $d"

	# every channel while all CPUs are busy (half pure computing, half memory
	# traffic): the driver's real-time copy threads must still get every frame
	say ""
	say "== all channels with a signal while every CPU is busy, $SCALE_DUR s"
	for i in $(seq 1 "$(nproc)"); do
		if [ $((i % 2)) = 1 ]; then yes > /dev/null & else dd if=/dev/zero of=/dev/null bs=16M 2>/dev/null & fi
		LOAD_PIDS+=($!)
	done
	sleep 1
	pids=()
	for x in "${sig_nodes[@]}"; do
		n=$(want_for "$x" "$SCALE_DUR")
		timeout -s INT $(cap_limit $SCALE_DUR) v4l2-ctl -d "$x" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/load.$(basename "$x")" 2>&1 &
		pids+=($!)
	done
	wait "${pids[@]}"
	stop_load
	for x in "${sig_nodes[@]}"; do
		n=$(want_for "$x" "$SCALE_DUR")
		xw=$(v4l2-ctl -d "$x" --get-fmt-video | sed -n 's/.*Width\/Height *: *\([0-9]*\)\/.*/\1/p')
		xh=$(v4l2-ctl -d "$x" --get-fmt-video | sed -n 's/.*Width\/Height *: *[0-9]*\/\([0-9]*\).*/\1/p')
		analyze "$TMP/load.$(basename "$x")" "$n" "${FPS[$x]}" $((xw * xh * 2)) "$x" "${KEY[$x]}.loaded" || bad "frames lost on $x under CPU load"
	done

	# applications killed at random moments (while starting, streaming, two at
	# once, with audio): no hang, no crash, and the channel keeps working.
	# Another channel streams all the while and must not notice.
	say ""
	obs=${sig_nodes[1]:-}
	say "== kill test: $CHAOS rounds of applications on $d killed at random moments${obs:+ (meanwhile $obs streams)}"
	ac=$(audio_of "${KEY[$d]#ch}")
	if [ -n "$obs" ]; then
		timeout -s INT $((CHAOS * 3 + 60)) v4l2-ctl -d "$obs" --stream-mmap=4 --stream-to=/dev/null --verbose > "$TMP/obs" 2>&1 &
		opid=$!
		ot0=$(date +%s.%N)
	fi
	hung=0; kills=0
	# (stderr off: bash reports every killed job; bad() writes to stdout)
	for i in $(seq 1 "$CHAOS"); do
		vp=()
		mode=$((RANDOM % 4))
		v4l2-ctl -d "$d" $([ $mode = 1 ] && echo --stream-user=4 || echo --stream-mmap=4) --stream-to=/dev/null > /dev/null 2>&1 &
		vp+=($!)
		[ $mode -ge 2 ] && { v4l2-ctl -d "$d" --stream-mmap=2 --stream-to=/dev/null > /dev/null 2>&1 & vp+=($!); }
		[ $mode = 3 ] && [ -n "$ac" ] && { arecord -q -D "hw:CARD=$ac" -f S16_LE -r 48000 -c 2 /dev/null > /dev/null 2>&1 & vp+=($!); }
		# 0 - 1 s: sometimes while the app is still opening and setting up
		sleep "0.$(printf '%03d' $((RANDOM % 1000)))"
		for p in "${vp[@]}"; do
			kill -9 "$p" 2>/dev/null; kills=$((kills + 1))
			sleep "0.0$((RANDOM % 10))"
		done
		# a killed process that doesn't go away is stuck in the driver (one that
		# has exited is a zombie until it's reaped: that counts as gone)
		for p in "${vp[@]}"; do
			for _ in $(seq 1 50); do alive "$p" || break; sleep 0.1; done
			if alive "$p"; then
				hung=$((hung + 1))
				bad "a killed application (pid $p) is still there after 5 s: $(cat /proc/$p/wchan 2>/dev/null) $(awk '{ print "state " $3 }' /proc/$p/stat 2>/dev/null)"
			fi
			# a stuck one would block wait forever: leave it to the report above
			alive "$p" && disown "$p" || wait "$p"
		done
	done 2>/dev/null
	say "   $kills applications killed, $hung stuck"
	res "chaos kills=$kills hung=$hung"
	n=$(want_for "$d" 2)
	timeout -s INT $(cap_limit 2) v4l2-ctl -d "$d" --stream-mmap=4 --stream-count="$n" --stream-to=/dev/null --verbose > "$TMP/after" 2>&1
	analyze "$TMP/after" "$n" "${FPS[$d]}" $((w * h * 2)) "$d afterwards, 2 s" "${KEY[$d]}.after" || bad "$d doesn't work properly after the kill test"
	if [ -n "$obs" ]; then
		kill -INT "$opid" 2>/dev/null; wait "$opid" 2>/dev/null
		# frames for the time it ran, less a second for its start
		n=$(awk -v f="${FPS[$obs]}" -v a="$ot0" -v b="$(date +%s.%N)" 'BEGIN { printf "%d", f * (b - a - 1) }')
		ow=$(v4l2-ctl -d "$obs" --get-fmt-video | sed -n 's/.*Width\/Height *: *\([0-9]*\)\/.*/\1/p')
		oh=$(v4l2-ctl -d "$obs" --get-fmt-video | sed -n 's/.*Width\/Height *: *[0-9]*\/\([0-9]*\).*/\1/p')
		analyze "$TMP/obs" "$n" "${FPS[$obs]}" $((ow * oh * 2)) "$obs during the kill test" "${KEY[$obs]}.observer" || bad "$obs lost frames while applications on $d were killed"
	fi
fi

# the audio sample clock of every card (also without a signal, where the
# driver supplies the silence): $ADUR s of samples must take $ADUR s
if [ "$EXTRA" = 1 ] && [ -n "$cards" ]; then
	say ""
	say "== audio clock: $ADUR s from every card at once"
	pids=()
	for c in $cards; do
		( t0=$(date +%s%N)
		  timeout -s INT $((ADUR * 2 + 15)) arecord -q -D "hw:CARD=$c" -f S16_LE -r 48000 -c 2 -t raw -d "$ADUR" "$TMP/a.$c.raw" > "$TMP/a.$c.log" 2>&1
		  echo "$? $(( $(date +%s%N) - t0 ))" > "$TMP/a.$c.t" ) &
		pids+=($!)
	done
	wait "${pids[@]}"
	for c in $cards; do
		read -r rc ns < "$TMP/a.$c.t"
		bytes=$(stat -c %s "$TMP/a.$c.raw" 2>/dev/null || echo 0)
		# samples per second of wall time (the start of arecord costs a few ms)
		rate=$(awk -v b=$bytes -v ns=$ns 'BEGIN { r = 0; if (ns > 0) r = b / 4 / (ns / 1e9); printf "%.0f", r }')
		peak=$(head -c 1920000 "$TMP/a.$c.raw" 2>/dev/null | od -An -td2 -v | awk '{ for (i = 1; i <= NF; i++) { v = $i < 0 ? -$i : $i; if (v > m) m = v } } END { print m + 0 }')
		say "   $c: $bytes bytes in $(awk -v ns=$ns 'BEGIN { printf "%.2f", ns / 1e9 }') s = $rate samples/s, loudest sample $peak$([ "$peak" = 0 ] && echo ' (silence)')"
		res "aclock.$c rate=$rate peak=$peak"
		if [ "$rc" != 0 ] || [ "$bytes" -lt $((ADUR * 48000 * 4)) ]; then bad "$c: audio capture incomplete (arecord exit $rc, $bytes bytes): $(head -c 200 "$TMP/a.$c.log")"
		elif [ "$rate" -lt 47520 ] || [ "$rate" -gt 48480 ]; then bad "$c: audio runs at $rate samples/s instead of 48000"; fi
	done
fi

# ---------------------------------------------------------------- frame accounting
# Every frame the card completed must be taken in or counted as dropped, once.
if [ -n "$stats_start" ] && [ -n "$idle_moving" ]; then
	say ""
	say "== driver frame accounting: skipped, the card counts frames while nothing captures"
	res "accounting idle_moving=1"
elif [ -n "$stats_start" ]; then
	say ""
	say "== driver frame accounting over the whole run (card frames = taken in + dropped)"
	while read -r d line; do
		[ -n "$d" ] || continue
		before=$(printf '%s\n' "$stats_start" | awk -v d="$d" '$1 == d { $1 = ""; print }')
		[ -n "$before" ] || continue
		out=$(awk -v a="$before" -v b="$line" 'BEGIN {
			n = split(a, x, " "); for (i = 1; i <= n; i++) { split(x[i], kv, "="); p[kv[1]] = kv[2] }
			n = split(b, x, " "); for (i = 1; i <= n; i++) { split(x[i], kv, "="); c[kv[1]] = kv[2] }
			if (!("aborted" in c)) { print "old"; exit }
			card = c["card_frames"] - p["card_frames"]; if (card < 0) card += 4294967296
			split("dma_overrun stale_half unpaired aborted copy_missed", k, " ")
			drops = 0; txt = ""
			for (i = 1; i <= 5; i++) { v = c[k[i]] - p[k[i]]; drops += v; if (v) txt = txt " " k[i] " " v }
			inn = c["frames_in"] - p["frames_in"]
			printf "%d %d %d %d%s", card, inn, drops, card - inn - drops, txt }')
		[ "$out" = old ] && continue
		read -r card inn drops un rest <<< "$out"
		say "   $d: card $card, taken in $inn, dropped $drops${rest:+ (${rest# })}, unaccounted $un"
		res "accounting.${KEY[$d]} card=$card in=$inn dropped=$drops unaccounted=$un"
		if [ "$un" -gt 2 ] || [ "$un" -lt -2 ]; then bad "$d: $un frames neither taken in nor counted as dropped - a driver bug, please report"; fi
	done <<< "$(stats_snap)"
fi

# ---------------------------------------------------------------- 4. kernel log
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
	res "kernel problems=$(printf '%s' "$probs" | grep -c .)"
fi

say ""
say "RESULT: $([ $fail = 0 ] && echo OK || echo 'PROBLEMS FOUND (see !! above)')"
res "result ok=$([ $fail = 0 ] && echo 1 || echo 0)"
say "While streaming: sudo tools/stutter-monitor.py --quiet (driver + system, no extra load)"
exit $fail
