#!/bin/bash
# Memory leak check for the HwsCapture driver on the real card. Repeats the
# allocation-heavy paths many times and checks that the kernel's memory comes
# back to where it started:
#
#   A. stream cycles: open, stream a few frames, close - per channel, with
#      mmap and user-pointer buffers, at the input size and scaled
#   B. audio cycles: open, capture briefly, close - per card
#   C. driver reloads: unload, load, open every channel once (only when the
#      module file is given in HWS_KO, as tools/test-on-card.sh --leak does)
#
# Measured after a warm-up round: vmalloc in use (all, the driver's own, and
# videobuf2's video buffers), unreclaimable slab memory, and kmemleak's report
# if the kernel has kmemleak. Needs root (it reads /proc/vmallocinfo); best run
# through tools/test-on-card.sh --leak, which also restores your driver.
#
#   sudo tools/leak-check.sh                    # A and B with the loaded driver
#   CYCLES=1000 sudo tools/leak-check.sh        # more stream cycles per channel (default 300)
set -u
CYCLES=${CYCLES:-300}		# stream cycles per channel
ACYCLES=${ACYCLES:-100}		# audio cycles per card
RELOADS=${RELOADS:-10}
fail=0
say() { printf '%s\n' "$*"; }
bad() { printf '  !! %s\n' "$*"; fail=1; }

[ "$(id -u)" = 0 ] || { say "run as root (sudo), it reads /proc/vmallocinfo"; exit 2; }
for t in v4l2-ctl arecord; do command -v $t >/dev/null || { say "missing $t"; exit 2; }; done
lsmod | grep -q '^HwsCapture' || { say "HwsCapture module not loaded"; exit 2; }

TMPD=$(mktemp -d /tmp/hws-leak.XXXXXX); trap 'rm -rf "$TMPD"' EXIT
KMEMLEAK=/sys/kernel/debug/kmemleak
[ -e $KMEMLEAK ] || mount -t debugfs none /sys/kernel/debug 2>/dev/null
have_kmemleak=0; [ -w $KMEMLEAK ] && have_kmemleak=1

# ---------------------------------------------------------------- measuring
# kB: vmalloc total, vmalloc of this driver, vmalloc of videobuf2 (video
# buffers), unreclaimable slab; plus the number of driver/vb2 vmalloc areas.
# "unpurged" areas are already freed, the kernel just hasn't recycled them
# yet (vm_map_ram, used for user-pointer buffers, frees lazily): not counted.
snap() {
	awk '
		/unpurged/ { next }
		{ sz = $2 / 1024; tot += sz }
		/\[HwsCapture\]/ { drv += sz; ndrv++ }
		/\[videobuf2_vmalloc\]|vb2_vmalloc/ { vb2 += sz; nvb2++ }
		/\[snd[_a-z]*\]|snd_dma_|snd_pcm/ { snd += sz; nsnd++ }
		END { printf "%d %d %d %d %d %d %d", tot, drv, vb2, ndrv, nvb2, snd, nsnd }' /proc/vmallocinfo
	printf ' %d\n' "$(awk '/^SUnreclaim:/ { print $2 }' /proc/meminfo)"
}
# vmalloc per allocating function, to name whatever grew
callers() { awk '!/unpurged/ { c = $3; sub(/\+.*/, "", c); m = $0; if (match(m, /\[[a-z0-9_]+\]/)) c = c " " substr(m, RSTART, RLENGTH); sz[c] += $2 } END { for (c in sz) print sz[c], c }' /proc/vmallocinfo | sort -k2; }
base() { read -r B_TOT B_DRV B_VB2 B_NDRV B_NVB2 B_SND B_NSND B_SLAB < <(snap); callers > "$TMPD/callers.base"; }
report() {	# <label> <operations done>
	local tot drv vb2 ndrv nvb2 snd nsnd slab
	read -r tot drv vb2 ndrv nvb2 snd nsnd slab < <(snap)
	say "   after $1: driver $((drv - B_DRV)) kB ($((ndrv - B_NDRV)) areas), video buffers $((vb2 - B_VB2)) kB ($((nvb2 - B_NVB2)) areas), ALSA $((snd - B_SND)) kB ($((nsnd - B_NSND)) areas), slab $((slab - B_SLAB)) kB; all vmalloc $((tot - B_TOT)) kB"
	[ $((nsnd - B_NSND)) -gt 0 ] && bad "$((nsnd - B_NSND)) ALSA buffers more than before: audio buffers not freed"
	# video buffers belong to open files: with everything closed they must all be gone
	[ $((nvb2 - B_NVB2)) -gt 0 ] && bad "$((nvb2 - B_NVB2)) video buffers still allocated after all files were closed"
	[ $((ndrv - B_NDRV)) -ne 0 ] && bad "the driver's own vmalloc areas changed by $((ndrv - B_NDRV))"
	# slab moves with everything else on the system: only clear growth per operation counts
	local per=$(( ($2 > 0) ? (slab - B_SLAB) * 1024 / $2 : 0 ))
	[ $((slab - B_SLAB)) -gt 4096 ] && [ $per -gt 256 ] && bad "slab grew by $((slab - B_SLAB)) kB (~$per bytes per operation): possible leak, repeat to confirm"
	# growth elsewhere in the kernel: name it, but it isn't this driver's
	if [ $((tot - B_TOT)) -gt 16384 ]; then
		say "   vmalloc grew outside the driver, video buffers and ALSA; largest growth by allocating function:"
		callers > "$TMPD/callers.now"
		join -1 2 -2 2 -a 2 -e 0 -o '2.2,1.1,2.1' <(awk '{ print $1, $2 $3 }' "$TMPD/callers.base" | sort -k2) <(awk '{ print $1, $2 $3 }' "$TMPD/callers.now" | sort -k2) 2>/dev/null |
			awk '{ d = ($3 - $2) / 1024; if (d > 512) printf "      %+8d kB  %s\n", d, $1 }' | sort -rn | head -5
	fi
}

# ---------------------------------------------------------------- the devices
nodes() { for d in /dev/video*; do v4l2-ctl -d "$d" --info 2>/dev/null | grep -q 'Driver name *: *HwsCapture' && echo "$d"; done; }
cards() { arecord -l 2>/dev/null | sed -n 's/^card [0-9]*: \(HAudio[0-9]*\) .*/\1/p'; }
NODES=$(nodes); CARDS=$(cards)
[ -n "$NODES" ] || { say "no HwsCapture video devices"; exit 2; }
declare -A FMT
for d in $NODES; do
	FMT[$d]=$(v4l2-ctl -d "$d" --get-fmt-video | sed -n 's/.*Width\/Height *: *\([0-9]*\)\/\([0-9]*\).*/width=\1,height=\2,pixelformat=YUYV/p')
done
stream_round() {	# <cycle number>: one open/stream/close per channel, varied
	local d i=$1 io size
	for d in $NODES; do
		io="--stream-mmap=4"; [ $((i % 2)) = 1 ] && io="--stream-user=4"
		size="${FMT[$d]}"; [ $((i % 3)) = 2 ] && size="width=1280,height=720,pixelformat=YUYV"
		v4l2-ctl -d "$d" --set-fmt-video="$size" $io --stream-count=3 --stream-to=/dev/null >/dev/null 2>&1
	done
}
audio_round() { local c; for c in $CARDS; do arecord -q -D "hw:CARD=$c" -f S16_LE -r 48000 -c 2 -s 4800 /dev/null >/dev/null 2>&1; done; }
restore_fmt() { local d; for d in $NODES; do v4l2-ctl -d "$d" --set-fmt-video="${FMT[$d]}" >/dev/null 2>&1; done; }

say "== leak check: $(echo $NODES | wc -w) video channel(s), $(echo $CARDS | wc -w) audio card(s); kmemleak: $([ $have_kmemleak = 1 ] && echo yes || echo 'not in this kernel (CONFIG_DEBUG_KMEMLEAK)')"
[ $have_kmemleak = 1 ] && echo clear > $KMEMLEAK

# warm-up: first-use allocations (caches, lazily created structures) aren't leaks
for i in 0 1 2; do stream_round $i; done; audio_round; restore_fmt
sleep 1; base
say "   baseline: driver $B_DRV kB in $B_NDRV areas, video buffers $B_VB2 kB, ALSA $B_SND kB in $B_NSND areas, slab $B_SLAB kB; all vmalloc $B_TOT kB"

# ---------------------------------------------------------------- A. streams
say "== A. $CYCLES stream cycles per channel (open, 3 frames, close; mmap/userptr; input size/scaled)"
t0=$(date +%s)
for i in $(seq 1 "$CYCLES"); do stream_round "$i"; done
restore_fmt; sleep 1
report "$CYCLES stream cycles ($(( $(date +%s) - t0 )) s)" $((CYCLES * $(echo $NODES | wc -w)))

# ---------------------------------------------------------------- B. audio
if [ -n "$CARDS" ]; then
	say "== B. $ACYCLES audio cycles per card (open, 100 ms, close)"
	for i in $(seq 1 "$ACYCLES"); do audio_round; done
	sleep 1
	report "$ACYCLES audio cycles" $((CYCLES * $(echo $NODES | wc -w) + ACYCLES * $(echo $CARDS | wc -w)))
fi

# ---------------------------------------------------------------- C. reloads
if [ -n "${HWS_KO:-}" ] && [ -f "$HWS_KO" ]; then
	say "== C. $RELOADS driver reloads (unload, load, open every channel once)"
	unload() { local n; for n in $(seq 1 20); do rmmod HwsCapture 2>/dev/null && return 0; sleep 0.5; done; return 1; }
	# compare with the driver unloaded: everything it ever allocated must be gone
	unload || { bad "can't unload the driver"; RELOADS=0; }
	sleep 1; read -r u_tot _ _ _ _ _ _ u_slab < <(snap)
	for i in $(seq 1 "$RELOADS"); do
		insmod "$HWS_KO" timing_debug=0 || { bad "reload $i: insmod failed"; break; }
		udevadm settle 2>/dev/null; sleep 1
		for d in $(nodes); do v4l2-ctl -d "$d" --stream-mmap=4 --stream-count=3 --stream-to=/dev/null >/dev/null 2>&1; done
		unload || { bad "reload $i: can't unload"; break; }
	done
	sleep 1; read -r tot _ _ _ _ _ _ slab < <(snap)
	say "   unloaded, after $RELOADS reloads: vmalloc $((tot - u_tot)) kB, slab $((slab - u_slab)) kB (per reload: $(( (tot - u_tot) / (RELOADS > 0 ? RELOADS : 1) )) kB vmalloc, $(( (slab - u_slab) / (RELOADS > 0 ? RELOADS : 1) )) kB slab)"
	# one load allocates ~64 MB of ring buffers alone: a leak of anything sizable shows per reload
	[ $(( (tot - u_tot) / (RELOADS > 0 ? RELOADS : 1) )) -gt 1024 ] && bad "vmalloc grows with every reload"
	[ $(( (slab - u_slab) / (RELOADS > 0 ? RELOADS : 1) )) -gt 256 ] && bad "slab grows with every reload (repeat to confirm)"
	insmod "$HWS_KO" timing_debug=1 2>/dev/null; udevadm settle 2>/dev/null
else
	say "== C. driver reloads: skipped (run through tools/test-on-card.sh --leak to include them)"
fi

# ---------------------------------------------------------------- kmemleak
if [ $have_kmemleak = 1 ]; then
	echo scan > $KMEMLEAK; sleep 5; echo scan > $KMEMLEAK
	leaks=$(grep -c '^unreferenced object' $KMEMLEAK 2>/dev/null)
	ours=$(awk '/^unreferenced object/ { o = $0 } /HwsCapture|hws_/ { if (o) { print o; o = "" } }' $KMEMLEAK | wc -l)
	say "== kmemleak: $leaks unreferenced object(s) on the whole system, $ours with this driver in the backtrace"
	[ "$ours" -gt 0 ] && { bad "kmemleak reports leaks from the driver:"; awk '/^unreferenced object/,/^$/' $KMEMLEAK | grep -B2 -A14 -iE 'HwsCapture|hws_' | head -60 | sed 's/^/      /'; }
fi

say ""
say "LEAK CHECK: $([ $fail = 0 ] && echo 'OK, no leak found' || echo 'PROBLEMS FOUND (see !! above)')"
exit $fail
