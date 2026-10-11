#!/bin/bash
# Full verification of the current driver source; every output is scanned in full.
S=$(cd "$(dirname "$0")" && pwd); SRC=$(cd "${1:?usage: verify.sh <driver src dir>}" && pwd) || exit 1; fail=0
scan() {	# file label
	local ub=$(grep -c "runtime error" "$1") as=$(grep -c "ERROR: AddressSanitizer" "$1")
	echo "    sanitizer reports: UBSan $ub, ASan $as"; [ "$ub$as" = "00" ] || fail=1
}
echo "== converters: $(python3 -I $S/conv/extract.py $SRC/hws_video.c > $S/conv/driver_conv.inc && echo extracted)"
(cd $S/conv && gcc -O1 -g -fsanitize=address,undefined -w -o h harness.c && ASAN_OPTIONS=detect_leaks=0 ./h > out.txt 2>&1); echo "   $(grep combinations $S/conv/out.txt)"; scan $S/conv/out.txt
grep -q "faults: 0" $S/conv/out.txt || fail=1
python3 -I - $SRC/hws_video.c > $S/conv/doubling.inc <<'PY'
import re, sys
s = open(sys.argv[1]).read()
for name in ("SetDeInterlace", "hws_dcopy", "hws_copy_doubled"):
    print(re.search(r"^static (?:inline )?void " + name + r"\(.*?^\}\n", s, re.S | re.M).group(0))
PY
(cd $S/conv && gcc -O1 -fsanitize=address,undefined -w -o dbl doubling.c && ./dbl > dbl.txt 2>&1); echo "== interlaced direct path: $(cat $S/conv/dbl.txt | tail -1)"; scan $S/conv/dbl.txt
grep -q "mismatches: 0" $S/conv/dbl.txt || fail=1
rm -f $S/sim/sim $S/sim/sim_tsan
if ! $S/sim/build.sh $SRC > $S/sim/build.log 2>&1; then echo "!! simulator build FAILED:"; grep -m5 error $S/sim/build.log; echo "RESULT: FAIL"; exit 1; fi
for mode in race logic nodirect irqfaults; do for f in 60 240 480; do
	o=$S/sim/out_${mode}_$f.txt
	unset SIM_DMA_LOCK SIM_NO_DIRECT SIM_LOSE_IRQ SIM_DUP_IRQ SIM_NTSC
	if [ $mode = logic ]; then export SIM_DMA_LOCK=1 SIM_NTSC=1; fi	# also: 59.94 instead of 60
	if [ $mode = nodirect ]; then export SIM_NO_DIRECT=1; fi
	if [ $mode = irqfaults ]; then export SIM_LOSE_IRQ=40 SIM_DUP_IRQ=40; fi
	ASAN_OPTIONS=detect_leaks=0 timeout 300 $S/sim/sim $f 45 > $o 2>&1; rc=$?
	echo "== sim $f fps, mode=$mode (race=card DMA race present, logic=race removed + 59.94 fps, nodirect=ring path only, irqfaults=lost+duplicate IRQs): exit $rc"
	grep -E "^frames|delivered|restarts|steady|duplicate|out of order|mod 100|sequence not|timestamp not|STREAMOFF|ERROR b|stalls|latency|driver stats|accounting" $o | sed 's/^/    /'
	for k in "torn/mixed, steady state" "duplicate frames" "frames out of order" "sequence not increasing" "timestamp not increasing" "left in driver after STREAMOFF" "completed after STREAMOFF" "2 s stalls" "watchdog re-inits"; do
		v=$(grep "$k" $o | grep -o '[0-9]\+' | tail -1); [ "${v:-1}" = 0 ] || { echo "    !! $k = $v"; fail=1; }; done
	# an app that keeps showing its frame until the next one arrives (2 buffers,
	# 10 ms per frame) must get most frames at 60 fps (it got half when the
	# driver decided at the first half-frame whether anyone had a buffer)
	if [ $f = 60 ]; then
		p=$(grep -o "frames: [0-9]*%" $o | grep -o "[0-9]*"); echo "    steady 2-buffer app got ${p:-?}% of the frames"
		[ "${p:-0}" -ge 75 ] || { echo "    !! steady 2-buffer app got only ${p:-0}% of the frames"; fail=1; }
	fi
	# the exact frame rate reported by G_PARM (1001/60000 vs 1/60)
	ok=$(grep -o "frame rate measurements: right [0-9]*" $o | grep -o "[0-9]*$"); echo "    exact frame rate measured right ${ok:-?} times"
	[ $f != 60 ] || [ "${ok:-0}" -gt 0 ] || { echo "    !! the frame rate was never measured"; fail=1; }
	# card frames = frames taken in + counted drops; at most the last frame or two still in flight
	for u in $(grep -o "unaccounted -*[0-9]*" $o | grep -o -- "-*[0-9]*$"); do
		[ "$u" -ge 0 ] && [ "$u" -le 2 ] || { echo "    !! $u frames neither taken in nor counted as dropped"; fail=1; }; done
	# a capture with no audio from the card must keep moving with silence at 48 kHz
	a=$(grep -o "expected [0-9]*: [0-9]*%" $o | grep -o "[0-9]*%" | tr -d %); echo "    audio without card audio ran at ${a:-?}% of 48 kHz"
	[ "${a:-0}" -ge 95 ] && [ "${a:-0}" -le 105 ] || { echo "    !! silent audio stream at ${a:-?}% of its rate"; fail=1; }
	scan $o; [ $rc = 0 ] || fail=1
done; done
unset SIM_DMA_LOCK SIM_NO_DIRECT SIM_LOSE_IRQ SIM_DUP_IRQ SIM_NTSC
for f in 60 240; do
	o=$S/sim/out_hostile_$f.txt
	SIM_HOSTILE=1 ASAN_OPTIONS=detect_leaks=0 timeout 300 $S/sim/sim $f 45 > $o 2>&1; rc=$?
	echo "== sim $f fps, mode=hostile (1 s phases of garbage registers, IRQ storm, DMA garbage, ALSA open/close, then sane hardware): exit $rc"
	grep -E "^coverage|ALSA periods|delivered|out of order|sequence not|timestamp not|STREAMOFF|ERROR b" $o | sed 's/^/    /'
	for k in "torn/mixed, steady state" "duplicate frames" "frames out of order" "sequence not increasing" "timestamp not increasing" "left in driver after STREAMOFF" "completed after STREAMOFF"; do
		v=$(grep "$k" $o | grep -o '[0-9]\+' | tail -1); [ "${v:-1}" = 0 ] || { echo "    !! $k = $v"; fail=1; }; done
	# the fuzz must actually reach the data paths, not just bounce off the checks
	for v in $(grep "^coverage" $o | grep -o '[0-9]\+' | head -3); do [ "$v" -gt 0 ] || { echo "    !! a data path was never reached"; fail=1; }; done
	scan $o; [ $rc = 0 ] || fail=1
done
# the card's interrupts die for 6 s on one channel: the watchdog must notice, show the
# no-signal picture, and frames must flow in order again afterwards
o=$S/sim/out_stall.txt
SIM_STALL=1 ASAN_OPTIONS=detect_leaks=0 timeout 300 $S/sim/sim 60 30 > $o 2>&1; rc=$?
echo "== sim 60 fps, interrupts of ch0 dead 10-16 s (watchdog): exit $rc"
grep -E "no frames from the card|bringing the card up|frames from the card again|watchdog re-inits|no-signal" $o | sed 's/^/    /'
grep -q "no frames from the card for 2 s" $o && grep -q "frames from the card again" $o && grep -q "watchdog re-inits *1 " $o || { echo "    !! the watchdog didn't notice the stall or the frames' return"; fail=1; }
for k in "torn/mixed, steady state" "duplicate frames" "frames out of order" "sequence not increasing" "timestamp not increasing" "left in driver after STREAMOFF" "completed after STREAMOFF"; do
	v=$(grep "$k" $o | grep -o '[0-9]\+' | tail -1); [ "${v:-1}" = 0 ] || { echo "    !! $k = $v"; fail=1; }; done
scan $o; [ $rc = 0 ] || fail=1
echo "== TSan (120 fps, 20 s)"
(cd $S/sim && gcc -O1 -g -fsanitize=thread -pthread -w -I. -Ifake -I$SRC -include shim.h -o sim_tsan sim.c && TSAN_OPTIONS=report_signal_unsafe=0 timeout 400 setarch -R ./sim_tsan 120 20 > tsan.txt 2>&1) || true
[ -x $S/sim/sim_tsan ] || { echo "!! TSan build FAILED"; fail=1; }
grep -E "steady|duplicate|STREAMOFF" $S/sim/tsan.txt | sed 's/^/    /'
echo "    race reports: $(grep -c 'WARNING: ThreadSanitizer: data race' $S/sim/tsan.txt); top frames:"
grep -A2 "WARNING: ThreadSanitizer: data race" $S/sim/tsan.txt | grep -E "^ +#0" | sed -E 's/^ +#0 ([^ ]+) .*(core.inc|sim.c|shim.h|memintrinsics.inc|interceptors[^:]*):([0-9]+).*/\1 \2:\3/' | sort | uniq -c | sed 's/^/      /'
echo "RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL)"
