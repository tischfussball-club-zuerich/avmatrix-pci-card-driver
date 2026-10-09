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
rm -f $S/sim/sim $S/sim/sim_tsan
if ! $S/sim/build.sh $SRC > $S/sim/build.log 2>&1; then echo "!! simulator build FAILED:"; grep -m5 error $S/sim/build.log; echo "RESULT: FAIL"; exit 1; fi
for mode in race logic nodirect irqfaults; do for f in 60 240 480; do
	o=$S/sim/out_${mode}_$f.txt
	unset SIM_DMA_LOCK SIM_NO_DIRECT SIM_LOSE_IRQ SIM_DUP_IRQ
	if [ $mode = logic ]; then export SIM_DMA_LOCK=1; fi
	if [ $mode = nodirect ]; then export SIM_NO_DIRECT=1; fi
	if [ $mode = irqfaults ]; then export SIM_LOSE_IRQ=40 SIM_DUP_IRQ=40; fi
	ASAN_OPTIONS=detect_leaks=0 timeout 300 $S/sim/sim $f 45 > $o 2>&1; rc=$?
	echo "== sim $f fps, mode=$mode (race=card DMA race present, logic=race removed, nodirect=ring path only, irqfaults=lost+duplicate IRQs): exit $rc"
	grep -E "^frames|delivered|restarts|steady|duplicate|out of order|mod 100|sequence not|timestamp not|STREAMOFF|ERROR b|stalls|latency|driver stats" $o | sed 's/^/    /'
	for k in "torn/mixed, steady state" "duplicate frames" "frames out of order" "sequence not increasing" "timestamp not increasing" "left in driver after STREAMOFF" "completed after STREAMOFF" "2 s stalls"; do
		v=$(grep "$k" $o | grep -o '[0-9]\+' | tail -1); [ "${v:-1}" = 0 ] || { echo "    !! $k = $v"; fail=1; }; done
	scan $o; [ $rc = 0 ] || fail=1
done; done
unset SIM_DMA_LOCK SIM_NO_DIRECT SIM_LOSE_IRQ SIM_DUP_IRQ
echo "== TSan (120 fps, 20 s)"
(cd $S/sim && gcc -O1 -g -fsanitize=thread -pthread -w -I. -Ifake -I$SRC -include shim.h -o sim_tsan sim.c && TSAN_OPTIONS=report_signal_unsafe=0 timeout 400 setarch -R ./sim_tsan 120 20 > tsan.txt 2>&1) || true
[ -x $S/sim/sim_tsan ] || { echo "!! TSan build FAILED"; fail=1; }
grep -E "steady|duplicate|STREAMOFF" $S/sim/tsan.txt | sed 's/^/    /'
echo "    race reports: $(grep -c 'WARNING: ThreadSanitizer: data race' $S/sim/tsan.txt); top frames:"
grep -A2 "WARNING: ThreadSanitizer: data race" $S/sim/tsan.txt | grep -E "^ +#0" | sed -E 's/^ +#0 ([^ ]+) .*(core.inc|sim.c|shim.h|memintrinsics.inc|interceptors[^:]*):([0-9]+).*/\1 \2:\3/' | sort | uniq -c | sed 's/^/      /'
echo "RESULT: $([ $fail = 0 ] && echo PASS || echo FAIL)"
