#!/bin/bash
# Try this driver on a machine that has another HwsCapture driver installed,
# without installing anything:
#
#   1. builds the driver from this checkout in a temporary directory
#   2. unloads the currently loaded HwsCapture module and loads the new one
#      (in memory only - nothing under /lib/modules or DKMS is touched)
#   3. runs tools/card-check.sh against it
#   4. puts the original driver back (also on errors and Ctrl-C)
#
# Everything is written to a log file in the current directory.
#
#   tools/test-on-card.sh            # test, then restore the original driver
#   tools/test-on-card.sh --keep     # leave the new driver loaded afterwards
#   tools/test-on-card.sh --leak     # memory leak check (tools/leak-check.sh) instead
#   tools/test-on-card.sh --compare  # same check on the installed driver first, then
#                                    # on this one, and a table of both results
#   DUR=60 tools/test-on-card.sh     # longer captures (each phase)
#
# --compare leaves v4l2-compliance out for the installed driver (older versions
# of this driver crash the kernel in the control ioctls it tests);
# OLD_COMPLIANCE=1 runs it there too.
#
# A reboot always brings back the installed driver.
set -u
export LC_ALL=C	# awk/printf with a decimal point, whatever the desktop's language
HERE=$(cd "$(dirname "$0")/.." && pwd)
KEEP=0; LEAK=0; COMPARE=0
for a in "$@"; do
	case "$a" in
	--keep) KEEP=1 ;;
	--leak) LEAK=1 ;;
	--compare) COMPARE=1 ;;
	*) echo "unknown option $a (--keep, --leak, --compare)"; exit 2 ;;
	esac
done
[ $LEAK = 1 ] && [ $COMPARE = 1 ] && { echo "--leak and --compare can't be combined"; exit 2; }
BUILD=""
SWAPPED=0
LOG=$PWD/test-on-card-$(date +%Y%m%d-%H%M%S).log
# tee ignores Ctrl-C/hangup: it must outlive this script's cleanup, or the cleanup's
# own output kills the script (SIGPIPE) before the original driver is back
exec > >(trap '' INT TERM HUP; exec tee "$LOG") 2>&1
trap '' PIPE

say() { printf '%s\n' "$*"; }
die() { say "ERROR: $*"; exit 1; }

# compare_results <old results> <new results>: one row per number, both drivers side by side
compare_results() {
	say ""
	say "################################################################ comparison"
	say "old: $OLD_DESC"
	say "new: this checkout (srcversion $NEW_SRC)"
	say ""
	# card-check runs as root and owns the files it wrote: only create missing ones
	for f in "$1" "$2"; do [ -e "$f" ] || : > "$f"; done
	awk '
		function kv(line, which,   n, i, a, key, f) {
			n = split(line, a, " "); key = a[1]
			if (!(key in seen)) { seen[key] = 1; order[++nk] = key }
			for (i = 2; i <= n; i++) if (split(a[i], f, "=") == 2) v[which, key, f[1]] = f[2]
		}
		# the new run first: its order (with v4l2-compliance) sets the rows
		FILENAME == ARGV[1] { kv($0, "new"); next }
		{ kv($0, "old") }
		function get(w, k, f) { return ((w, k, f) in v) ? v[w, k, f] : "-" }
		function cpucell(w, k) { return get(w, k, "busy") (get(w, k, "channels") == "-" ? "" : " (" get(w, k, "channels") " ch)") }
		# "-" (or "-/-") = not measured; a negative number is a measurement
		function na(x) { return x == "" || x == "-" || x ~ /^-\// }
		function missing(w, k,   f, t) {
			f = get(w, k, "frames"); t = get(w, k, "want")
			if (f == "-" || t == "-") return "-"
			return t - f > 0 ? t - f : 0	# a capture can get a frame more than the time allows
		}
		function row(label, o, n,   mark) {
			mark = ""
			if (o != n && !na(o) && !na(n) && better != "") {
				if ((better == "low" && n + 0 < o + 0) || (better == "high" && n + 0 > o + 0)) mark = "  better"
				else if ((better == "low" && n + 0 > o + 0) || (better == "high" && n + 0 < o + 0)) mark = "  worse"
			}
			printf "%-46s %16s %16s%s\n", label, o, n, mark
		}
		function cap(k, label,   w) {
			# the wanted count follows each driver frame rate (60 vs 59.94): judge what is missing
			better = ""; row(label ": frames received",
				get("old", k, "frames") "/" get("old", k, "want"), get("new", k, "frames") "/" get("new", k, "want"))
			better = "low"
			row("   frames missing", missing("old", k), missing("new", k))
			row("   lost (sequence gaps)", get("old", k, "lost"), get("new", k, "lost"))
			row("   duplicate / out of order", get("old", k, "dup"), get("new", k, "dup"))
			row("   late (>1.5 frame periods)", get("old", k, "late"), get("new", k, "late"))
			row("   longest gap, ms", get("old", k, "maxgap"), get("new", k, "maxgap"))
			if (get("old", k, "wrong") + get("new", k, "wrong") > 0) row("   wrong size", get("old", k, "wrong"), get("new", k, "wrong"))
			better = ""; row("   fps from the timestamps", get("old", k, "fps"), get("new", k, "fps"))
			# totals only over captures both drivers made: inputs without a signal
			# are captured from a driver that never reports "no signal", skipped
			# with one that does
			if (get("old", k, "lost") != "-" && get("new", k, "lost") != "-")
				for (w = 1; w <= 2; w++) {
					which = w == 1 ? "old" : "new"
					tl[which] += get(which, k, "lost") + get(which, k, "dup")
					tlate[which] += get(which, k, "late")
					tmiss[which] += missing(which, k)
				}
		}
		END {
			printf "%-46s %16s %16s\n", "", "installed", "new"
			for (i = 1; i <= nk; i++) {
				k = order[i]; split(k, p, "."); ch = p[1]; what = p[2]
				if (what == "info") { better = ""; row(ch ": input", get("old", k, "fmt") "@" get("old", k, "fps"), get("new", k, "fmt") "@" get("new", k, "fps")) }
				else if (what == "compliance") { better = "low"; row(ch ": v4l2-compliance failures", get("old", k, "failed"), get("new", k, "failed")) }
				else if (what == "input") cap(k, ch ": capture at the input size")
				else if (what == "rate") { better = ""; row(ch ": frame rate reported to apps", get("old", k, "reported"), get("new", k, "reported")) }
				else if (what == "scaled") cap(k, ch ": capture scaled")
				else if (what == "picture") { better = ""; row(ch ": scaled picture, distinct byte values", get("old", k, "values"), get("new", k, "values")) }
				else if (what == "together") cap(k, ch ": all channels + audio at once")
				else if (what == "scalecheck") { better = "high"
					row(ch ": scaled vs. input, brightness match", get("old", k, "luma"), get("new", k, "luma"))
					row("   colour match U", get("old", k, "u"), get("new", k, "u"))
					row("   colour match V", get("old", k, "v"), get("new", k, "v")) }
				else if (what == "controls") { better = "high"; row(ch ": brightness control, min->max change", get("old", k, "diff"), get("new", k, "diff")) }
				else if (what == "nosig") { better = "high"; row(ch ": no-signal picture with text (1 = yes)", get("old", k, "text"), get("new", k, "text")) }
				else if (what == "userptr") cap(k, ch ": user-pointer buffers")
				else if (what == "app1") cap(k, ch ": two apps at once, app 1")
				else if (what == "app2") cap(k, ch ": two apps at once, app 2")
				else if (what == "loaded") cap(k, ch ": every CPU busy")
				else if (what == "after") cap(k, ch ": after the kill test")
				else if (what == "observer") cap(k, ch ": while apps elsewhere were killed")
				else if (ch == "chaos") { better = "low"; row("kill test: apps stuck after kill -9", get("old", k, "hung"), get("new", k, "hung")) }
				else if (ch == "aclock") { better = ""
					row("audio " what ": samples per second", get("old", k, "rate"), get("new", k, "rate"))
					row("   loudest sample", get("old", k, "peak"), get("new", k, "peak")) }
				else if (ch == "audio") { better = "high"; row("audio " what " ok (1 = yes)", get("old", k, "ok"), get("new", k, "ok")) }
				else if (ch == "accounting") { better = "low"
					row("driver " what ": frames dropped on the way in", get("old", k, "dropped"), get("new", k, "dropped"))
					row("   frames unaccounted (must be 0)", get("old", k, "unaccounted"), get("new", k, "unaccounted"))
					row("   frames gone, their interrupt lost", get("old", k, "irq_lost"), get("new", k, "irq_lost"))
					row("   no frame from the card (late)", get("old", k, "card_late"), get("new", k, "card_late"))
					row("      of these right after a capture start", get("old", k, "late_start"), get("new", k, "late_start")) }
				else if (ch == "cpu") {
					# comparable only with as many channels streaming on both
					better = get("old", k, "channels") == get("new", k, "channels") ? "low" : ""
					row("whole system CPU busy, % (all at once)", cpucell("old", k), cpucell("new", k))
				}
				else if (ch == "kernel") { better = "low"; row("kernel errors (Oops, BUG, WARNING)", get("old", k, "problems"), get("new", k, "problems")) }
				else if (ch == "result") { better = "high"; row("card-check result OK (1 = yes)", get("old", k, "ok"), get("new", k, "ok")) }
			}
			print ""
			better = "low"
			row("TOTAL frames lost or duplicated", tl["old"] + 0, tl["new"] + 0)
			row("TOTAL frames missing (not received)", tmiss["old"] + 0, tmiss["new"] + 0)
			row("TOTAL late frames", tlate["old"] + 0, tlate["new"] + 0)
		}' "$2" "$1"
	say ""
	say "\"-\" = not measured (e.g. no signal, or v4l2-compliance skipped for the installed driver)."
	say "TOTAL rows count only the captures both drivers made."
	say "Late frames with equal frame counts from card and driver come from the card/source."
}

# ---------------------------------------------------------------- checks
say "== $(date)  kernel $(uname -r)  repo $(git -C "$HERE" log -1 --format='%h %s' 2>/dev/null)"
KDIR=/lib/modules/$(uname -r)/build
[ -d "$KDIR" ] || die "kernel headers for $(uname -r) missing (see README, step 1)"
for t in make gcc v4l2-ctl v4l2-compliance arecord; do
	command -v $t >/dev/null || die "missing $t (packages: build-essential v4l-utils alsa-utils)"
done
sudo -v || die "needs sudo for loading the module"

# what is loaded now, so it can be put back exactly
ORIG_LOADED=0; ORIG_FILE=""; ORIG_SRC=""
if [ -d /sys/module/HwsCapture ]; then
	ORIG_LOADED=1
	ORIG_SRC=$(cat /sys/module/HwsCapture/srcversion 2>/dev/null)
fi
ORIG_FILE=$(modinfo -F filename HwsCapture 2>/dev/null)
say "== installed driver: ${ORIG_FILE:-none}  (loaded: $([ $ORIG_LOADED = 1 ] && echo "yes, srcversion $ORIG_SRC" || echo no))"

# ---------------------------------------------------------------- build
BUILD=$(mktemp -d /tmp/hws-test-build.XXXXXX)
cp "$HERE"/src/*.c "$HERE"/src/*.h "$HERE"/src/Makefile "$BUILD"/ || die "copying the sources failed"
say "== building in $BUILD"
make -C "$KDIR" M="$BUILD" modules > "$BUILD/build.log" 2>&1 || { tail -20 "$BUILD/build.log"; die "build failed"; }
NEW_SRC=$(modinfo -F srcversion "$BUILD/HwsCapture.ko")
say "   built HwsCapture.ko (srcversion $NEW_SRC)"

# ---------------------------------------------------------------- nothing may use the card
# one line per user of this driver's devices: "<device> <pid> <program> <uid>"
card_users() {
	local dev n p
	for dev in /dev/video* /dev/snd/*; do
		[ -e "$dev" ] || continue
		case "$dev" in
		/dev/video*) v4l2-ctl -d "$dev" --info 2>/dev/null | grep -q 'Driver name *: *HwsCapture' || continue ;;
		/dev/snd/*) n=$(basename "$dev" | sed -n 's/^[a-zA-Z]*C\([0-9]*\).*/\1/p'); [ -n "$n" ] || continue
			# /proc/asound/cards: " 1 [HAudio1        ]: HwsCapture - HAudio 1"
			grep -qE "^ *$n \[.*\]: HwsCapture" /proc/asound/cards 2>/dev/null || continue ;;
		esac
		for p in $(sudo fuser "$dev" 2>/dev/null); do
			echo "$dev $p $(ps -o comm= -p "$p" 2>/dev/null) $(ps -o uid= -p "$p" 2>/dev/null | tr -d ' ')"
		done
	done
}
# PipeWire/WirePlumber of this user only watch the card; they can be stopped
# for the test and started again afterwards. Their sockets restart them on
# demand, so those have to be stopped too.
PW_UNITS="pipewire.socket pipewire-pulse.socket wireplumber.service pipewire-pulse.service pipewire.service"
PW_STOPPED=0
restart_pipewire() {
	[ $PW_STOPPED = 1 ] || return
	say "== starting PipeWire/WirePlumber again"
	systemctl --user start pipewire.socket pipewire-pulse.socket wireplumber.service pipewire.service pipewire-pulse.service 2>/dev/null ||
		say "   !! could not start them; log out and in again, or: systemctl --user start pipewire wireplumber"
	PW_STOPPED=0
}
restore() {
	[ $SWAPPED = 1 ] || return
	if [ $KEEP = 1 ]; then
		say "== --keep: the new driver stays loaded until 'sudo rmmod HwsCapture' or a reboot"
		return
	fi
	say "== restoring the original driver"
	# a capture that is still closing can hold the device for a moment
	for i in 1 2 3 4 5 6 7 8 9 10; do
		[ -d /sys/module/HwsCapture ] || break
		sudo rmmod HwsCapture 2>/dev/null && break
		sleep 1
	done
	if [ $ORIG_LOADED = 1 ]; then
		sudo modprobe HwsCapture 2>/dev/null || { [ -n "$ORIG_FILE" ] && sudo insmod "$ORIG_FILE"; }
		now=$(cat /sys/module/HwsCapture/srcversion 2>/dev/null)
		[ "$now" = "$ORIG_SRC" ] && say "   original driver loaded again" ||
			say "   !! could not reload the original driver (srcversion '$now'); a reboot restores it"
	else
		say "   no driver was loaded before; left unloaded"
	fi
	SWAPPED=0
}
# from here on, whatever happens, the original state comes back
trap '[ -n "${SUDO_KEEP:-}" ] && kill $SUDO_KEEP 2>/dev/null; restore; restart_pipewire; [ -n "$BUILD" ] && rm -rf "$BUILD"; say "log: $LOG"' EXIT
trap 'say "interrupted"; exit 130' INT TERM HUP
# a long run outlasts sudo's login: keep it fresh, so the restore never waits for a password
( while sleep 60; do sudo -n -v 2>/dev/null || exit; done ) &
SUDO_KEEP=$!
users=$(card_users)
if [ -n "$users" ] && ! echo "$users" | awk -v me="$(id -u)" '$3 !~ /^(pipewire|pipewire-pulse|wireplumber)$/ || $4 != me { bad = 1 } END { exit bad }'; then
	:	# someone else than our own PipeWire: report below
elif [ -n "$users" ]; then
	say "== stopping PipeWire/WirePlumber for the test (desktop audio pauses; started again at the end)"
	systemctl --user stop $PW_UNITS 2>/dev/null
	PW_STOPPED=1
	sleep 2
	users=$(card_users)
fi
if [ -n "$users" ]; then
	restart_pipewire
	say "The card is in use:"
	echo "$users" | while read -r dev pid comm uid; do say "   $dev  pid $pid  $comm  (uid $uid)"; done
	die "close these programs first, nothing was changed"
fi

# ---------------------------------------------------------------- --compare: the installed driver first
OLD_RES=$BUILD/results.old; NEW_RES=$BUILD/results.new
OLD_DESC=""
if [ $COMPARE = 1 ]; then
	say ""
	if [ $ORIG_LOADED = 0 ]; then
		say "== loading the installed driver for the comparison"
		sudo modprobe HwsCapture 2>/dev/null || die "no installed HwsCapture driver to compare with (modprobe failed)"
		SWAPPED=1	# from now on the exit handler unloads it again (it wasn't loaded before)
		sleep 3	# let udev create the device nodes
	fi
	OLD_DESC="installed driver $(modinfo -F version HwsCapture 2>/dev/null) (srcversion $(cat /sys/module/HwsCapture/srcversion 2>/dev/null))"
	say "################################################################ 1/2: $OLD_DESC"
	sudo env DUR="${DUR:-10}" ${SCALE_DUR:+SCALE_DUR=$SCALE_DUR} SKIP_COMPLIANCE="$([ "${OLD_COMPLIANCE:-0}" = 1 ] && echo 0 || echo 1)" \
		${EXTRA:+EXTRA=$EXTRA} ${CHAOS:+CHAOS=$CHAOS} ${ADUR:+ADUR=$ADUR} \
		RESULTS="$OLD_RES" "$HERE/tools/card-check.sh"
	say ""
	say "################################################################ 2/2: this checkout (srcversion $NEW_SRC)"
fi

# ---------------------------------------------------------------- swap, with guaranteed restore
# swapped before the unload: Ctrl-C while rmmod runs must still bring the original back
SWAPPED=1
if [ -d /sys/module/HwsCapture ]; then
	sudo rmmod HwsCapture || die "can't unload the current driver (still in use?)"
fi
# timestamps, not line counts: a full kernel log buffer drops old lines as new ones come
DMESG_START=$(sudo dmesg | tail -1 | sed -n 's/^\[ *\([0-9.]*\)\].*/\1/p')
kmsg_since() { sudo dmesg | awk -v t="${DMESG_START:-0}" 'match($0, /^\[ *[0-9.]+\]/) { v = substr($0, RSTART + 1, RLENGTH - 2); gsub(/ /, "", v); if (v + 0 > t + 0) print }'; }
if ! out=$(sudo insmod "$BUILD/HwsCapture.ko" timing_debug=1 dma_probe=1 2>&1); then
	say "$out"
	echo "$out" | grep -qi 'key was rejected' && say "   Secure Boot is on: the module must be signed (install with dkms-install.sh, see README) or Secure Boot disabled"
	die "loading the new driver failed"
fi
[ "$(cat /sys/module/HwsCapture/srcversion 2>/dev/null)" = "$NEW_SRC" ] || die "the loaded module is not the one just built"
say "== new driver loaded:"
sleep 3	# let udev create the device nodes
kmsg_since | grep -i hwscapture | sed 's/^/   /'

# ---------------------------------------------------------------- test
say ""
if [ $LEAK = 1 ]; then
	# with the module file, so it can also unload and reload the driver
	sudo env HWS_KO="$BUILD/HwsCapture.ko" ${CYCLES:+CYCLES=$CYCLES} ${ACYCLES:+ACYCLES=$ACYCLES} ${RELOADS:+RELOADS=$RELOADS} "$HERE/tools/leak-check.sh"
else
	sudo env DUR="${DUR:-10}" ${SCALE_DUR:+SCALE_DUR=$SCALE_DUR} SKIP_COMPLIANCE="${SKIP_COMPLIANCE:-0}" \
		${EXTRA:+EXTRA=$EXTRA} ${CHAOS:+CHAOS=$CHAOS} ${ADUR:+ADUR=$ADUR} \
		$([ $COMPARE = 1 ] && echo "RESULTS=$NEW_RES") "$HERE/tools/card-check.sh"
fi
rc=$?
[ $COMPARE = 1 ] && compare_results "$OLD_RES" "$NEW_RES"

say ""
say "== all kernel messages during the test"
kmsg_since | grep -v ': dma probe, ' | sed 's/^/   /' | tail -60
exit $rc
