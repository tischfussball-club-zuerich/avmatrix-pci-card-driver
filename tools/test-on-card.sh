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
#   DUR=60 tools/test-on-card.sh     # longer captures (each phase)
#
# A reboot always brings back the installed driver.
set -u
HERE=$(cd "$(dirname "$0")/.." && pwd)
KEEP=0
[ "${1:-}" = "--keep" ] && KEEP=1
BUILD=""
SWAPPED=0
LOG=$PWD/test-on-card-$(date +%Y%m%d-%H%M%S).log
exec > >(tee "$LOG") 2>&1

say() { printf '%s\n' "$*"; }
die() { say "ERROR: $*"; exit 1; }

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
	sudo rmmod HwsCapture 2>/dev/null
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
trap 'restore; restart_pipewire; [ -n "$BUILD" ] && rm -rf "$BUILD"; say "log: $LOG"' EXIT
trap 'say "interrupted"; exit 130' INT TERM
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

# ---------------------------------------------------------------- swap, with guaranteed restore

if [ $ORIG_LOADED = 1 ]; then
	sudo rmmod HwsCapture || die "can't unload the current driver (still in use?), nothing was changed"
fi
SWAPPED=1
DMESG_START=$(sudo dmesg | wc -l)
if ! out=$(sudo insmod "$BUILD/HwsCapture.ko" timing_debug=1 2>&1); then
	say "$out"
	echo "$out" | grep -qi 'key was rejected' && say "   Secure Boot is on: the module must be signed (install with dkms-install.sh, see README) or Secure Boot disabled"
	die "loading the new driver failed"
fi
[ "$(cat /sys/module/HwsCapture/srcversion 2>/dev/null)" = "$NEW_SRC" ] || die "the loaded module is not the one just built"
say "== new driver loaded:"
sleep 3	# let udev create the device nodes
sudo dmesg | tail -n +$((DMESG_START + 1)) | grep -i hwscapture | sed 's/^/   /'

# ---------------------------------------------------------------- test
say ""
sudo env DUR="${DUR:-10}" ${SCALE_DUR:+SCALE_DUR=$SCALE_DUR} SKIP_COMPLIANCE="${SKIP_COMPLIANCE:-0}" "$HERE/tools/card-check.sh"
rc=$?

say ""
say "== all kernel messages during the test"
sudo dmesg | tail -n +$((DMESG_START + 1)) | sed 's/^/   /' | tail -60
exit $rc
