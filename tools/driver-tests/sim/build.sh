#!/bin/sh
# build.sh <driver src dir> : extract the capture core and build the simulator
set -e
H=$(cd "$(dirname "$0")" && pwd); SRC=$1
mkdir -p $H/fake
for f in $(grep -o '#include <[^>]*>' $SRC/hws.h $SRC/hws_reg.h | sed 's/.*<\(.*\)>/\1/' | sort -u); do mkdir -p $H/fake/$(dirname $f); : > $H/fake/$f; done
python3 -I $H/../conv/extract.py $SRC/hws_video.c > $H/conv.inc
python3 -I $H/extract.py $SRC/hws_video.c hws_frame_period_ns hws_copy_too_late hws_tstat_max hws_tstat_frame_in hws_tstat_counters hws_tstat_fold \
  hws_take_buffers hws_max_queued hws_complete_buffers hws_nosig_work video_data_process \
  hws_buffer_queue_multi hws_return_buffers hws_start_streaming_multi hws_stop_streaming_multi \
  EnableVideoCapture CheckVideoCapture CheckCardStatus StartVideoCapture StopVideoCapture \
  SetVideoFormteSize ChangeVideoSize hws_get_mark hws_put_mark hws_direct_requeue_locked hws_direct_reclaim hws_fanout_drain hws_fanout_work hws_direct_complete hws_dcopy hws_copy_doubled hws_take_late_buffers hws_inline_fanout hws_list_count hws_frame_drop hws_direct_half hws_copy_half MemCopyVideoToSteam MemCopyAudioToSteam SetAudioQuene \
  hws_audio_dpc SetQuene hws_video_dpc irqhandler hws_irq_thread hws_copy_work \
  hws_enable_card_irqs CheckAudioCapture StartAudioCapture StopAudioCapture EnableAudioCapture SetDMAAddress InitVideoSys hws_measure_rate hws_reset_rate Get_Video_Status CheckVideFmt _deliver_samples audio_data_process hws_audio_silence_work > $H/named.inc
# forward declarations for everything extracted
python3 -I - $H/conv.inc $H/named.inc > $H/protos.inc <<'PY'
import re, sys
for f in sys.argv[1:]:
    t = open(f).read()
    for m in re.finditer(r"^((?:static\s+)?(?:inline\s+)?[A-Za-z_][\w\s\*]*?\b\w+\s*\([^;{]*?\))\s*\n?\s*\{", t, re.M):
        sig = m.group(1)
        if re.match(r"\s*(if|for|while|switch)\b", sig): continue
        sig = re.sub(r"\s+", " ", sig).strip()
        print(("" if sig.startswith("static") else "static ") + sig + ";")
PY
cat $H/protos.inc $H/conv.inc $H/named.inc > $H/core.inc
gcc -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -pthread -w \
    -I$H -I$H/fake -I$SRC -include $H/shim.h -o $H/sim $H/sim.c
echo built
