// SPDX-License-Identifier: GPL-2.0-or-later
/*
 */

#include <linux/pci.h>
#include <linux/kernel.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-dma-contig.h>
#include "hws.h"
#include "hws_reg.h"
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/rawmidi.h>
#include <sound/initval.h>

static void hws_adapters_init(struct hws_pcie_dev *dev);
static void hws_get_video_param(struct hws_pcie_dev *dev,int index);
static int StartVideoCapture(struct hws_pcie_dev *pdx,int index);
static int StartAudioCapture(struct hws_pcie_dev *pdx,int index);
static void StopAudioCapture(struct hws_pcie_dev *pdx,int index);
static void StopVideoCapture(struct hws_pcie_dev *pdx,int index);
static void InitVideoSys(struct hws_pcie_dev *pdx,int set);
static void hws_v4l2_dev_release(struct v4l2_device *v4l2_dev);
static void WRITE_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 RegisterOffset, u32 Value);
static u32 READ_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 RegisterOffset);
static void hws_video_unregister_nodes(struct hws_pcie_dev *dev);
static void hws_video_release_v4l2(struct hws_pcie_dev *dev);
static void hws_audio_unregister(struct hws_pcie_dev *dev);
static void hws_dev_release(struct kref *kref);
static void hws_snd_card_free(struct snd_card *card);
static void hws_direct_reclaim(struct hws_pcie_dev *pdx, int ch, struct hws_vfh_ctx *ctx);
static void hws_fanout_work(struct kthread_work *work);
static void hws_fanout_drain(struct hws_video *v);
static void video_data_process(struct kthread_work *p_work);
static void hws_hw_bringup(struct hws_pcie_dev *pdx);

static int direct_io = 1;
module_param(direct_io, int, 0644);
MODULE_PARM_DESC(direct_io, "Copy frames straight from DMA into app buffers when no conversion is needed (default 1)");

//------------------------ frame pipeline timing diagnostics
/*
 * Direct path with non-temporal stores: the 4 MB per frame written into the
 * app's buffer bypasses the CPU cache instead of evicting everything else
 * from it. Can help with several channels; can cost if the app reads the
 * frame right away on the same CPU. Off by default: measure on your system.
 */
static int nt_copy;
module_param(nt_copy, int, 0644);
MODULE_PARM_DESC(nt_copy, "Direct path: write frames with non-temporal stores, bypassing the CPU cache (default 0; interlaced input always does)");

/* direct path into an app buffer */
static inline void hws_dcopy(void *dst, const void *src, size_t n, bool nt)
{
	if (nt)
		memcpy_flushcache(dst, src, n);
	else
		memcpy(dst, src, n);
}

/*
 * Every open file has its own buffer queue, so without a limit any user with
 * the device could hold gigabytes of kernel memory nobody is charged for.
 */
static int buf_limit_mb = 1024;	/* 8 apps with 32 buffers of 1080p fit */
module_param(buf_limit_mb, int, 0644);
MODULE_PARM_DESC(buf_limit_mb, "Buffer memory of all applications on one channel, MiB (default 1024)");

static int timing_debug;
module_param(timing_debug, int, 0644);
MODULE_PARM_DESC(timing_debug, "Frame timing diagnostics: 0=off, 1=statistics (each second with a drop or late frame, else every 10 s), 2=every second and warn on late frames");

static u64 hws_frame_period_ns(struct hws_pcie_dev *pdx, int ch)
{
	u32 rate = pdx->m_pVCAPStatus[ch][0].dwFrameRate;

	if (rate == 0 || rate > 240)
		rate = 60;
	return div_u64(NSEC_PER_SEC, rate);
}

/*
 * The card rewrites a half one half-period after announcing it. A copy that
 * ends later than that may hold new data, even if the interrupt that would
 * show this (the other half's) was lost: treat it as overrun. The interrupt
 * time is taken when the handler runs, possibly late, so allow only 3/8 of a
 * period (2 ms of interrupt latency headroom at 60 fps).
 */
static bool hws_copy_too_late(struct hws_pcie_dev *pdx, int ch, u64 t_irq)
{
	return ktime_get_ns() - t_irq > hws_frame_period_ns(pdx, ch) * 3 / 8;
}

static inline void hws_tstat_max(u64 *max, u64 v)
{
	if (v > *max)
		*max = v;
}

/* Called from the IRQ thread once both halves of a frame are in the ring. */
static void hws_tstat_frame_in(struct hws_pcie_dev *pdx, int ch, u64 irq_ns)
{
	struct hws_timing_stats *ts = &pdx->tstats[ch];
	u64 interval;

	pdx->frame_open[ch] = false;	/* counted: see hws_frame_drop() */
	pdx->half1_drop[ch] = NULL;

	atomic_inc(&ts->frames_in);
	hws_tstat_max(&ts->copy_lat_max, ktime_get_ns() - irq_ns);

	if (ts->last_frame_ns) {
		u64 period = hws_frame_period_ns(pdx, ch);

		interval = irq_ns - ts->last_frame_ns;
		/* the exact rate (59.94 vs 60) from regular intervals; a frame the
		 * card skipped would bias it */
		if (interval > period / 2 && interval < period * 7 / 5)
		{
			atomic64_add(BIT_ULL(RATE_N_SHIFT) + interval, &ts->rate_acc);
		}
		if (interval < ts->interval_min)
			ts->interval_min = interval;
		hws_tstat_max(&ts->interval_max, interval);
		/* a gap the driver's own drops don't explain: no frame from the card */
		if (interval > period * 3 / 2 && !ts->lost_since_frame)
		{
			atomic_inc(&ts->card_late);
			if (ts->frames_since_start < 10)
				atomic_inc(&ts->card_late_start);
			if (timing_debug >= 2)
				pr_warn_ratelimited("HwsCapture ch%d: late frame from hardware, interval %llu us, frame %u after the capture start\n",
						    ch, div_u64(interval, NSEC_PER_USEC), ts->frames_since_start);
		}
	}
	ts->last_frame_ns = irq_ns;
	ts->lost_since_frame = false;
	ts->frames_since_start++;
}

static void hws_tstat_counters(struct hws_timing_stats *ts, atomic_t **c)
{
	c[HWS_ST_IN] = &ts->frames_in;
	c[HWS_ST_OUT] = &ts->frames_out;
	c[HWS_ST_IRQ_BUSY] = &ts->irq_busy_skip;
	c[HWS_ST_STALE] = &ts->stale_half;
	c[HWS_ST_OVERRUN] = &ts->dma_overrun;
	c[HWS_ST_DIRECT] = &ts->direct;
	c[HWS_ST_OVERWRITE] = &ts->ring_overwrite;
	c[HWS_ST_COALESCED] = &ts->wq_coalesced;
	c[HWS_ST_NO_BUF] = &ts->no_app_buf;
	c[HWS_ST_EMPTY] = &ts->worker_empty;
	c[HWS_ST_SKIP] = &ts->ring_skip;
	c[HWS_ST_UNPAIRED] = &ts->unpaired;
	c[HWS_ST_ABORTED] = &ts->aborted;
	c[HWS_ST_MISSED] = &ts->copy_missed;
	c[HWS_ST_IRQ_LOST] = &ts->irq_lost;
	c[HWS_ST_CARD_LATE] = &ts->card_late;
	c[HWS_ST_LATE_START] = &ts->card_late_start;
}

/* end a report window: its counts (win, may be NULL) go into the totals */
static void hws_tstat_fold(struct hws_timing_stats *ts, u32 *win)
{
	atomic_t *c[HWS_ST_N];
	unsigned long flags;
	int i;

	hws_tstat_counters(ts, c);
	spin_lock_irqsave(&ts->fold_lock, flags);
	for (i = 0; i < HWS_ST_N; i++) {
		u32 v = atomic_xchg(c[i], 0);

		ts->tot[i] += v;
		if (win)
			win[i] = v;
	}
	spin_unlock_irqrestore(&ts->fold_lock, flags);
}

static void hws_tstat_report(struct hws_pcie_dev *pdx)
{
	u64 now = ktime_get_ns();
	int ch;

	for (ch = 0; ch < pdx->m_nCurreMaxVideoChl; ch++) {
		struct hws_timing_stats *ts = &pdx->tstats[ch];
		u64 elapsed_ms = div_u64(now - ts->window_start_ns, NSEC_PER_MSEC);
		/*
		 * A line per second only when something happened (a drop, or a frame
		 * gap of over 1.5 frame periods); otherwise one every 10 s. The
		 * counters keep adding up in between, so totals stay complete.
		 */
		bool eventful = atomic_read(&ts->stale_half) || atomic_read(&ts->dma_overrun) ||
				atomic_read(&ts->unpaired) || atomic_read(&ts->ring_overwrite) ||
				atomic_read(&ts->aborted) || atomic_read(&ts->copy_missed) ||
				atomic_read(&ts->ring_skip) || atomic_read(&ts->no_app_buf) ||
				atomic_read(&ts->irq_lost) ||
				ts->interval_max * 2 > 3 * hws_frame_period_ns(pdx, ch);
		u32 w[HWS_ST_N];
		u32 c0, card;

		if (pdx->m_bVCapStarted[ch] && !eventful && timing_debug < 2 && elapsed_ms < 10000)
			continue;
		hws_tstat_fold(ts, w);
		/* frames the card completed (one second-half interrupt each) vs.
		 * frames the driver committed: more interrupts = frames lost in the driver */
		c0 = READ_ONCE(pdx->m_half_cnt[ch][0]);
		card = c0 - ts->last_half0_cnt;

		ts->last_half0_cnt = c0;

		if (pdx->m_bVCapStarted[ch] && elapsed_ms > 0)
			pr_info("HwsCapture ch%d: in %llu fps (hw %u) out %llu fps | frames card %u in %u | irq_busy %u stale_half %u dma_overrun %u unpaired %u aborted %u copy_missed %u direct %u overwrite %u skip %u wq_coalesced %u no_app_buf %u empty %u irq_lost %u card_late %u late_start %u | interval %llu-%llu us, copy_lat %llu us, copy_wake %llu us, copy_dur %llu us, wq_lat %llu us, lock_hold %llu us, backlog %d\n",
				ch, div64_u64((u64)w[HWS_ST_IN] * 1000, elapsed_ms),
				pdx->m_pVCAPStatus[ch][0].dwFrameRate,
				div64_u64((u64)w[HWS_ST_OUT] * 1000, elapsed_ms),
				card, w[HWS_ST_IN],
				w[HWS_ST_IRQ_BUSY], w[HWS_ST_STALE],
				w[HWS_ST_OVERRUN], w[HWS_ST_UNPAIRED], w[HWS_ST_ABORTED], w[HWS_ST_MISSED], w[HWS_ST_DIRECT],
				w[HWS_ST_OVERWRITE], w[HWS_ST_SKIP],
				w[HWS_ST_COALESCED],
				w[HWS_ST_NO_BUF], w[HWS_ST_EMPTY],
				w[HWS_ST_IRQ_LOST], w[HWS_ST_CARD_LATE], w[HWS_ST_LATE_START],
				ts->interval_min == U64_MAX ? 0 : div_u64(ts->interval_min, NSEC_PER_USEC),
				div_u64(ts->interval_max, NSEC_PER_USEC),
				div_u64(ts->copy_lat_max, NSEC_PER_USEC),
				div_u64(ts->copy_wake_max, NSEC_PER_USEC),
				div_u64(ts->copy_dur_max, NSEC_PER_USEC),
				div_u64(ts->wq_lat_max, NSEC_PER_USEC),
				div_u64(ts->lock_hold_max, NSEC_PER_USEC),
				ts->backlog_max);

		ts->interval_min = U64_MAX;
		ts->interval_max = 0;
		ts->copy_lat_max = 0;
		ts->copy_wake_max = 0;
		ts->copy_dur_max = 0;
		ts->wq_lat_max = 0;
		ts->lock_hold_max = 0;
		ts->backlog_max = 0;
		ts->window_start_ns = now;
	}
}


struct hws_scaler;
static	void VideoScaler(struct hws_scaler *sc, BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void FHD_To_HD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void HD_To_FHD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void FHD_To_800X600_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void HD_To_800X600_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void SD_PAL_To_SD_NTSC_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void SD_NTSC_To_SD_PAL_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void SD_PAL_To_FHD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void SD_PAL_To_HD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void SD_NTSC_To_FHD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void SD_NTSC_To_HD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void V1280X1024_To_FHD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void V1280X1024_To_HD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void V1280X1024_To_800X600_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void FHD_To_SD_NTSC_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void FHD_To_SD_PAL_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void HD_To_SD_NTSC_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void HD_To_SD_PAL_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void V1280X1024_NTSC_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void V1280X1024_PAL_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
static	void All_VideoScaler(struct hws_scaler *sc, BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
#define MAKE_ENTRY( __vend, __chip, __subven, __subdev, __configptr) {	\
.vendor		= (__vend),					\
.device		= (__chip),					\
.subvendor	= (__subven),					\
.subdevice	= (__subdev),					\
.driver_data	= (unsigned long) (__configptr)			\
}

static const struct pci_device_id hws_pci_table[] = {
	MAKE_ENTRY(0x8888, 0x9534, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8534, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8554, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8524, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x6524, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8504, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x6504, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8532, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8512, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8501, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x6502, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8504, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x1F33, 0x8524, 0x8888, 0x0007, NULL),
	MAKE_ENTRY(0x8888, 0x8521, 0x8888, 0x0007, NULL),	/* vendor 20260519 */

	{ }
};
static const v4l2_model_timing_t support_videofmt[]= {
	[V4L2_MODEL_VIDEOFORMAT_1920X1080P60]	= V4L2_MODEL_TIMING(1920,1080,60,0),
	[V4L2_MODEL_VIDEOFORMAT_1280X720P60]	= V4L2_MODEL_TIMING(1280,720,60,0),
	[V4L2_MODEL_VIDEOFORMAT_720X480P60]		= V4L2_MODEL_TIMING(720,480,60,0),
    [V4L2_MODEL_VIDEOFORMAT_720X576P50]		= V4L2_MODEL_TIMING(720,576,50,0),
	[V4L2_MODEL_VIDEOFORMAT_800X600P60]		= V4L2_MODEL_TIMING(800,600,60,0),
	[V4L2_MODEL_VIDEOFORMAT_640X480P60]		= V4L2_MODEL_TIMING(640,480,60,0),
	[V4L2_MODEL_VIDEOFORMAT_1024X768P60]	= V4L2_MODEL_TIMING(1024,768,60,0),
	[V4L2_MODEL_VIDEOFORMAT_1280X768P60]	= V4L2_MODEL_TIMING(1280,768,60,0),
	[V4L2_MODEL_VIDEOFORMAT_1280X800P60]	= V4L2_MODEL_TIMING(1280,800,60,0),
	[V4L2_MODEL_VIDEOFORMAT_1280X1024P60]	= V4L2_MODEL_TIMING(1280,1024,60,0),
	[V4L2_MODEL_VIDEOFORMAT_1360X768P60]	= V4L2_MODEL_TIMING(1360,768,60,0),
	[V4L2_MODEL_VIDEOFORMAT_1440X900P60]	= V4L2_MODEL_TIMING(1440,900,60,0),
	[V4L2_MODEL_VIDEOFORMAT_1680X1050P60]	= V4L2_MODEL_TIMING(1680,1050,60,0),
    [V4L2_MODEL_VIDEOFORMAT_1080X1920P60]	= V4L2_MODEL_TIMING(1080,1920,60,0),
};
static const framegrabber_pixfmt_t support_pixfmts[] = {

	[FRAMEGRABBER_PIXFMT_YUYV]={ //YUYV index=0
		.name     = "4:2:2, packed, YUYV",
		.fourcc   = V4L2_PIX_FMT_YUYV,
		.depth    = 16,
		.is_yuv   = true,
		.pixfmt_out = YUYV,
	},
};


/*
 * The portrait size (1080x1920, rotated) is the table's last entry. Offered
 * only on request: apps that pick the largest or the first-listed size (e.g.
 * GStreamer) would otherwise choose it and show a rotated picture. Load-time
 * only - a stream may hold that size.
 */
static bool portrait_output;
module_param(portrait_output, bool, 0444);
MODULE_PARM_DESC(portrait_output, "Also offer the rotated 1080x1920 portrait output size (default 0)");

static int hws_num_sizes(void)
{
	return portrait_output ? V4L2_MODEL_VIDEOFORMAT_NUM : V4L2_MODEL_VIDEOFORMAT_1080X1920P60;
}

static int v4l2_get_suport_VideoFormatIndex(struct v4l2_format *fmt)
{
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	int index;
	int videoIndex=-1;
	for(index=0; index<hws_num_sizes();index++)
	{
		if((pix->width==support_videofmt[index].frame_size.width)&&(pix->height==support_videofmt[index].frame_size.height))
		{
			videoIndex = index;
			break;
		}
	}
	return videoIndex;
}
static v4l2_model_timing_t *v4l2_model_get_support_videoformat(int index)
{
	if(index <0 ||index >=hws_num_sizes())
		return NULL;

	return (v4l2_model_timing_t *)&support_videofmt[index];
}


static framegrabber_pixfmt_t *v4l2_model_get_support_pixformat(int index)
{
	if(index <0 ||index >=ARRAY_SIZE(support_pixfmts))
		return NULL;

	return (framegrabber_pixfmt_t *)&support_pixfmts[index];
}
static const framegrabber_pixfmt_t *framegrabber_g_support_pixelfmt_by_fourcc(u32 fourcc)
{
	int i;
	int pixfmt_index=-1;
	for(i=0;i<FRAMEGRABBER_PIXFMT_MAX;i++)
	{
		if(support_pixfmts[i].fourcc==fourcc)
		{

			pixfmt_index=i;
			break;
		}
	}
	if(pixfmt_index==-1)
		return NULL;

	return &support_pixfmts[pixfmt_index];
}

/* every format reply carries complete colorimetry, never a caller's leftovers */
static void hws_fill_colorimetry(struct v4l2_pix_format *pix)
{
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	pix->flags = 0;
}

static int hws_vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct hws_video *videodev = video_drvdata(file);
	struct hws_pcie_dev *dev = videodev->dev;
	int vi_index;
	vi_index = videodev->index+1+dev->m_Device_PortID*dev->m_nCurreMaxVideoChl;
	strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	sprintf(cap->card, "%s %d",HWS_VIDEO_NAME,vi_index);
	/* unique per card, so apps and v4l2-compliance can tell boards apart */
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", pci_name(dev->pdev));
	cap->device_caps =	V4L2_CAP_VIDEO_CAPTURE |V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}
static int hws_vidioc_enum_fmt_vid_cap(struct file *file, void *priv_fh,struct v4l2_fmtdesc *f)
{
	struct hws_video *videodev = video_drvdata(file);
	int index = f->index;

	if(f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
	{
		pr_debug("%s: bad type\n",__func__);
		return -EINVAL;
	}
	if(videodev)
	{
		const framegrabber_pixfmt_t *pixfmt;
		if(f->index >= FRAMEGRABBER_PIXFMT_MAX)
		{
			return -EINVAL;
		}
		else
		{
			pixfmt=v4l2_model_get_support_pixformat(f->index);
			if(pixfmt ==NULL) return -EINVAL;
			f->index = index;
			f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			strscpy(f->description, pixfmt->name, sizeof(f->description));
			f->pixelformat=pixfmt->fourcc;
		}
	}
	return 0;
}
static const framegrabber_pixfmt_t *framegrabber_g_out_pixelfmt(struct hws_video *dev)
{
	return &support_pixfmts[dev->current_out_pixfmt];
}
static void hws_follow_input(struct hws_video *videodev, int w, int h);

/*
 * An input change while buffers were allocated couldn't move the default
 * format then; once they are freed (an app handling SOURCE_CHANGE: STREAMOFF,
 * REQBUFS(0), G_FMT) it can. hws_follow_input() rechecks both conditions.
 */
static void hws_follow_current(struct hws_video *videodev)
{
	struct hws_pcie_dev *pdx = videodev->dev;
	int ch = videodev->index, w, h;
	unsigned long flags;

	if (READ_ONCE(videodev->fmt_by_app) || READ_ONCE(pdx->m_curr_No_Video[ch]))
		return;
	spin_lock_irqsave(&pdx->videoslock[ch], flags);
	w = pdx->m_pVCAPStatus[ch][0].dwWidth;
	h = pdx->m_pVCAPStatus[ch][0].dwHeight;
	if (pdx->m_pVCAPStatus[ch][0].dwinterlace)
		h *= 2;
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);
	if (w > 0 && h > 0 && (w != videodev->current_out_width || h != videodev->curren_out_height))
		hws_follow_input(videodev, w, h);
}

static int hws_vidioc_g_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *fmt)
{
	struct hws_video *videodev = video_drvdata(file);
	const framegrabber_pixfmt_t *pixfmt;
	v4l2_model_timing_t *p_SupportmodeTiming;

	hws_follow_current(videodev);
	pixfmt=framegrabber_g_out_pixelfmt(videodev);
	if(pixfmt)
	{
		p_SupportmodeTiming = v4l2_model_get_support_videoformat(videodev->current_out_size_index);
		if(p_SupportmodeTiming ==NULL) return -EINVAL;
		fmt->fmt.pix.width=p_SupportmodeTiming->frame_size.width;
		fmt->fmt.pix.height=p_SupportmodeTiming->frame_size.height;
		fmt->fmt.pix.field=V4L2_FIELD_NONE; //Field
		fmt->fmt.pix.pixelformat  = pixfmt->fourcc;
		fmt->fmt.pix.bytesperline = (fmt->fmt.pix.width * pixfmt->depth) >> 3;
		fmt->fmt.pix.sizeimage =	fmt->fmt.pix.height * fmt->fmt.pix.bytesperline;
		hws_fill_colorimetry(&fmt->fmt.pix);
		return 0;
	}

	return -EINVAL;
}
static v4l2_model_timing_t *Get_input_framesizeIndex(int width,int height)
{
	int i;
	for(i =0;i<hws_num_sizes();i++)
	{
		if((support_videofmt[i].frame_size.width ==width)&&(support_videofmt[i].frame_size.height == height))
		{
			return (v4l2_model_timing_t *)&support_videofmt[i];
		}
	}
	return NULL;
}

static int hws_vidioc_try_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct hws_video *videodev = video_drvdata(file);
	v4l2_model_timing_t *pModeTiming;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	const framegrabber_pixfmt_t *fmt;

    if (pix->pixelformat == 0) {
        if (framegrabber_g_support_pixelfmt_by_fourcc(videodev->pixfmt))
            pix->pixelformat = videodev->pixfmt;
        else
            pix->pixelformat = V4L2_PIX_FMT_YUYV;
    }

	fmt = framegrabber_g_support_pixelfmt_by_fourcc(pix->pixelformat);
	if(!fmt)
	{
        pix->pixelformat = V4L2_PIX_FMT_YUYV;
        fmt = framegrabber_g_support_pixelfmt_by_fourcc(pix->pixelformat);
        if (!fmt) {
            pr_debug("HwsCapture: %s: format not supported, not even the fallback\n", __func__);
            return -EINVAL;
        }
	}
	if (pix->width == 0 || pix->height == 0) {
			pModeTiming = v4l2_model_get_support_videoformat(videodev->current_out_size_index);
			if (!pModeTiming) return -EINVAL;
			pix->width  = pModeTiming->frame_size.width;
			pix->height = pModeTiming->frame_size.height;
	}
	pModeTiming = Get_input_framesizeIndex(pix->width,pix->height);
	if(!pModeTiming)
	{
		pr_debug("%s: %dx%d not supported, using current size\n",__func__,pix->width,pix->height);
		pModeTiming = v4l2_model_get_support_videoformat(videodev->current_out_size_index);
		if(pModeTiming ==NULL) return -EINVAL;
		pix->field = V4L2_FIELD_NONE;
		pix->width=pModeTiming->frame_size.width;
		pix->height=pModeTiming->frame_size.height;
		pix->bytesperline = (pix->width * fmt->depth) >> 3;
		pix->sizeimage =pix->height * pix->bytesperline;
		hws_fill_colorimetry(pix);
		pix->priv = 0;
		return 0;
	}
	pix->field = V4L2_FIELD_NONE;
	pix->width=pModeTiming->frame_size.width;
	pix->height=pModeTiming->frame_size.height;
	pix->bytesperline = (pix->width * fmt->depth) >> 3;
	pix->sizeimage =pix->height * pix->bytesperline;
	hws_fill_colorimetry(pix);
	pix->priv = 0;

	return 0;


	return 0;
}
/* true if any consumer of this node has buffers allocated */
static bool hws_video_busy(struct hws_video *videodev)
{
	struct hws_vfh_ctx *ctx;
	unsigned long flags;
	bool busy = false;

	spin_lock_irqsave(&videodev->consumers_lock, flags);
	list_for_each_entry(ctx, &videodev->consumers, node) {
		if (vb2_is_busy(&ctx->vbq)) {
			busy = true;
			break;
		}
	}
	spin_unlock_irqrestore(&videodev->consumers_lock, flags);
	return busy;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,struct v4l2_format *f)
{
	struct hws_video *videodev = video_drvdata(file);
	int nVideoFmtIndex;
	int err;
	unsigned long flags;
	struct hws_pcie_dev *pdx = videodev->dev;
	err = hws_vidioc_try_fmt_vid_cap(file, priv, f);
	if (err)
		return err;

	nVideoFmtIndex = v4l2_get_suport_VideoFormatIndex(f);
	if(nVideoFmtIndex ==-1) return -EINVAL;

	/*
	 * The output format is shared by every consumer of this node. Changing it
	 * while any of them has buffers allocated would make the worker write
	 * frames larger than those buffers.
	 */
	mutex_lock(&videodev->video_lock);
	if (hws_video_busy(videodev))
	{
		err = (f->fmt.pix.width == videodev->current_out_width &&
		       f->fmt.pix.height == videodev->curren_out_height &&
		       f->fmt.pix.pixelformat == videodev->pixfmt) ? 0 : -EBUSY;
		mutex_unlock(&videodev->video_lock);
		return err;
	}
	spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
	videodev->current_out_size_index = nVideoFmtIndex;
	videodev->pixfmt     = f->fmt.pix.pixelformat;
	videodev->current_out_width      = f->fmt.pix.width;
	videodev->curren_out_height     = f->fmt.pix.height;
	spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
	videodev->fmt_by_app = true;
	mutex_unlock(&videodev->video_lock);
	return 0;
}
/* frame rate of the input as measured by the card (60 until it reports one) */
static int hws_input_fps(struct hws_video *videodev)
{
	u32 r = READ_ONCE(videodev->dev->m_pVCAPStatus[videodev->index][0].dwFrameRate);

	return (r >= 1 && r <= 240) ? r : 60;
}

/* time per frame: measured exactly when known (59.94 = 1001/60000), else 1/the card's rate */
static void hws_timeperframe(struct hws_video *videodev, u32 *num, u32 *den)
{
	u64 tpf = atomic64_read(&videodev->tpf);	/* one load: never half of an old value */
	u32 n = tpf >> 32, d = (u32)tpf;

	if (n && d) {
		*num = n;
		*den = d;
	} else {
		*num = 1;
		*den = hws_input_fps(videodev);
	}
}

/* frames come at the input's rate; there is no frame rate to choose */
static int hws_vidioc_g_parm(struct file *file,void *fh, struct v4l2_streamparm *setfps)
{
	struct hws_video *videodev = video_drvdata(file);

	if (setfps->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	memset(&setfps->parm.capture, 0, sizeof(setfps->parm.capture));
	setfps->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	hws_timeperframe(videodev, &setfps->parm.capture.timeperframe.numerator,
			 &setfps->parm.capture.timeperframe.denominator);
	return 0;
}
static int hws_vidioc_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize)
{
	const framegrabber_pixfmt_t *pixfmt;
	v4l2_model_timing_t *p_SupportmodeTiming;
	int width=0,height=0;
	pixfmt=framegrabber_g_support_pixelfmt_by_fourcc(fsize->pixel_format);
	if(pixfmt==NULL)
	{
		return -EINVAL;
	}
	p_SupportmodeTiming = v4l2_model_get_support_videoformat(fsize->index);
	if(p_SupportmodeTiming == NULL)
	{
		return -EINVAL;
	}
	width = p_SupportmodeTiming->frame_size.width;
	height = p_SupportmodeTiming->frame_size.height;

	if((width ==0) || (height ==0))
	{
		return -EINVAL;
	}
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->pixel_format=pixfmt->fourcc;
	fsize->discrete.width=width;
	fsize->discrete.height=height;
	return 0;


}

static int hws_vidioc_enum_input(struct file *file, void *priv,struct v4l2_input *i)
{
	struct hws_video *videodev = video_drvdata(file);
	if (i->index != 0)
		return -EINVAL;
	i->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(i->name, "HDMI/SDI", sizeof(i->name));
	i->std = 0;	/* digital input: no analog TV standard */
	i->capabilities = 0;
	i->status = READ_ONCE(videodev->dev->m_curr_No_Video[videodev->index]) ? V4L2_IN_ST_NO_SIGNAL : 0;

	return 0;
}

static int hws_vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;	/* one input */
	return 0;
}

static int hws_vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
return i ? -EINVAL : 0;
}
static ssize_t hws_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	return -EINVAL;	/* streaming I/O only */

}

static inline struct hws_vfh_ctx *hws_ctx_from_file(struct file *file)
{
    return container_of(file->private_data, struct hws_vfh_ctx, fh);
}

/* B1 multi-consumer vb2 ops (per-file queue). Defined later in this file. */
static const struct vb2_ops hwspcie_video_multi_qops;


static int hws_open(struct file *file)
{
    struct hws_video *videodev = video_drvdata(file);
    struct hws_pcie_dev *pdx = videodev->dev;
    unsigned long flags;
    struct hws_vfh_ctx *ctx;
    struct vb2_queue *q;
    int ret;

    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL_ACCOUNT);
    if (!ctx)
        return -ENOMEM;

    ctx->video = videodev;
    INIT_LIST_HEAD(&ctx->buf_queue);
    spin_lock_init(&ctx->qlock);
    spin_lock_init(&ctx->dlock);
    mutex_init(&ctx->qmutex);
    ctx->streaming = false;

    /* v4l2 file-handle */
    v4l2_fh_init(&ctx->fh, &videodev->vdev);
    file->private_data = &ctx->fh;

	#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,18,0))
	v4l2_fh_add(&ctx->fh);
	#else
	v4l2_fh_add(&ctx->fh, file);
	#endif

    /* per-file vb2 queue */
    q = &ctx->vbq;
    memset(q, 0, sizeof(*q));
    q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    q->io_modes = VB2_MMAP | VB2_USERPTR;
    q->gfp_flags = GFP_KERNEL | __GFP_ZERO;//GFP_DMA32;
    q->drv_priv = ctx;
    q->buf_struct_size = sizeof(struct hwsvideo_buffer);
    q->ops = &hwspcie_video_multi_qops;
    q->mem_ops = &vb2_vmalloc_memops;
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    /* Per-open queue, so a per-ctx mutex serializes only this fh. vb2 drops it
     * while blocking in DQBUF (via wait_prepare before 7.0, by itself since).
     * It must not be NULL: vb2_ops_wait_prepare() unlocks q->lock. */
    q->lock = &ctx->qmutex;
    q->dev = &pdx->pdev->dev;

    ret = vb2_queue_init(q);
    if (ret) {
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,18,0))
		v4l2_fh_del(&ctx->fh);
		#else
		v4l2_fh_del(&ctx->fh, file);
		#endif
        v4l2_fh_exit(&ctx->fh);
        kfree(ctx);
        file->private_data = NULL;
        return ret;
    }
    spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
    videodev->fileindex++;
    spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);

    /* add to consumers list */
    spin_lock_irqsave(&videodev->consumers_lock, flags);
    list_add_tail(&ctx->node, &videodev->consumers);
    spin_unlock_irqrestore(&videodev->consumers_lock, flags);

    return 0;

}
static int hws_release(struct file *file)
{
    struct hws_vfh_ctx *ctx;
    struct hws_video *videodev;
    struct hws_pcie_dev *pdx;
    unsigned long flags;

    if (!file->private_data)
        return 0;

    ctx = hws_ctx_from_file(file);
    videodev = ctx->video;
    pdx = videodev->dev;
    /* release vb2 queue resources; stops streaming and waits for the worker */
    /* video_lock: S_FMT on another handle walks the consumers and looks at
     * this queue (vb2_is_busy); it must not see it half released */
    mutex_lock(&ctx->qmutex);
    mutex_lock(&videodev->video_lock);
    vb2_queue_release(&ctx->vbq);
    spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
    if (videodev->fileindex > 0)
        videodev->fileindex--;
    /* the format stays as set, but the next input change may replace it */
    if (videodev->fileindex == 0)
        videodev->fmt_by_app = false;
    spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
    mutex_unlock(&videodev->video_lock);
    mutex_unlock(&ctx->qmutex);

    /* remove from consumers */
    spin_lock_irqsave(&videodev->consumers_lock, flags);
    list_del(&ctx->node);
    spin_unlock_irqrestore(&videodev->consumers_lock, flags);

    /* v4l2 fh cleanup */
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(6,18,0))
	v4l2_fh_del(&ctx->fh);
	#else
	v4l2_fh_del(&ctx->fh, file);
	#endif
    v4l2_fh_exit(&ctx->fh);
    file->private_data = NULL;

    mutex_destroy(&ctx->qmutex);
    kfree(ctx);
    return 0;
}

/*
 * Picture controls through the V4L2 control framework (QUERYCTRL, G/S/TRY,
 * ext controls, defaults, events, ctrl classes all handled by the core).
 * Values go to the card right away; the status thread re-applies them if the
 * card ever loses them.
 */
static void hws_apply_picture(struct hws_pcie_dev *pdx, int ch)
{
	unsigned long flags;
	u32 v;

	if (pdx->m_DeviceHW_Version == 0)
		return;
	/* checked under pm_lock: hws_block_streams() passes through it before unmapping */
	mutex_lock(&pdx->pm_lock);
	if (!pdx->suspended && !READ_ONCE(pdx->removing) && !READ_ONCE(pdx->m_PciDeviceLost) &&
	    pdx->map_bar0_addr) {
		v = (pdx->m_saturation[ch] & 0xff) << 24 | (pdx->m_hue[ch] & 0xff) << 16 |
		    (pdx->m_contrast[ch] & 0xff) << 8 | (pdx->m_brightness[ch] & 0xff);
		spin_lock_irqsave(&pdx->reg_lock, flags);
		WRITE_REGISTER_ULONG(pdx, (DWORD)(CVBS_IN_BASE + ((91 + ch * 2) * PCIE_BARADDROFSIZE)), v);
		spin_unlock_irqrestore(&pdx->reg_lock, flags);
	}
	mutex_unlock(&pdx->pm_lock);
}

static int hws_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hws_video *video = container_of(ctrl->handler, struct hws_video, ctrl_handler);
	struct hws_pcie_dev *pdx = video->dev;
	int ch = video->index;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		pdx->m_brightness[ch] = ctrl->val;
		break;
	case V4L2_CID_CONTRAST:
		pdx->m_contrast[ch] = ctrl->val;
		break;
	case V4L2_CID_SATURATION:
		pdx->m_saturation[ch] = ctrl->val;
		break;
	case V4L2_CID_HUE:
		pdx->m_hue[ch] = ctrl->val;
		break;
	default:
		return -EINVAL;
	}
	hws_apply_picture(pdx, ch);
	return 0;
}

static const struct v4l2_ctrl_ops hws_ctrl_ops = {
	.s_ctrl = hws_s_ctrl,
};

static int hws_init_controls(struct hws_video *video)
{
	struct v4l2_ctrl_handler *hdl = &video->ctrl_handler;

	v4l2_ctrl_handler_init(hdl, 4);
	/* the oldest hardware (version 121 and below, the 8501 at 122) doesn't
	 * get the picture register written (SetHardWareInfo): no controls rather
	 * than ones that change nothing */
	if (video->dev->m_DeviceHW_Version == 0) {
		video->ctrl_ready = true;
		return 0;
	}
	v4l2_ctrl_new_std(hdl, &hws_ctrl_ops, V4L2_CID_BRIGHTNESS,
			  MIN_VAMP_BRIGHTNESS_UNITS, MAX_VAMP_BRIGHTNESS_UNITS, 1, BrightnessDefault);
	v4l2_ctrl_new_std(hdl, &hws_ctrl_ops, V4L2_CID_CONTRAST,
			  MIN_VAMP_CONTRAST_UNITS, MAX_VAMP_CONTRAST_UNITS, 1, ContrastDefault);
	v4l2_ctrl_new_std(hdl, &hws_ctrl_ops, V4L2_CID_SATURATION,
			  MIN_VAMP_SATURATION_UNITS, MAX_VAMP_SATURATION_UNITS, 1, SaturationDefault);
	v4l2_ctrl_new_std(hdl, &hws_ctrl_ops, V4L2_CID_HUE,
			  MIN_VAMP_HUE_UNITS, MAX_VAMP_HUE_UNITS, 1, HueDefault);
	if (hdl->error) {
		int err = hdl->error;

		v4l2_ctrl_handler_free(hdl);
		return err;
	}
	video->ctrl_ready = true;
	return 0;
}
static int hws_vidioc_enum_frameintervals(struct file *file, void *fh,
										  struct v4l2_frmivalenum *fival)
{
	struct hws_video *videodev = video_drvdata(file);

	/* one interval: whatever the input delivers */
	if (fival->index != 0)
		return -EINVAL;
	if (!framegrabber_g_support_pixelfmt_by_fourcc(fival->pixel_format))
		return -EINVAL;
	if (Get_input_framesizeIndex(fival->width, fival->height) == NULL)
		return -EINVAL;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	hws_timeperframe(videodev, &fival->discrete.numerator, &fival->discrete.denominator);
	return 0;
}
static int hws_vidioc_s_parm(struct file *file, void *fh,struct v4l2_streamparm *a)
{
	/* the rate follows the input; report what is delivered */
	return hws_vidioc_g_parm(file, fh, a);
}

/*
 * B1 multi-consumer: every open file handle owns a vb2 queue. The buffer
 * ioctls below go straight to that queue under its own lock (ctx->qmutex,
 * also q->lock), so a blocking DQBUF on one handle never holds up another
 * handle, or another thread of the same app doing QBUF/poll. vb2 drops
 * q->lock while it waits for a buffer.
 */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,0))
#define HWS_VB2_MDEV(file)	(video_devdata(file)->v4l2_dev->mdev),
#else
#define HWS_VB2_MDEV(file)
#endif

static int hws_reqbufs(struct file *file, void *priv, struct v4l2_requestbuffers *p)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	/* buffer sizing must not race S_FMT on another file handle */
	mutex_lock(&ctx->video->video_lock);
	ret = vb2_reqbufs(&ctx->vbq, p);
	mutex_unlock(&ctx->video->video_lock);
	mutex_unlock(&ctx->qmutex);
	return ret;
}

static int hws_create_bufs(struct file *file, void *priv, struct v4l2_create_buffers *p)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	mutex_lock(&ctx->video->video_lock);
	ret = vb2_create_bufs(&ctx->vbq, p);
	mutex_unlock(&ctx->video->video_lock);
	mutex_unlock(&ctx->qmutex);
	return ret;
}

/* a user buffer larger than any frame needs: refused before vb2 pins it */
static bool hws_userptr_too_big(const struct v4l2_buffer *b)
{
	return b->memory == V4L2_MEMORY_USERPTR && b->length > 2 * PAGE_ALIGN(MAX_VIDEO_SCLAER_SIZE);
}

/*
 * ...and one too small for the format: vb2 would unpin the buffer's old
 * memory on that error without buf_cleanup, and its charge would stay
 * counted for good. A buffer already prepared (PREPARE_BUF) is queued as it
 * is: vb2 ignores the pointer and length passed with it. qmutex held.
 */
static bool hws_userptr_too_short(struct vb2_queue *q, const struct v4l2_buffer *b)
{
	struct vb2_buffer *vb;

	if (b->memory != V4L2_MEMORY_USERPTR || b->type != q->type)
		return false;
	vb = vb2_get_buffer(q, b->index);
	return vb && !vb->prepared && b->length < vb->planes[0].min_length;
}

static int hws_prepare_buf(struct file *file, void *priv, struct v4l2_buffer *b)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (hws_userptr_too_big(b))
		return -EINVAL;
	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	ret = hws_userptr_too_short(&ctx->vbq, b) ? -EINVAL :
	      vb2_prepare_buf(&ctx->vbq, HWS_VB2_MDEV(file) b);
	mutex_unlock(&ctx->qmutex);
	return ret;
}

static int hws_querybuf(struct file *file, void *priv, struct v4l2_buffer *b)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	ret = vb2_querybuf(&ctx->vbq, b);
	mutex_unlock(&ctx->qmutex);
	return ret;
}

/* the controls' state in the kernel log; at most once a second per channel */
static int hws_log_status(struct file *file, void *priv)
{
	struct hws_video *videodev = video_drvdata(file);
	u64 now = get_jiffies_64();

	if (time_before64(now, READ_ONCE(videodev->log_status_next)))
		return 0;
	WRITE_ONCE(videodev->log_status_next, now + HZ);
	return v4l2_ctrl_log_status(file, priv);
}

static int hws_qbuf(struct file *file, void *priv, struct v4l2_buffer *b)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (hws_userptr_too_big(b))
		return -EINVAL;
	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	ret = hws_userptr_too_short(&ctx->vbq, b) ? -EINVAL :
	      vb2_qbuf(&ctx->vbq, HWS_VB2_MDEV(file) b);
	mutex_unlock(&ctx->qmutex);
	return ret;
}

static int hws_dqbuf(struct file *file, void *priv, struct v4l2_buffer *b)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	ret = vb2_dqbuf(&ctx->vbq, b, file->f_flags & O_NONBLOCK);
	mutex_unlock(&ctx->qmutex);
	return ret;
}

static int hws_streamon(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	ret = vb2_streamon(&ctx->vbq, type);
	mutex_unlock(&ctx->qmutex);
	return ret;
}

static int hws_streamoff(struct file *file, void *priv, enum v4l2_buf_type type)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	ret = vb2_streamoff(&ctx->vbq, type);
	mutex_unlock(&ctx->qmutex);
	return ret;
}

static __poll_t hws_poll(struct file *file, struct poll_table_struct *wait)
{
	struct hws_vfh_ctx *ctx;
	__poll_t ret;

	if (!file->private_data)
		return EPOLLERR;
	ctx = hws_ctx_from_file(file);

	/*
	 * vb2_poll() looks at a done buffer after dropping done_lock, so it must
	 * not race STREAMOFF/REQBUFS(0) on another thread: hold the queue lock.
	 * Only a fatal signal ends the wait, so an ordinary one can't turn into
	 * EPOLLERR; it is held briefly, except by a QBUF pinning user memory that
	 * may itself wait (a FUSE file): killable, so that can't hang a task.
	 */
	if (mutex_lock_killable(&ctx->qmutex))
		return EPOLLERR;
	ret = vb2_poll(&ctx->vbq, file, wait);
	mutex_unlock(&ctx->qmutex);

	return ret;
}

static int hws_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct hws_vfh_ctx *ctx;

	if (!file->private_data)
		return -EINVAL;
	ctx = hws_ctx_from_file(file);

	/* vb2 serializes mmap against buffer allocation with its own mmap_lock */
	return vb2_mmap(&ctx->vbq, vma);
}

static const struct v4l2_file_operations hws_fops = {
    .owner          = THIS_MODULE,
    .open           = hws_open,
    .release        = hws_release,
    .read           = hws_read,
    .poll           = hws_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap           = hws_mmap,
};


/* control events, and V4L2_EVENT_SOURCE_CHANGE when the input's resolution changes */
static int hws_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	if (sub->type == V4L2_EVENT_SOURCE_CHANGE)
		return v4l2_src_change_event_subscribe(fh, sub);
	return v4l2_ctrl_subscribe_event(fh, sub);
}

static const struct v4l2_ioctl_ops hws_ioctl_fops = {
	.vidioc_querycap = hws_vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = hws_vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = hws_vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = vidioc_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = hws_vidioc_try_fmt_vid_cap,
	.vidioc_reqbufs       = hws_reqbufs,
	.vidioc_prepare_buf   = hws_prepare_buf,
	.vidioc_create_bufs   = hws_create_bufs,
	.vidioc_querybuf      = hws_querybuf,
	.vidioc_qbuf          = hws_qbuf,
	.vidioc_dqbuf         = hws_dqbuf,
	.vidioc_streamon      = hws_streamon,
	.vidioc_streamoff     = hws_streamoff,
	.vidioc_enum_framesizes   	= hws_vidioc_enum_framesizes,
	.vidioc_enum_frameintervals = hws_vidioc_enum_frameintervals,
	.vidioc_subscribe_event = hws_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_enum_input = hws_vidioc_enum_input,
	.vidioc_g_input = hws_vidioc_g_input,
	.vidioc_s_input = hws_vidioc_s_input,
	.vidioc_log_status = hws_log_status,
	//.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_g_parm = hws_vidioc_g_parm,
	.vidioc_s_parm = hws_vidioc_s_parm,
};

static void hws_buffer_finish(struct vb2_buffer *vb)
{
}

/*
 * B1 multi-consumer vb2 ops: q->drv_priv is struct hws_vfh_ctx
 */
static int hws_queue_setup_multi(struct vb2_queue *q,
               unsigned int *num_buffers, unsigned int *num_planes,
               unsigned int sizes[], struct device *alloc_devs[])
{
    struct hws_vfh_ctx *ctx = q->drv_priv;
    struct hws_video *videodev = ctx->video;
    struct hws_pcie_dev *pdx = videodev->dev;
    unsigned long flags;
    unsigned int size;

    spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
    size = 2 * videodev->current_out_width * videodev->curren_out_height;
    spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);

    if (*num_planes) {
        /* single plane; larger than the biggest frame twice over is only a way to eat memory */
        if (*num_planes != 1 || sizes[0] < size || sizes[0] > 2 * PAGE_ALIGN(MAX_VIDEO_SCLAER_SIZE))
            return -EINVAL;
        return 0;
    }

    *num_planes = 1;
    sizes[0] = size;
    return 0;
}

/* mmap buffers when allocated, user buffers whenever (re)pinned */
static int hws_buffer_init_multi(struct vb2_buffer *vb)
{
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct hwsvideo_buffer *buf = container_of(vbuf, struct hwsvideo_buffer, vb);
    struct hws_vfh_ctx *ctx = vb->vb2_queue->drv_priv;
    struct hws_video *videodev = ctx->video;
    s64 size = vb2_plane_size(vb, 0);
    s64 limit = (s64)max(READ_ONCE(buf_limit_mb), 16) << 20;

    /* a charge vb2 dropped without buf_cleanup (it shouldn't, see hws_userptr_too_short) */
    atomic64_sub(buf->charged, &videodev->buf_bytes);
    buf->charged = 0;
    if (atomic64_add_return(size, &videodev->buf_bytes) > limit) {
        atomic64_sub(size, &videodev->buf_bytes);
        return -ENOMEM;
    }
    buf->charged = size;
    return 0;
}

static void hws_buffer_cleanup_multi(struct vb2_buffer *vb)
{
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct hwsvideo_buffer *buf = container_of(vbuf, struct hwsvideo_buffer, vb);
    struct hws_vfh_ctx *ctx = vb->vb2_queue->drv_priv;

    atomic64_sub(buf->charged, &ctx->video->buf_bytes);
    buf->charged = 0;
}

static int hws_buffer_prepare_multi(struct vb2_buffer *vb)
{
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct hwsvideo_buffer *buf = container_of(vbuf, struct hwsvideo_buffer, vb);
    struct hws_vfh_ctx *ctx = vb->vb2_queue->drv_priv;
    struct hws_video *videodev = ctx->video;
    struct hws_pcie_dev *pdx = videodev->dev;
    unsigned long flags;
    u32 size;

    spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
    size = 2 * videodev->current_out_width * videodev->curren_out_height;
    spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);

    if (vb2_plane_size(vb, 0) < size)
        return -EINVAL;

    buf->mem = vb2_plane_vaddr(vb, 0);
    if (!buf->mem)
        return -EINVAL;
    vb2_set_plane_payload(vb, 0, size);
    return 0;
}

static void hws_buffer_queue_multi(struct vb2_buffer *vb)
{
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct hwsvideo_buffer *buf = container_of(vbuf, struct hwsvideo_buffer, vb);
    struct hws_vfh_ctx *ctx = vb->vb2_queue->drv_priv;
    unsigned long flags;

    spin_lock_irqsave(&ctx->qlock, flags);
    list_add_tail(&buf->queue, &ctx->buf_queue);
    spin_unlock_irqrestore(&ctx->qlock, flags);
}

/* hand every buffer still queued on this fh back to vb2 */
static void hws_return_buffers(struct hws_vfh_ctx *ctx, enum vb2_buffer_state state)
{
    struct hwsvideo_buffer *buf, *tmp;
    unsigned long flags;
    LIST_HEAD(list);

    spin_lock_irqsave(&ctx->qlock, flags);
    list_splice_init(&ctx->buf_queue, &list);
    spin_unlock_irqrestore(&ctx->qlock, flags);

    list_for_each_entry_safe(buf, tmp, &list, queue) {
        list_del(&buf->queue);
        vb2_buffer_done(&buf->vb.vb2_buf, state);
    }
}

static int hws_start_streaming_multi(struct vb2_queue *q, unsigned int count)
{
    struct hws_vfh_ctx *ctx = q->drv_priv;
    struct hws_video *videodev = ctx->video;
    struct hws_pcie_dev *pdx = videodev->dev;
    unsigned long flags;

    mutex_lock(&videodev->engine_lock);
    if (pdx->removing || READ_ONCE(pdx->m_PciDeviceLost)) {
        mutex_unlock(&videodev->engine_lock);
        hws_return_buffers(ctx, VB2_BUF_STATE_QUEUED);
        return -ENODEV;
    }
    if (videodev->engine_users >= HWS_MAX_STREAMERS) {
        mutex_unlock(&videodev->engine_lock);
        hws_return_buffers(ctx, VB2_BUF_STATE_QUEUED);
        return -EBUSY;
    }
    /* Start hardware engine only once, on first streamer; resume starts it if suspended */
    mutex_lock(&pdx->pm_lock);
    if (videodev->engine_users++ == 0 && !pdx->suspended) {
        /* a worker run left over from the previous session must not race the ring reset */
        /* (a half-frame copy of that session first: it may queue the worker) */
        kthread_flush_work(&videodev->copy_work);
        kthread_flush_work(&videodev->videowork);
        StartVideoCapture(pdx, videodev->index);
    }
    mutex_unlock(&pdx->pm_lock);
    mutex_unlock(&videodev->engine_lock);

    spin_lock_irqsave(&ctx->dlock, flags);
    ctx->seq_valid = false;
    ctx->have_last = false;
    spin_unlock_irqrestore(&ctx->dlock, flags);
    spin_lock_irqsave(&videodev->consumers_lock, flags);
    ctx->streaming = true;
    spin_unlock_irqrestore(&videodev->consumers_lock, flags);
    return 0;
}

static void hws_stop_streaming_multi(struct vb2_queue *q)
{
    struct hws_vfh_ctx *ctx = q->drv_priv;
    struct hws_video *videodev = ctx->video;
    struct hws_pcie_dev *pdx = videodev->dev;
    unsigned long flags;

    spin_lock_irqsave(&videodev->consumers_lock, flags);
    ctx->streaming = false;
    spin_unlock_irqrestore(&videodev->consumers_lock, flags);

    /* the IRQ thread may hold this fh's buffers half-filled for the direct path */
    hws_direct_reclaim(pdx, videodev->index, ctx);
    /* ...or have handed complete frames with them to the fan-out worker */
    kthread_flush_work(&videodev->fanout_work);

    /*
     * The worker may be filling a buffer it already took from this fh. Wait for
     * it, otherwise vb2 frees the buffer memory while it is still being written.
     */
    kthread_flush_work(&videodev->videowork);

    mutex_lock(&videodev->engine_lock);
    mutex_lock(&pdx->pm_lock);
    /* Stop engine when the last streamer goes away (suspend already stopped the card) */
    if (videodev->engine_users > 0 && --videodev->engine_users == 0 && !pdx->removing &&
        !pdx->suspended) {
        StopVideoCapture(pdx, videodev->index);
        /* cancel, let a racing worker run finish (it may re-arm), cancel again */
        kthread_cancel_delayed_work_sync(&videodev->nosig_work);
        kthread_flush_work(&videodev->videowork);
        kthread_cancel_delayed_work_sync(&videodev->nosig_work);
    }
    mutex_unlock(&pdx->pm_lock);
    mutex_unlock(&videodev->engine_lock);

    /* Return all pending buffers for this ctx */
    hws_return_buffers(ctx, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops hwspcie_video_multi_qops = {
    .queue_setup    = hws_queue_setup_multi,
    .buf_init       = hws_buffer_init_multi,
    .buf_cleanup    = hws_buffer_cleanup_multi,
    .buf_prepare    = hws_buffer_prepare_multi,
    .buf_finish     = hws_buffer_finish,
    .buf_queue      = hws_buffer_queue_multi,
    #if (LINUX_VERSION_CODE < KERNEL_VERSION(7,0,0))
    .wait_prepare   = vb2_ops_wait_prepare,
    .wait_finish    = vb2_ops_wait_finish,
    #endif
    .start_streaming= hws_start_streaming_multi,
    .stop_streaming = hws_stop_streaming_multi,
};
/* 5x7 glyphs for the "no signal" picture: one byte per row, bit 4 = left column */
static const struct { char c; u8 row[7]; } hws_nosig_font[] = {
	{ 'A', { 0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11 } },
	{ 'G', { 0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f } },
	{ 'I', { 0x0e, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0e } },
	{ 'L', { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f } },
	{ 'N', { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 } },
	{ 'O', { 0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e } },
	{ 'P', { 0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10 } },
	{ 'S', { 0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e } },
	{ 'T', { 0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 } },
	{ 'U', { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e } },
	{ '1', { 0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e } },
	{ '2', { 0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f } },
	{ '3', { 0x1f, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0e } },
	{ '4', { 0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02 } },
	{ '5', { 0x1f, 0x10, 0x1e, 0x01, 0x01, 0x11, 0x0e } },
	{ '6', { 0x06, 0x08, 0x10, 0x1e, 0x11, 0x11, 0x0e } },
	{ '7', { 0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 } },
	{ '8', { 0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e } },
};

/*
 * One line of text, centred at row y0, each font pixel a scale x scale block
 * of white. scale and the left edge are even, so every block covers whole
 * YUYV pixel pairs and the chroma stays clean. Nothing if it doesn't fit.
 */
static void hws_nosig_text(u8 *dst, int w, int h, int y0, int scale, const char *txt)
{
	int len = strlen(txt), tw = (len * 6 - 1) * scale, x0, k, r, c, y;

	if (tw > w || y0 < 0 || y0 + 7 * scale > h)
		return;
	x0 = ((w - tw) / 2) & ~1;
	for (k = 0; k < len; k++) {
		const u8 *rows = NULL;
		int g;

		for (g = 0; g < ARRAY_SIZE(hws_nosig_font); g++)
			if (hws_nosig_font[g].c == txt[k])
				rows = hws_nosig_font[g].row;
		if (!rows)
			continue;	/* space */
		for (r = 0; r < 7; r++)
			for (c = 0; c < 5; c++) {
				int x = x0 + (k * 6 + c) * scale;

				if (!(rows[r] & (0x10 >> c)))
					continue;
				for (y = y0 + r * scale; y < y0 + (r + 1) * scale; y++) {
					u8 *p = dst + (y * w + x) * 2;
					int i;

					for (i = 0; i < scale / 2; i++, p += 4) {
						p[0] = 235; p[1] = 128; p[2] = 235; p[3] = 128;
					}
				}
			}
	}
}

/* YUYV "no signal" picture: blue, with "NO SIGNAL" and the input number */
static void SetNoVideoMem(uint8_t *pDest, int w, int h, int ch)
{
	static const char big[] = "NO SIGNAL";
	char small[12];
	static const u8 blue[4] = { 41, 240, 41, 109 };	/* Y U Y V */
	int y, scale, small_scale, ytext;

	for (y = 0; y < w / 2 * h; y++)
		memcpy(pDest + 4 * y, blue, 4);

	/* "NO SIGNAL" about half the width, at most a sixth of the height */
	scale = min((w / 2) / (int)(sizeof(big) * 6 - 7), h / 6 / 7) & ~1;
	if (scale < 2)
		return;
	small_scale = max(scale / 2 & ~1, 2);
	snprintf(small, sizeof(small), "INPUT %d", ch + 1);
	ytext = (h - 7 * scale - 4 * small_scale - 7 * small_scale) / 2;
	hws_nosig_text(pDest, w, h, ytext, scale, big);
	hws_nosig_text(pDest, w, h, ytext + 7 * scale + 4 * small_scale, small_scale, small);
}

static const u8 hws_silence[MAX_DMA_AUDIO_PK_SIZE];

/* frames into the ALSA ring at the write position; ring_lock held */
static void hws_ring_write(struct hws_audio *drv, struct snd_pcm_runtime *runtime,
			   const u8 *data, u32 frames, u32 frame_bytes)
{
	/* the packet may be larger than the whole ring: wrap as often as needed */
	while (frames) {
		u32 chunk = min(frames, drv->ring_size_byframes - drv->ring_wpos_byframes);

		memcpy(runtime->dma_area + drv->ring_wpos_byframes * frame_bytes, data, chunk * frame_bytes);
		data += chunk * frame_bytes;
		frames -= chunk;
		drv->ring_wpos_byframes = (drv->ring_wpos_byframes + chunk) % drv->ring_size_byframes;
		drv->period_used_byframes += chunk;
	}
}

/*
 * Append one captured packet to the ALSA ring. Everything that touches the
 * runtime happens under ring_lock and only while the stream runs, so a
 * concurrent STOP/close can't free dma_area underneath us.
 *
 * fill_after 0: a packet from the card (its time is noted in the same locked
 * step). Otherwise silence, written only if no real packet came since
 * last_real_ns was fill_after - checked under the lock, so a real packet can't
 * slip in between and get silence after it. Returns false if skipped for that.
 * Silence covers the time up to fill_until (the end of its packet).
 */
static bool _deliver_samples(struct hws_audio *drv, const u8 *aud_data, u32 aud_len,
			     u64 fill_after, u64 fill_until)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	u32 frame_bytes, frames;
	unsigned long flags;
	int elapsed = 0;

	spin_lock_irqsave(&drv->ring_lock, flags);
	if (!fill_after)
		atomic64_set(&drv->last_real_ns, ktime_get_ns());
	else if (atomic64_read(&drv->last_real_ns) != fill_after) {
		spin_unlock_irqrestore(&drv->ring_lock, flags);
		return false;
	}
	substream = drv->substream;
	runtime = substream ? substream->runtime : NULL;
	if (!drv->running || !runtime || !runtime->dma_area ||
	    drv->ring_size_byframes == 0 || drv->period_size_byframes == 0 ||
	    drv->channels == 0) {
		spin_unlock_irqrestore(&drv->ring_lock, flags);
		return true;
	}

	frame_bytes = 2 * drv->channels;
	frames = aud_len / frame_bytes;
	if (fill_after)
	{
		drv->silence_until = fill_until;
		drv->after_silence = true;
	}
	else if (drv->after_silence && frames)
	{
		/*
		 * The card is back: its packet started one packet before its
		 * interrupt. Silence that ran past that start is taken off the
		 * front of the packet, a gap before it is filled - otherwise each
		 * dropout would leave the audio up to a packet later or earlier.
		 */
		struct hws_pcie_dev *pdx = drv->dev;
		u64 irq = max((u64)atomic64_read(&pdx->m_audio_half_ts[drv->index][0]),
			      (u64)atomic64_read(&pdx->m_audio_half_ts[drv->index][1]));
		s64 start = irq - div_u64((u64)frames * NSEC_PER_SEC, 48000);
		s64 over = (s64)(drv->silence_until - start);
		u32 adj = min_t(u64, div_u64((u64)abs(over) * 48000, NSEC_PER_SEC), frames);

		drv->after_silence = false;
		if (over > 0)
		{
			aud_data += adj * frame_bytes;
			frames -= adj;
		}
		else
		{
			while (adj)
			{
				u32 n = min_t(u32, adj, sizeof(hws_silence) / frame_bytes);

				hws_ring_write(drv, runtime, hws_silence, n, frame_bytes);
				adj -= n;
			}
		}
	}
	hws_ring_write(drv, runtime, aud_data, frames, frame_bytes);
	/* check if a period available */
	if (drv->period_used_byframes >= drv->period_size_byframes) {
		elapsed = 1;
		drv->period_used_byframes %= drv->period_size_byframes;
	}
	spin_unlock_irqrestore(&drv->ring_lock, flags);

	/* outside ring_lock: this may call back into trigger(STOP) and pointer() */
	if (elapsed)
		snd_pcm_period_elapsed(substream);
	return true;
}

static void audio_data_process(struct work_struct *p_work)
{
	struct hws_audio *drv = container_of(p_work, struct hws_audio, audiowork);
	struct hws_pcie_dev *pdx = drv->dev;
	int dwAudioCh = drv->index;
	ACAP_AUDIO_INFO *ai = &pdx->m_AudioInfo[dwAudioCh];
	unsigned long flags;
	int n;

	/* drain every ready packet in order; one run may cover several IRQs */
	for (n = 0; n < MAX_AUDIO_QUEUE; n++) {
		int nIndex = -1;
		int i, idx;
		BYTE *bBuf;
		int aud_len;

		spin_lock_irqsave(&pdx->audiolock[dwAudioCh], flags);
		for (i = 0; i < MAX_AUDIO_QUEUE; i++) {
			idx = (pdx->m_nRDAudioIndex[dwAudioCh] + i) % MAX_AUDIO_QUEUE;
			if (ai->pStatusInfo[idx].byLock == MEM_LOCK) {
				nIndex = idx;
				break;
			}
		}
		if (nIndex < 0) {
			spin_unlock_irqrestore(&pdx->audiolock[dwAudioCh], flags);
			break;
		}
		ai->pStatusInfo[nIndex].byLock = MEM_READING;
		bBuf = ai->m_pAudioBufData[nIndex];
		aud_len = ai->pStatusInfo[nIndex].dwLength;
		pdx->m_nRDAudioIndex[dwAudioCh] = (nIndex + 1) % MAX_AUDIO_QUEUE;
		spin_unlock_irqrestore(&pdx->audiolock[dwAudioCh], flags);

		if (bBuf && aud_len > 0)
		{
			_deliver_samples(drv, bBuf, aud_len, 0, 0);
			atomic_inc(&drv->pkts_card);
		}

		spin_lock_irqsave(&pdx->audiolock[dwAudioCh], flags);
		if (ai->pStatusInfo[nIndex].byLock == MEM_READING)
			ai->pStatusInfo[nIndex].byLock = MEM_UNLOCK;
		spin_unlock_irqrestore(&pdx->audiolock[dwAudioCh], flags);
	}
}
static void FHD_To_800X600_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int x,y;
	BYTE*pSrcBuf;
	int *pSrcData;
	int *pDestData;
	pSrcBuf = pSrc;
	pDestData = (int *) pOut;
	for(y=0;y<in_h;y++)
	{
		switch(y%9)
		{
			case 0:
			case 2:
			case 4:
			case 6:
			case 8:
			{
				pSrcBuf = pSrc + (y*in_w*2)+(240*2);
				for(x=0;x<(in_w-480);)
				{
					pSrcData =(int *)pSrcBuf;
					pDestData[0] = pSrcData[0];
					pDestData[1] = pSrcData[2];
					pDestData[2] = pSrcData[4];
					pDestData[3] = pSrcData[6];
					pDestData[4] = pSrcData[8];
					pSrcBuf +=18*2;
					pDestData +=5;
					x= x+18;
				}
				break;
			}
			case 1:
			case 3:
			case 5:
			case 7:
			{
				break;
			}
		}

	}

}
static void HD_To_800X600_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	for(y=0;y<(in_h-120);y++)
	{
		memcpy(pDestBuf,(pSrcBuf+60*in_w*2),out_w*2);
		pDestBuf += out_w*2;
		pSrcBuf += in_w*2;
	}

}
static void SD_NTSC_To_SD_PAL_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	DWORD dwI;
	DWORD dwJ;
	BYTE *pDstYUV;

	pSrcBuf = pSrc;
	pDestBuf = pOut;


	// PAL 720X576  NTSC 720X480
	pDstYUV =  pDestBuf;
	for(dwI =0; dwI <48; dwI++)
	{
		for(dwJ=0; dwJ<out_w; dwJ++)
		{
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV +=2;
		}
	}
	pDestBuf += 48*in_w*2;
	memcpy(pDestBuf,pSrcBuf,in_h*in_w*2);
	pDestBuf +=in_h*in_w*2;
	pDstYUV = pDestBuf;
	for(dwI =0; dwI <48; dwI++)
	{
		for(dwJ=0; dwJ<out_w; dwJ++)
		{
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV +=2;
		}
	}

}

static void SD_PAL_To_SD_NTSC_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	// PAL 720X576  NTSC 720X480
	memcpy(pDestBuf,(pSrcBuf+48*in_w*2),out_h*out_w*2);

}
static void SD_NTSC_To_FHD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	int dwJ;
	BYTE*pDstYUV;

	pSrcBuf = pSrc;
	pDestBuf = pOut;


	// PAL 720X480   x*2 =1440,(y-36)*2= 1080

	//--- 60 line
	for(y=0;y<60;y++)
	{
		pDstYUV = pDestBuf;
		for(dwJ=0; dwJ<out_w; dwJ++)
		{
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV +=2;
		}
		pDestBuf +=out_w*2;
	}
	for(y=0;y<(out_h-120);)
	{
		pDstYUV = pDestBuf;
		for(dwJ=0; dwJ<240; dwJ++)
		{
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV +=2;
		}
		pDestBuf +=240*2;
		for(x=0;x<720;)
		{

			pDestBuf[0] = pSrcBuf[0];//y
			pDestBuf[1] = pSrcBuf[1];//cb

			pDestBuf[2] = pSrcBuf[0];//y
			pDestBuf[3] = pSrcBuf[3];//cr

			pDestBuf[4] = pSrcBuf[2];//y
			pDestBuf[5] = pSrcBuf[1];//cb

			pDestBuf[6] = pSrcBuf[2];//y
			pDestBuf[7] = pSrcBuf[3];//cr

			pSrcBuf +=4;
			pDestBuf +=8;
			x= x+2;
		}
		pDstYUV = pDestBuf;
		for(dwJ=0; dwJ<240; dwJ++)
		{
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV +=2;
		}
		pDestBuf +=240*2;
		//------------copy line
		memcpy(pDestBuf,pDestBuf-(out_w*2),out_w*2);
		pDestBuf += out_w*2;
		y = y+2;
	}
	//-- 60 line
	for(y=0;y<60;y++)
	{
		pDstYUV = pDestBuf;
		for(dwJ=0; dwJ<out_w; dwJ++)
		{
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV +=2;
		}
		pDestBuf +=out_w*2;

	}
}

static void SD_PAL_To_FHD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	int dwJ;
	BYTE*pDstYUV;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	// PAL 720X576   x*2 =1440,(y-36)*2= 1080
	pSrcBuf = pSrcBuf+18*in_w*2;
	for(y=0;y<out_h;)
	{
		pDstYUV = pDestBuf;
		for(dwJ=0; dwJ<240; dwJ++)
		{
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV +=2;
		}
		pDestBuf +=240*2;
		for(x=0;x<720;)
		{

			pDestBuf[0] = pSrcBuf[0];//y
			pDestBuf[1] = pSrcBuf[1];//cb

			pDestBuf[2] = pSrcBuf[0];//y
			pDestBuf[3] = pSrcBuf[3];//cr

			pDestBuf[4] = pSrcBuf[2];//y
			pDestBuf[5] = pSrcBuf[1];//cb

			pDestBuf[6] = pSrcBuf[2];//y
			pDestBuf[7] = pSrcBuf[3];//cr

			pSrcBuf +=4;
			pDestBuf +=8;
			x= x+2;
		}
		pDstYUV = pDestBuf;
		for(dwJ=0; dwJ<240; dwJ++)
		{
			pDstYUV[0] = 0x10;
			pDstYUV[1] = 0x80;
			pDstYUV +=2;
		}
		pDestBuf +=240*2;
		//------------copy line
		memcpy(pDestBuf,pDestBuf-(out_w*2),out_w*2);
		pDestBuf += out_w*2;
		y = y+2;
	}


}

static void SD_PAL_To_HD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;


	// PAL 720X576   640X360-> 1280*720
	pSrcBuf = pSrcBuf+108*in_w*2;
	for(y=0;y<out_h;)
	{
		pSrcBuf += 40*2;
		for(x=0;x<640;)
		{

			pDestBuf[0] = pSrcBuf[0];//y
			pDestBuf[1] = pSrcBuf[1];//cb

			pDestBuf[2] = pSrcBuf[0];//y
			pDestBuf[3] = pSrcBuf[3];//cr

			pDestBuf[4] = pSrcBuf[2];//y
			pDestBuf[5] = pSrcBuf[1];//cb

			pDestBuf[6] = pSrcBuf[2];//y
			pDestBuf[7] = pSrcBuf[3];//cr

			pSrcBuf +=4;
			pDestBuf +=8;
			x= x+2;
		}
		pSrcBuf += 40*2;
		//------------copy line
		memcpy(pDestBuf,pDestBuf-(out_w*2),out_w*2);
		pDestBuf += out_w*2;
		y= y+2;
	}


}
static void V1280X1024_To_FHD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{

	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	pSrcBuf = pSrcBuf+152*in_w*2;
	HD_To_FHD_Scaler(pSrcBuf,pDestBuf,1280,720,out_w,out_h);
}
void V1280X1024_To_HD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	pSrcBuf = pSrcBuf+152*in_w*2;
	memcpy(pDestBuf,pSrcBuf,out_h*out_w*2);

}
void V1280X1024_To_800X600_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	pSrcBuf = pSrcBuf+152*in_w*2;
	HD_To_800X600_Scaler(pSrcBuf,pDestBuf,1280,720,out_w,out_h);


}

void SD_NTSC_To_HD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	// PAL 720X480   640X360-> 1280*720
	pSrcBuf = pSrcBuf+60*in_w*2;
	for(y=0;y<out_h;)
	{
		pSrcBuf += 40*2;
		for(x=0;x<640;)
		{

			pDestBuf[0] = pSrcBuf[0];//y
			pDestBuf[1] = pSrcBuf[1];//cb

			pDestBuf[2] = pSrcBuf[0];//y
			pDestBuf[3] = pSrcBuf[3];//cr

			pDestBuf[4] = pSrcBuf[2];//y
			pDestBuf[5] = pSrcBuf[1];//cb

			pDestBuf[6] = pSrcBuf[2];//y
			pDestBuf[7] = pSrcBuf[3];//cr

			pSrcBuf +=4;
			pDestBuf +=8;
			x= x+2;
		}
		pSrcBuf += 40*2;
		//------------copy line
		memcpy(pDestBuf,pDestBuf-(out_w*2),out_w*2);
		pDestBuf += out_w*2;
		y= y+2;
	}


}

void FHD_To_HD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)

{
	int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	int *pDestData;
	pSrcBuf = pSrc;
	pDestData = (int *) pOut;
	for(y=0;y<in_h;y++)
	{
		if(y%3 != 2)
		{
			for(x=0;x<in_w;)
			{
				/* source steps 6 bytes, so every other read is unaligned */
				memcpy(pDestData, pSrcBuf, 4);
				if(x%2==1)
				{
					pDestBuf =(BYTE *)pDestData;
					pDestBuf[1] = pSrcBuf[3];
					pDestBuf [3] = pSrcBuf[1];
				}
				pDestData +=1;
				pSrcBuf +=6;
				x= x+3;
			}
		}
		else
		{
			pSrcBuf += in_w*2;
		}

	}

}
void HD_To_FHD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{

	int x,y;
	BYTE*pSrcBuf;
	DWORD *pSrcData;
	DWORD *pDestData;
	int    out_w_size;
	BYTE*pDestTmp;
	out_w_size = out_w*2;
	pDestData = (DWORD *) pOut;
	pSrcData = (DWORD *) pSrc;
	for(y=0;y<out_h;y++)
	{

		if((y%3)==2)
		{
			pSrcBuf = (BYTE*)pDestData;
			memcpy(pSrcBuf,(BYTE*)(pSrcBuf-out_w_size),out_w_size);
			pDestData += out_w_size/4;
		}
		else
		{

			for(x=0;x<out_w;)
			{
				*pDestData = *pSrcData;
				pDestData ++;
				pSrcData++;

				*pDestData = *pSrcData;
				pDestTmp =(BYTE*)pDestData;
				pDestTmp[2] = pDestTmp[0];
				pDestData ++;

				*pDestData = *pSrcData;
				pDestTmp =(BYTE*)pDestData;
				pDestTmp[0] = pDestTmp[2];
				pSrcData++;
				pDestData++;
				x= x+6;
			}

		}


	}


}
void FHD_To_SD_NTSC_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	BYTE *pSrcData;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//-- 1920X1080 0->720X480
	pSrcBuf = pSrcBuf + 60*in_w*2;
	for(y=0;y<out_h;)
	{
		pSrcData = pSrcBuf + 240*2;
		for(x =0;x<720;)
		{
			pDestBuf[0] = pSrcData[0];//y
			pDestBuf[1] = pSrcData[1];//cb

			pDestBuf[2] = pSrcData[4];//y
			pDestBuf[3] = pSrcData[3];//cr

			x = x+ 2;
			pSrcData +=8;
			pDestBuf +=4;
		}
		pSrcBuf += in_w*4;
		y= y+1;
	}

}
void FHD_To_SD_PAL_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	BYTE *pSrcData;
	BYTE*pDstYUV;
	int dwJ;
	pSrcBuf = pSrc;
	pDestBuf = pOut;

	//-- 1920X1080 0->720X576
	pDstYUV = pDestBuf;
	for(dwJ=0; dwJ<18*out_w; dwJ++)
	{
		pDstYUV[0] = 0x10;
		pDstYUV[1] = 0x80;
		pDstYUV +=2;
	}
	pDestBuf += 18*out_w*2;
	for(y=0;y<540;)
	{
		pSrcData = pSrcBuf + 240*2;
		for(x =0;x<720;)
		{
			pDestBuf[0] = pSrcData[0];//y
			pDestBuf[1] = pSrcData[1];//cb

			pDestBuf[2] = pSrcData[4];//y
			pDestBuf[3] = pSrcData[3];//cr

			x = x+ 2;
			pSrcData +=8;
			pDestBuf +=4;
		}
		pSrcBuf += in_w*4;
		y= y+1;
	}
	pDstYUV = pDestBuf;
	for(dwJ=0; dwJ<18*out_w; dwJ++)
	{
		pDstYUV[0] = 0x10;
		pDstYUV[1] = 0x80;
		pDstYUV +=2;
	}
}

void HD_To_SD_NTSC_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//-- 1280X720 ->720X480
	pSrcBuf = pSrcBuf + 120*in_w*2;
	for(y=0;y<out_h;)
	{
		memcpy(pDestBuf,pSrcBuf+280*2,out_w*2);
		pSrcBuf += in_w*2;
		pDestBuf += out_w*2;
		y= y+1;
	}

}
void HD_To_SD_PAL_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//-- 1920X1080 0->720X576
	pSrcBuf = pSrcBuf + 72*in_w*2;
	for(y=0;y<out_h;)
	{
		memcpy(pDestBuf,pSrcBuf+280*2,out_w*2);
		pSrcBuf += in_w*2;
		pDestBuf += out_w*2;
		y= y+1;
	}
}
void V1280X1024_NTSC_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//-- 1280X1024 0->720X480
	pSrcBuf = pSrcBuf + 272*in_w*2;
	for(y=0;y<out_h;)
	{
		memcpy(pDestBuf,pSrcBuf+280*2,out_w*2);
		pSrcBuf += in_w*2;
		pDestBuf += out_w*2;
		y= y+1;
	}
}

void V1280X1024_PAL_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//-- 1280x1024 0->720X576
	pSrcBuf = pSrcBuf + 224*in_w*2;
	for(y=0;y<out_h;)
	{
		memcpy(pDestBuf,pSrcBuf+280*2,out_w*2);
		pSrcBuf += in_w*2;
		pDestBuf += out_w*2;
		y= y+1;
	}
}
static void VideoRotate90deg(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)

{
	int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	BYTE *pSrcData;
	BYTE *pDestData;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	for(y=0;y<in_h;y++)
	{
		pSrcData = pSrcBuf;
		pDestData = pDestBuf;
		for(x=0;x<in_w/2;x++)
		{
			if((y%2) ==0)
			{
				pDestData[0]= pSrcData[0];
				pDestData[1]= pSrcData[1];
				pDestData[out_w*2+0]= pSrcData[2];
				pDestData[out_w*2+1]= pSrcData[1];
			}
			else
			{
				pDestData[0]= pSrcData[0];
				pDestData[1]= pSrcData[3];
				pDestData[out_w*2+0]= pSrcData[2];
				pDestData[out_w*2+1]= pSrcData[3];
			}
			pDestData +=out_w*4;
			pSrcData +=4;

		}
		pSrcBuf += in_w*2;
		pDestBuf += 2;

	}


}

static void FillBlackYUYV(BYTE *pDst, int pixels)
{
	/* Y=0x10 U/V=0x80; 8 bytes per store (byte order kept on any endianness) */
	static const BYTE black[8] = { 0x10, 0x80, 0x10, 0x80, 0x10, 0x80, 0x10, 0x80 };
	int i;

	for (i = 0; i + 4 <= pixels; i += 4, pDst += 8)
		memcpy(pDst, black, 8);
	for (; i < pixels; i++, pDst += 2)
	{
		pDst[0] = 0x10;
		pDst[1] = 0x80;
	}
}

/*
 * Generic fallback: center the input in the output, cropping or padding with
 * black. Offsets are even so Y-U-Y-V pairs stay intact, and the copied area
 * never extends past the source, so the padding stays black.
 */
/* a + (b - a) * w / 256 */
static inline int hws_lerp(int a, int b, int w)
{
	return a + (((b - a) * w) >> 8);
}

/* one sample's two neighbours in a source row: c0 and c0 + 1, weight of c0 + 1 */
struct hws_scale_col {
	u16 c0;
	u8 w;
};

/* All_VideoScaler's per-channel column tables and two-row cache */
struct hws_scaler {
	struct hws_scale_col luma[MAX_VIDEO_HW_W];
	struct hws_scale_col chroma[MAX_VIDEO_HW_W / 2];
	BYTE row[2][MAX_VIDEO_HW_W * 2];
};

static inline u32 hws_ld_le32(const BYTE *p)
{
	__le32 v;

	memcpy(&v, p, 4);
	return le32_to_cpu(v);
}

static inline u64 hws_ld_le64(const BYTE *p)
{
	__le64 v;

	memcpy(&v, p, 8);
	return le64_to_cpu(v);
}

/*
 * One source row scaled horizontally (YUYV). Where both neighbours of every
 * sample lie inside the row (x < fast), one 32-bit load fetches a luma pair
 * and one 64-bit load both chroma pairs; a + (b - a) * w / 256 is computed as
 * (a * (256 - w) + b * w) / 256, U and V in 16-bit lanes of one product. The
 * last columns, whose right neighbour is clamped to the row's end, go byte by
 * byte: no load ever reaches past the row.
 */
static void hws_scale_hrow(const struct hws_scaler *sc, BYTE *o, const BYTE *a,
			   int out_w, int fast, int cw)
{
	int x;

	for (x = 0; x < fast; x += 2, o += 4)
	{
		const struct hws_scale_col *l0 = &sc->luma[x], *l1 = &sc->luma[x + 1], *c = &sc->chroma[x / 2];
		u32 y0 = hws_ld_le32(a + 2 * l0->c0), y1 = hws_ld_le32(a + 2 * l1->c0);
		u64 q = hws_ld_le64(a + 4 * c->c0);
		u32 p0 = ((q >> 8) & 0xFF) | ((q >> 24) & 0xFF) << 16;		/* U, V of pair c0 */
		u32 p1 = ((q >> 40) & 0xFF) | ((q >> 56) & 0xFF) << 16;		/* and of c0 + 1 */
		u32 uv = ((p0 * (256 - c->w) + p1 * c->w) >> 8) & 0x00FF00FF;

		o[0] = ((y0 & 0xFF) * (256 - l0->w) + ((y0 >> 16) & 0xFF) * l0->w) >> 8;
		o[1] = uv;
		o[2] = ((y1 & 0xFF) * (256 - l1->w) + ((y1 >> 16) & 0xFF) * l1->w) >> 8;
		o[3] = uv >> 16;
	}
	for (; x < out_w; x += 2, o += 4)
	{
		const struct hws_scale_col *l0 = &sc->luma[x], *l1 = &sc->luma[x + 1], *c = &sc->chroma[x / 2];
		int r0 = min(l0->c0 + 1, cw - 1), r1 = min(l1->c0 + 1, cw - 1), rc = min(c->c0 + 1, cw / 2 - 1);

		o[0] = hws_lerp(a[2 * l0->c0], a[2 * r0], l0->w);
		o[1] = hws_lerp(a[4 * c->c0 + 1], a[4 * rc + 1], c->w);
		o[2] = hws_lerp(a[2 * l1->c0], a[2 * r1], l1->w);
		o[3] = hws_lerp(a[4 * c->c0 + 3], a[4 * rc + 3], c->w);
	}
}

/* two horizontally scaled rows blended into an output row, 4 bytes per 64-bit step */
static void hws_scale_vmix(BYTE *d, const BYTE *a, const BYTE *b, int n, int wy)
{
	const u64 m = 0x00FF00FF00FF00FFULL;
	u64 wa = 256 - wy, wb = wy;
	int i;

	if (!wy)
	{
		memcpy(d, a, n);
		return;
	}
	for (i = 0; i + 8 <= n; i += 8)
	{
		u64 x, y;

		memcpy(&x, a + i, 8);
		memcpy(&y, b + i, 8);
		/* every byte in its own 16-bit lane: at most 255 * 256, no carry out */
		x = ((((x & m) * wa + (y & m) * wb) >> 8) & m) |
		    ((((x >> 8) & m) * wa + ((y >> 8) & m) * wb) & ~m);
		memcpy(d + i, &x, 8);
	}
	for (; i < n; i++)
		d[i] = hws_lerp(a[i], b[i], wy);
}

/*
 * Any size to any size, for the pairs without a dedicated scaler. Like those
 * (e.g. 1080p -> 800x600), the centre of the picture with the output's aspect
 * ratio is scaled to fill the output: no distortion, no black bars. Bilinear
 * in YUYV, 16.16 fixed point, sample centres aligned; integer only.
 *
 * Horizontal first, then vertical - the same arithmetic the per-pixel loop
 * below does, byte for byte - so each source row's horizontal result is
 * computed once and reused by the output rows that need it (with sc, the
 * channel's tables; without, the per-pixel loop: ~2.5x slower).
 */
static void All_VideoScaler(struct hws_scaler *sc, BYTE *pSrc, BYTE *pOut, int in_w, int in_h, int out_w, int out_h)
{
	int cw = in_w, chh = in_h, cx, cy, x, y;
	int pitch = in_w * 2;
	int stepx, stepc, stepy, posy;

	if (in_w < 2 || in_h < 1 || out_w < 2 || out_h < 1)
		return;
	/* source region with the output's aspect ratio */
	if ((s64)in_w * out_h > (s64)in_h * out_w)
		cw = (int)div_s64((s64)in_h * out_w, out_h);
	else
		chh = (int)div_s64((s64)in_w * out_h, out_w);
	cw = max(cw & ~1, 2);
	chh = max(chh, 1);
	cx = ((in_w - cw) / 2) & ~1;	/* even: YUYV pairs share their chroma */
	cy = (in_h - chh) / 2;

	stepx = (int)div_s64((s64)cw << 16, out_w);		/* luma, per output pixel */
	stepc = (int)div_s64((s64)(cw / 2) << 16, out_w / 2);	/* chroma, per output pair */
	stepy = (int)div_s64((s64)chh << 16, out_h);
	posy = stepy / 2 - 32768;

	if (sc && out_w <= MAX_VIDEO_HW_W && !(out_w & 1))
	{
		int posx = stepx / 2 - 32768, posc = stepc / 2 - 32768, fast = 0;
		int have[2] = { -1, -1 };	/* source row in each slot of sc->row */

		for (x = 0; x < out_w; x++, posx += stepx)
		{
			int px = max(posx, 0);

			sc->luma[x].c0 = px >> 16;
			sc->luma[x].w = (px >> 8) & 255;
		}
		for (x = 0; x < out_w / 2; x++, posc += stepc)
		{
			int px = max(posc, 0);

			sc->chroma[x].c0 = px >> 16;
			sc->chroma[x].w = (px >> 8) & 255;
		}
		/* positions only grow: the columns with both neighbours in the row come first */
		while (fast < out_w && sc->luma[fast].c0 <= cw - 2 && sc->luma[fast + 1].c0 <= cw - 2 &&
		       sc->chroma[fast / 2].c0 <= cw / 2 - 2)
			fast += 2;

		for (y = 0; y < out_h; y++, posy += stepy)
		{
			int py = max(posy, 0), r0 = py >> 16, wy = (py >> 8) & 255;
			int r1 = min(r0 + 1, chh - 1), s0, s1;

			s0 = have[0] == r0 ? 0 : have[1] == r0 ? 1 : -1;
			if (s0 < 0)
			{
				s0 = have[0] == r1 ? 1 : 0;	/* keep r1 if it is there */
				hws_scale_hrow(sc, sc->row[s0], pSrc + (size_t)(cy + r0) * pitch + cx * 2, out_w, fast, cw);
				have[s0] = r0;
			}
			s1 = s0;
			if (wy)
			{
				s1 = have[!s0] == r1 ? !s0 : have[s0] == r1 ? s0 : -1;
				if (s1 < 0)
				{
					s1 = !s0;
					hws_scale_hrow(sc, sc->row[s1], pSrc + (size_t)(cy + r1) * pitch + cx * 2, out_w, fast, cw);
					have[s1] = r1;
				}
			}
			hws_scale_vmix(pOut + (size_t)y * out_w * 2, sc->row[s0], sc->row[s1], out_w * 2, wy);
		}
		return;
	}

	for (y = 0; y < out_h; y++, posy += stepy)
	{
		int py = max(posy, 0), r0 = py >> 16, wy = (py >> 8) & 255;
		int r1 = min(r0 + 1, chh - 1);
		const BYTE *a = pSrc + (size_t)(cy + r0) * pitch + cx * 2;
		const BYTE *b = pSrc + (size_t)(cy + r1) * pitch + cx * 2;
		BYTE *d = pOut + (size_t)y * out_w * 2;
		int posx = stepx / 2 - 32768, posc = stepc / 2 - 32768;

		for (x = 0; x < out_w; x += 2, d += 4)
		{
			int k, px, c0, c1, wx, v[2];

			for (k = 0; k < 2; k++, posx += stepx)
			{
				px = max(posx, 0);
				c0 = px >> 16;
				c1 = min(c0 + 1, cw - 1);
				wx = (px >> 8) & 255;
				v[k] = hws_lerp(hws_lerp(a[2 * c0], a[2 * c1], wx),
						hws_lerp(b[2 * c0], b[2 * c1], wx), wy);
			}
			px = max(posc, 0);
			c0 = px >> 16;
			c1 = min(c0 + 1, cw / 2 - 1);
			wx = (px >> 8) & 255;
			posc += stepc;
			d[0] = v[0];
			d[1] = hws_lerp(hws_lerp(a[4 * c0 + 1], a[4 * c1 + 1], wx),
					hws_lerp(b[4 * c0 + 1], b[4 * c1 + 1], wx), wy);
			d[2] = v[1];
			d[3] = hws_lerp(hws_lerp(a[4 * c0 + 3], a[4 * c1 + 3], wx),
					hws_lerp(b[4 * c0 + 3], b[4 * c1 + 3], wx), wy);
		}
	}
}

static void VideoScaler(struct hws_scaler *sc, BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	if((in_w == 1920)&&(in_h==1080)&&(out_w==1280)&&(out_h==720))
	{
		FHD_To_HD_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1920)&&(in_h==1080)&&(out_w==800)&&(out_h==600))
	{
		FHD_To_800X600_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1280)&&(in_h==720)&&(out_w==1920)&&(out_h==1080))
	{
		HD_To_FHD_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1280)&&(in_h==720)&&(out_w==800)&&(out_h==600))
	{
		HD_To_800X600_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 720)&&(in_h==576)&&(out_w==720)&&(out_h==480))
	{
		SD_PAL_To_SD_NTSC_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 720)&&(in_h==480)&&(out_w==720)&&(out_h==576))
	{
		SD_NTSC_To_SD_PAL_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 720)&&(in_h==576)&&(out_w==1920)&&(out_h==1080))
	{
		SD_PAL_To_FHD_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 720)&&(in_h==480)&&(out_w==1920)&&(out_h==1080))
	{
		SD_NTSC_To_FHD_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 720)&&(in_h==576)&&(out_w==1280)&&(out_h==720))
	{
		SD_PAL_To_HD_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 720)&&(in_h==480)&&(out_w==1280)&&(out_h==720))
	{
		SD_NTSC_To_HD_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1280)&&(in_h==1024)&&(out_w==1920)&&(out_h==1080))
	{
		V1280X1024_To_FHD_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1280)&&(in_h==1024)&&(out_w==1280)&&(out_h==720))
	{
		V1280X1024_To_HD_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1280)&&(in_h==1024)&&(out_w==800)&&(out_h==600))
	{
		V1280X1024_To_800X600_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1920)&&(in_h==1080)&&(out_w==720)&&(out_h==480))
	{
		FHD_To_SD_NTSC_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1920)&&(in_h==1080)&&(out_w==720)&&(out_h==576))
	{
		FHD_To_SD_PAL_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1280)&&(in_h==720)&&(out_w==720)&&(out_h==480))
	{
		HD_To_SD_NTSC_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1280)&&(in_h==720)&&(out_w==720)&&(out_h==576))
	{
		HD_To_SD_PAL_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1280)&&(in_h==1024)&&(out_w==720)&&(out_h==480))
	{
		V1280X1024_NTSC_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else if((in_w == 1280)&&(in_h==1024)&&(out_w==720)&&(out_h==576))
	{
		V1280X1024_PAL_Scaler(pSrc,pOut,in_w,in_h,out_w,out_h);
	}
	else
	{
		All_VideoScaler(sc, pSrc,pOut,in_w,in_h,out_w,out_h);
	}
}
/*
 * One 24-byte packet of the card's 4:2:0 layout -> 16 YUYV pixels (32 bytes).
 * Luma: s[0..3], s[8..15], s[20..23]. Chroma for odd byte positions 1, 5, ...
 * comes from a, for 3, 7, ... from b, at offsets 4..7 and 16..19. Built as
 * four 64-bit words instead of 32 byte stores: the same bytes as the original
 * per-byte code, on any endianness.
 */
static inline u64 hws_yuyv8(const BYTE *y, const BYTE *a, const BYTE *b)
{
	return (u64)y[0] | (u64)a[0] << 8 | (u64)y[1] << 16 | (u64)b[0] << 24 |
	       (u64)y[2] << 32 | (u64)a[1] << 40 | (u64)y[3] << 48 | (u64)b[1] << 56;
}

static inline void hws_yuyv_packet(BYTE *d, const BYTE *s, const BYTE *a, const BYTE *b)
{
	__le64 q[4];

	q[0] = cpu_to_le64(hws_yuyv8(s + 0,  a + 4,  b + 4));
	q[1] = cpu_to_le64(hws_yuyv8(s + 8,  a + 6,  b + 6));
	q[2] = cpu_to_le64(hws_yuyv8(s + 12, a + 16, b + 16));
	q[3] = cpu_to_le64(hws_yuyv8(s + 20, a + 18, b + 18));
	memcpy(d, q, sizeof(q));
}

/* one line of packets; a/b advance with the luma (they are this line or the next) */
static inline void hws_yuyv_line(BYTE *d, const BYTE *s, const BYTE *a, const BYTE *b, DWORD packets)
{
	DWORD j;

	for (j = 0; j < packets; j++, d += 32, s += 24, a += 24, b += 24)
		hws_yuyv_packet(d, s, a, b);
}

static inline u64 hws_le64_load(const BYTE *p)
{
	__le64 v;

	memcpy(&v, p, sizeof(v));
	return le64_to_cpu(v);
}

static inline void hws_le64_store(BYTE *p, u64 v)
{
	__le64 q = cpu_to_le64(v);

	memcpy(p, &q, sizeof(q));
}

/* 4 YUYV pixels: chroma (odd bytes) from the line above, luma y[0..3] */
static inline u64 hws_ymerge(const BYTE *up, const BYTE *y)
{
	return (hws_le64_load(up) & 0xFF00FF00FF00FF00ULL) |
	       (u64)y[0] | (u64)y[1] << 16 | (u64)y[2] << 32 | (u64)y[3] << 48;
}

/* d = the line above (up) with its luma replaced by nw bytes of y */
static void hws_ymerge_line(BYTE *d, const BYTE *up, const BYTE *y, int nw)
{
	int i;

	for (i = 0; i + 4 <= nw; i += 4)
		hws_le64_store(d + 2 * i, hws_ymerge(up + 2 * i, y + i));
	for (; i < nw; i++) {
		d[2 * i] = y[i];
		d[2 * i + 1] = up[2 * i + 1];
	}
}

/* same for a line of 24-byte packets (luma at 0..3, 8..15, 20..23) */
static void hws_ymerge_packets(BYTE *d, const BYTE *up, const BYTE *s, int packets, int nw)
{
	int j;

	for (j = 0; j < packets; j++) {
		hws_le64_store(d,      hws_ymerge(up,      s));
		hws_le64_store(d + 8,  hws_ymerge(up + 8,  s + 8));
		hws_le64_store(d + 16, hws_ymerge(up + 16, s + 12));
		hws_le64_store(d + 24, hws_ymerge(up + 24, s + 20));
		d += 32;
		up += 32;
		s += 24;
	}
	if (nw * 2 > packets * 32)	/* rest of the line: unchanged copy of the line above */
		memcpy(d, up, nw * 2 - packets * 32);
}

static void FillNV12ToYUY2(BYTE *pSrc,BYTE *pOut, int nw,int nh,int interlace)
{
	BYTE *pVideoMem = pOut ;// g_pD1TempBuffer;
	DWORD dwSrcPitch;
	DWORD dwPacket;
	BYTE *pSrc1;
	BYTE *pDstYUV = (BYTE *)pVideoMem;
	BYTE *pNextLineSrc1;
	BYTE bLineSel=0;
	DWORD dwI;
	dwSrcPitch = nw*12/8;
	dwPacket = dwSrcPitch/24;

	pSrc1  = (BYTE *)pSrc;
	pDstYUV = (BYTE *)pVideoMem;

	if(interlace ==0)
	{
		for(dwI=0; dwI <nh; dwI++)
		{
			switch(bLineSel)
			{
				case 0:
				{
					pNextLineSrc1 = (BYTE *)pSrc1+dwSrcPitch+nw; //  line2
					/* 16 pixels per packet; chroma from this line and line 2 */
					hws_yuyv_line(pDstYUV, pSrc1, pSrc1, pNextLineSrc1, dwPacket);
					pSrc1 += dwPacket*24;
					pNextLineSrc1 += dwPacket*24;
					pDstYUV += dwPacket*32;
					break;
				}
				case 1:
				{
					/* chroma of the line above + this line's luma, one pass */
					hws_ymerge_line(pDstYUV, pDstYUV-(nw*2), pSrc1, nw);
					pDstYUV += nw*2;
					pSrc1 += nw;

					break;
				}
				case 2:
				{
					/* chroma of the line above + this line's luma packets, one pass */
					hws_ymerge_packets(pDstYUV, pDstYUV-(nw*2), pSrc1, dwPacket, nw);
					pDstYUV += dwPacket*32;
					pSrc1 += dwPacket*24;
					break;
				}
				case 3:
				{
					/* chroma of the line above + this line's luma, one pass */
					hws_ymerge_line(pDstYUV, pDstYUV-(nw*2), pSrc1, nw);
					pDstYUV += nw*2;
					pSrc1 += nw;
					break;
				}

			}
			bLineSel ++;
			if(bLineSel>=4)
			{
				bLineSel =0;
			}
		}
	}
	else
	{
		nh= nh/2;
		for( dwI=0; dwI <nh; dwI++)
		{
			switch(bLineSel)
			{
				case 0:
				{
					pNextLineSrc1 = (BYTE *)pSrc1+dwSrcPitch+nw; //  line2
					/* 16 pixels per packet; chroma from this line and line 2 */
					hws_yuyv_line(pDstYUV, pSrc1, pSrc1, pNextLineSrc1, dwPacket);
					pSrc1 += dwPacket*24;
					pNextLineSrc1 += dwPacket*24;
					pDstYUV += dwPacket*32;
					memcpy(pDstYUV,pDstYUV-(nw*2),nw*2); // copy up line
					pDstYUV = pDstYUV+ nw*2;
					break;
				}
				case 1:
				{
					/* chroma of the line above + this line's luma, one pass */
					hws_ymerge_line(pDstYUV, pDstYUV-(nw*2), pSrc1, nw);
					pDstYUV += nw*2;
					pSrc1 += nw;
					memcpy(pDstYUV,pDstYUV-(nw*2),nw*2); // copy up line
					pDstYUV = pDstYUV+ nw*2;
					break;
				}
				case 2:
				{
					/* chroma of the line above + this line's luma packets, one pass */
					hws_ymerge_packets(pDstYUV, pDstYUV-(nw*2), pSrc1, dwPacket, nw);
					pDstYUV += dwPacket*32;
					pSrc1 += dwPacket*24;
					memcpy(pDstYUV,pDstYUV-(nw*2),nw*2); // copy up line
					pDstYUV = pDstYUV+ nw*2;
					break;
				}
				case 3:
				{
					/* chroma of the line above + this line's luma, one pass */
					hws_ymerge_line(pDstYUV, pDstYUV-(nw*2), pSrc1, nw);
					pDstYUV += nw*2;
					pSrc1 += nw;
					memcpy(pDstYUV,pDstYUV-(nw*2),nw*2); // copy up line
					pDstYUV = pDstYUV+ nw*2;
					break;
				}

			}
			bLineSel ++;
			if(bLineSel>=4)
			{
				bLineSel =0;
			}
		}
	}
}
static void SetDeInterlace(BYTE *pSrc,BYTE *pOut, int nw,int nh,int interlace)
{
	int dwSrcPitch = nw*2;
	int h;
	if(interlace==0)
	{
		memcpy(pOut,pSrc,nw*nh*2);
	}
	else
	{
		/* nh is the frame height (2x the field); the source holds nh/2 field lines */
		nh = nh/2;
		for( h=0; h <nh; h++)
		{
			memcpy(pOut,pSrc,dwSrcPitch);
			pOut += dwSrcPitch;
			memcpy(pOut,pSrc,dwSrcPitch);
			pSrc += dwSrcPitch;
			pOut += dwSrcPitch;
		}
	}

}
static void FillYUU2(BYTE *pSrc,BYTE *pOut, int nw,int nh,int interlace)
{
	BYTE *pVideoMem = pOut ;// g_pD1TempBuffer;
	DWORD dwSrcPitch;
	DWORD dwI;
	DWORD dwPacket;
	BYTE *pSrc1  = (BYTE *)pSrc;
	BYTE *pDstYUV = (BYTE *)pVideoMem;
	BYTE *pNextLineSrc1;
	BOOL bUVSel;

	dwSrcPitch = nw*12/8;
	dwPacket = dwSrcPitch/24;
	pNextLineSrc1 = (BYTE *)pSrc1+dwSrcPitch;
	if(interlace ==0)
	{
		for( dwI=0; dwI <nh; dwI++)
		{
			/* chroma for the odd bytes alternates between this line and the next */
			bUVSel = (dwI&0x1);
			hws_yuyv_line(pDstYUV, pSrc1, bUVSel ? pNextLineSrc1 : pSrc1,
				      bUVSel ? pSrc1 : pNextLineSrc1, dwPacket);
			pSrc1 += dwPacket*24;
			pNextLineSrc1 += dwPacket*24;
			pDstYUV += dwPacket*32;
		}
		/* the last line has no next line for its chroma: black */
		FillBlackYUYV(pDstYUV - nw*2, nw);
	}
	else
	{
		for( dwI=0; dwI <(nh/2); dwI++)
		{
			/* chroma for the odd bytes alternates between this line and the next */
			bUVSel = (dwI&0x1);
			hws_yuyv_line(pDstYUV, pSrc1, bUVSel ? pNextLineSrc1 : pSrc1,
				      bUVSel ? pSrc1 : pNextLineSrc1, dwPacket);
			pSrc1 += dwPacket*24;
			pNextLineSrc1 += dwPacket*24;
			pDstYUV += dwPacket*32;
			memcpy(pDstYUV,(pDstYUV-nw*2),nw*2);
			pDstYUV = pDstYUV+ nw*2;
		}
		/* the last field line (shown twice) has no next line for its chroma: black */
		FillBlackYUYV(pDstYUV - 2*nw*2, 2*nw);

	}

}


/*
 * Take at most one queued buffer from every streaming consumer, so each
 * consumer gets each frame once. Returns the number of buffers moved to @list.
 */
static int hws_take_buffers(struct hws_video *videodev, struct list_head *list)
{
    struct hws_vfh_ctx *ctx;
    unsigned long flags;
    int n = 0;

    spin_lock_irqsave(&videodev->consumers_lock, flags);
    list_for_each_entry(ctx, &videodev->consumers, node) {
        struct hwsvideo_buffer *buf;

        if (!ctx->streaming)
            continue;
        spin_lock(&ctx->qlock);
        buf = list_first_entry_or_null(&ctx->buf_queue, struct hwsvideo_buffer, queue);
        if (buf) {
            list_move_tail(&buf->queue, list);
            n++;
        }
        spin_unlock(&ctx->qlock);
    }
    spin_unlock_irqrestore(&videodev->consumers_lock, flags);

    return n;
}

/*
 * One buffer from each streaming consumer that has none in @list yet: apps
 * that queued after the direct path picked the frame's buffers at its first
 * half. hws_complete_buffers() fills them from the first buffer.
 */
static void hws_take_late_buffers(struct hws_video *videodev, struct list_head *list)
{
    struct hws_vfh_ctx *ctx;
    struct hwsvideo_buffer *buf;
    unsigned long flags;

    spin_lock_irqsave(&videodev->consumers_lock, flags);
    list_for_each_entry(ctx, &videodev->consumers, node) {
        bool have = false;

        if (!ctx->streaming)
            continue;
        list_for_each_entry(buf, list, queue)
            if (buf->vb.vb2_buf.vb2_queue->drv_priv == ctx) {
                have = true;
                break;
            }
        if (have)
            continue;
        spin_lock(&ctx->qlock);
        buf = list_first_entry_or_null(&ctx->buf_queue, struct hwsvideo_buffer, queue);
        if (buf)
            list_move_tail(&buf->queue, list);
        spin_unlock(&ctx->qlock);
    }
    spin_unlock_irqrestore(&videodev->consumers_lock, flags);
}

/*
 * How many waiting frames some consumer can still take: the largest number
 * of queued buffers over the streaming consumers. Frames beyond that reach
 * nobody; the rest go out in order, and a consumer with fewer buffers misses
 * some of them without costing the others any (the smallest count would drop
 * frames an app with enough buffers could have shown). Returns INT_MAX if
 * nobody streams.
 */
static int hws_max_queued(struct hws_video *videodev)
{
    struct hws_vfh_ctx *ctx;
    struct list_head *pos;
    unsigned long flags;
    int max = 0;
    bool streaming = false;

    spin_lock_irqsave(&videodev->consumers_lock, flags);
    list_for_each_entry(ctx, &videodev->consumers, node) {
        int n = 0;

        if (!ctx->streaming)
            continue;
        spin_lock(&ctx->qlock);
        list_for_each(pos, &ctx->buf_queue)
            if (++n >= MAX_VIDEO_QUEUE)
                break;
        spin_unlock(&ctx->qlock);
        streaming = true;
        if (n > max)
            max = n;
    }
    spin_unlock_irqrestore(&videodev->consumers_lock, flags);
    return streaming ? max : INT_MAX;
}

/*
 * Convert one captured frame (bBuf) into an output buffer of nWidth x nHeight
 * YUYV. bBuf is a ring slot marked MEM_READING, so it is read in place (no
 * staging copy). Uses the channel's scratch buffers, so only the channel's
 * worker may call it. Returns 0 on success.
 */
static int hws_convert_frame(struct hws_pcie_dev *pdx, int nDecoder, BYTE *bBuf, BYTE *dst,
                             int in_width, int in_height, int in_vsize, int nCopySize,
                             int interlace, int nWidth, int nHeight, int out_size,
                             int needRotateVideo)
{
    if (pdx->m_Device_SupportYV12 == 1) {
        if (in_vsize != out_size) {
            if (!pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf) {
                return -1;
            }
            if (!needRotateVideo) {
                FillYUU2(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pScaler, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, dst, in_width, in_height, nWidth, nHeight);
            } else {
                if (!pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf) {
                    return -1;
                }
                FillYUU2(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                /* scale to the unrotated size, then rotate (as for YUYV) */
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pScaler, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, in_width, in_height, nHeight, nWidth);
                VideoRotate90deg(pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, dst, nHeight, nWidth, nWidth, nHeight);
            }
        } else {
            if ((in_width == nHeight) && (in_height == nWidth)) {
                if (!pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf) {
                    return -1;
                }
                FillYUU2(bBuf, pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, in_width, in_height, interlace);
                VideoRotate90deg(pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, dst, nHeight, nWidth, nWidth, nHeight);
            } else {
                FillYUU2(bBuf, dst, in_width, in_height, interlace);
            }
        }
    } else if (pdx->m_Device_SupportYV12 == 2) {
        if (in_vsize != out_size) {
            if (!pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf) {
                return -1;
            }
            if (!needRotateVideo) {
                FillNV12ToYUY2(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pScaler, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, dst, in_width, in_height, nWidth, nHeight);
            } else {
                if (!pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf) {
                    return -1;
                }
                FillNV12ToYUY2(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                /* scale to the unrotated size, then rotate (as for YUYV) */
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pScaler, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, in_width, in_height, nHeight, nWidth);
                VideoRotate90deg(pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, dst, nHeight, nWidth, nWidth, nHeight);
            }
        } else {
            if ((in_width == nHeight) && (in_height == nWidth)) {
                if (!pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf) {
                    return -1;
                }
                FillNV12ToYUY2(bBuf, pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, in_width, in_height, interlace);
                VideoRotate90deg(pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, dst, nHeight, nWidth, nWidth, nHeight);
            } else {
                FillNV12ToYUY2(bBuf, dst, in_width, in_height, interlace);
            }
        }
    } else {
        if (in_vsize != out_size) {
            if (!pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf) {
                return -1;
            }
            if (!needRotateVideo) {
                if (interlace == 0) {
                    /* progressive: the ring slot (MEM_READING) is the source, no staging copy */
                    VideoScaler(pdx->m_VideoInfo[nDecoder].m_pScaler, bBuf, dst, in_width, in_height, nWidth, nHeight);
                } else {
                    SetDeInterlace(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                    VideoScaler(pdx->m_VideoInfo[nDecoder].m_pScaler, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, dst, in_width, in_height, nWidth, nHeight);
                }
            } else {
                if (!pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf) {
                    return -1;
                }
                SetDeInterlace(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pScaler, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, in_width, in_height, nHeight, nWidth);
                VideoRotate90deg(pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, dst, nHeight, nWidth, nWidth, nHeight);
            }
        } else {
            if ((in_width == nHeight) && (in_height == nWidth)) {
                if (!pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf) {
                    return -1;
                }
                SetDeInterlace(bBuf, pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, in_width, in_height, interlace);
                VideoRotate90deg(pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, dst, nHeight, nWidth, nWidth, nHeight);
            } else {
                SetDeInterlace(bBuf, dst, in_width, in_height, interlace);
            }
        }
    }
    return 0;
}

/*
 * Complete the buffers on @list with one frame. @ok says whether the first
 * buffer was checked and holds a valid frame; the others get a copy of it.
 * The first buffer is handed back last: once done, the app may requeue it
 * (and with USERPTR unmap it) while we would still be copying from it.
 */
static int hws_complete_buffers(struct list_head *list, int ok, int out_size,
                                u64 frame_ts, u32 frame_seq)
{
    struct hwsvideo_buffer *buf, *tmp, *first;
    int delivered = 0;

    first = list_first_entry_or_null(list, struct hwsvideo_buffer, queue);
    if (!first)
        return 0;

    list_for_each_entry_safe(buf, tmp, list, queue) {
        if (buf == first)
            continue;
        if (ok && buf->mem && vb2_plane_size(&buf->vb.vb2_buf, 0) >= out_size)
            memcpy(buf->mem, first->mem, out_size);
        else
            buf->mem_bad = true;
        cond_resched();	/* callers can sleep; many consumers = many 4 MB copies */
    }

    list_for_each_entry_safe_reverse(buf, tmp, list, queue) {
        struct hws_vfh_ctx *ctx = buf->vb.vb2_buf.vb2_queue->drv_priv;
        bool bad = !ok || buf->mem_bad;

        list_del(&buf->queue);
        buf->mem_bad = false;
        if (bad) {
            vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
            continue;
        }

        /*
         * Per-consumer sequence: gaps mean frames this consumer missed. The
         * worker and the IRQ thread (direct path) can both deliver around a
         * mode switch; dlock keeps sequence and timestamp strictly increasing.
         */
        {
            unsigned long dflags;
            u32 seq;
            u64 ts = frame_ts;

            spin_lock_irqsave(&ctx->dlock, dflags);
            if (!ctx->seq_valid) {
                ctx->seq_base = frame_seq;
                ctx->seq_valid = true;
            }
            seq = frame_seq - ctx->seq_base;
            if (ctx->have_last) {
                if ((s32)(seq - ctx->last_seq) <= 0)
                    seq = ctx->last_seq + 1;
                if ((s64)(ts - ctx->last_ts) <= 0)
                    ts = ctx->last_ts + 1;
            }
            ctx->have_last = true;
            ctx->last_seq = seq;
            ctx->last_ts = ts;
            buf->vb.sequence = seq;
            buf->vb.vb2_buf.timestamp = ts;
            buf->vb.field = V4L2_FIELD_NONE;
            vb2_set_plane_payload(&buf->vb.vb2_buf, 0, out_size);
            vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
            spin_unlock_irqrestore(&ctx->dlock, dflags);
        }
        delivered++;
    }
    return delivered;
}

/* keeps "no signal" frames coming at ~30 fps instead of once per status poll */
static void hws_nosig_work(struct kthread_work *work)
{
    struct hws_video *videodev = container_of(work, struct hws_video, nosig_work.work);

    /*
     * Queue the conversion pass instead of running it here: it runs right
     * after on this same thread, and every flush of videowork (stop/start
     * streaming, recovery) then also waits for it.
     */
    kthread_queue_work(videodev->dev->conv_worker[videodev->index], &videodev->videowork);
}

static void video_data_process(struct kthread_work *p_work)
{
    struct hws_video *videodev = container_of(p_work, struct hws_video, videowork);
    struct hws_pcie_dev *pdx = videodev->dev;
    int nDecoder = videodev->index;
    ACAP_VIDEO_INFO *vi = &pdx->m_VideoInfo[nDecoder];
    struct hws_timing_stats *ts = &pdx->tstats[nDecoder];
    unsigned long devflags;
    int i, round;
    LIST_HEAD(bufs);

    /*
     * videoslock is held only to pick a slot and mark it MEM_READING; the
     * producer never writes a slot in that state. Conversion runs without the
     * lock, so interrupts stay enabled and the IRQ thread never spins on us.
     *
     * Ready frames are delivered oldest first and the loop drains the ring, so
     * a late worker run (or a coalesced queue_work) catches up instead of
     * leaving frames behind. The bound keeps a slow consumer from looping
     * here forever while the hardware keeps producing.
     */
    for (round = 0; round < 2 * MAX_VIDEO_QUEUE; round++) {
        int nVindex = -1, backlog = 0, no_video, ok, delivered, avail;
        int in_width, in_height, in_vsize, nCopySize, interlace;
        int nWidth, nHeight, out_size;
        int needRotateVideo = 0;
        u64 lock_ns, frame_ts;
        u32 frame_seq;
        BYTE *bBuf;

        /* outside videoslock: consumers_lock is never nested inside it */
        avail = hws_max_queued(videodev);

        spin_lock_irqsave(&pdx->videoslock[nDecoder], devflags);
        lock_ns = ktime_get_ns();
        /* read once: the status thread changes it without this lock */
        no_video = READ_ONCE(pdx->m_curr_No_Video[nDecoder]);
        nWidth  = videodev->current_out_width;
        nHeight = videodev->curren_out_height;
        out_size = nWidth * nHeight * 2;

        if (no_video) {
            u64 now = ktime_get_ns();

            /* ~30 fps whoever wakes us (status poll, timer, a late frame) */
            if (round == 0 && now - videodev->nosig_last_ns < 30 * NSEC_PER_MSEC) {
                spin_unlock_irqrestore(&pdx->videoslock[nDecoder], devflags);
                if (READ_ONCE(pdx->m_bVCapStarted[nDecoder]))
                    /* +1: rounding down could re-arm with 0 and spin until the next tick */
                    kthread_mod_delayed_work(pdx->conv_worker[nDecoder], &videodev->nosig_work,
                                     nsecs_to_jiffies(videodev->nosig_last_ns + 33 * NSEC_PER_MSEC - now) + 1);
                return;
            }
            videodev->nosig_last_ns = now;
            /* drop captured frames; deliver one "no signal" frame per run */
            for (i = 0; i < MAX_VIDEO_QUEUE; i++)
                if (vi->pStatusInfo[i].byLock == MEM_LOCK)
                    vi->pStatusInfo[i].byLock = MEM_UNLOCK;
            frame_seq = vi->seq_counter++;
            spin_unlock_irqrestore(&pdx->videoslock[nDecoder], devflags);

            /* the signal went away mid-frame: its second half will never come */
            if (round == 0)
                hws_direct_reclaim(pdx, nDecoder, NULL);

            if (round == 0 && hws_take_buffers(videodev, &bufs)) {
                struct hwsvideo_buffer *first = list_first_entry(&bufs, struct hwsvideo_buffer, queue);

                ok = first->mem && vb2_plane_size(&first->vb.vb2_buf, 0) >= out_size;
                if (ok)
                    SetNoVideoMem(first->mem, nWidth, nHeight, nDecoder);
                hws_complete_buffers(&bufs, ok, out_size, ktime_get_ns(), frame_seq);
            }
            if (READ_ONCE(pdx->m_bVCapStarted[nDecoder]))
                kthread_mod_delayed_work(pdx->conv_worker[nDecoder], &videodev->nosig_work, msecs_to_jiffies(33));
            return;
        }

        for (i = 0; i < MAX_VIDEO_QUEUE; i++) {
            if (vi->pStatusInfo[i].byLock != MEM_LOCK)
                continue;
            backlog++;
            if (nVindex < 0 || (s32)(vi->frame_seq[i] - vi->frame_seq[nVindex]) < 0)
                nVindex = i;
        }
        if (nVindex < 0) {
            spin_unlock_irqrestore(&pdx->videoslock[nDecoder], devflags);
            if (round == 0)
                atomic_inc(&ts->worker_empty);
            return;
        }
        if (backlog > ts->backlog_max)
            ts->backlog_max = backlog;

        /*
         * More frames waiting than the consumers have buffers for: they can't
         * all be shown, so drop the oldest ones now instead of delivering stale
         * frames and dropping the fresh ones. Keeps latency at a minimum while
         * an app with enough buffers still gets every frame, also next to a
         * slower one. And never more than two waiting: a worker that can't
         * keep up (slow conversion, busy CPU) shows the newest frames, not
         * ones a whole ring old.
         */
        while (backlog > 2 || (backlog > 1 && backlog > avail)) {
            int oldest = -1;

            for (i = 0; i < MAX_VIDEO_QUEUE; i++)
                if (vi->pStatusInfo[i].byLock == MEM_LOCK &&
                    (oldest < 0 || (s32)(vi->frame_seq[i] - vi->frame_seq[oldest]) < 0))
                    oldest = i;
            vi->pStatusInfo[oldest].byLock = MEM_UNLOCK;
            atomic_inc(&ts->ring_skip);
            backlog--;
            nVindex = -1;
            for (i = 0; i < MAX_VIDEO_QUEUE; i++)
                if (vi->pStatusInfo[i].byLock == MEM_LOCK &&
                    (nVindex < 0 || (s32)(vi->frame_seq[i] - vi->frame_seq[nVindex]) < 0))
                    nVindex = i;
        }

        vi->pStatusInfo[nVindex].byLock = MEM_READING;
        bBuf      = vi->m_pVideoBufData[nVindex];
        in_width  = vi->pStatusInfo[nVindex].dwWidth;
        in_height = vi->pStatusInfo[nVindex].dwHeight;
        interlace = vi->pStatusInfo[nVindex].dwinterlace;
        frame_ts  = vi->frame_ts[nVindex];
        frame_seq = vi->frame_seq[nVindex];
        hws_tstat_max(&ts->lock_hold_max, ktime_get_ns() - lock_ns);
        spin_unlock_irqrestore(&pdx->videoslock[nDecoder], devflags);

        /*
         * A direct-path frame still being fanned out is older than this one:
         * let it finish first. Checked after picking the slot: the IRQ thread
         * queues the fan-out before it commits a newer frame under videoslock,
         * so seeing that frame here means the fan-out is already visible.
         */
        if (READ_ONCE(videodev->fanout_inflight))
            hws_fanout_drain(videodev);	/* same thread as the queued fan-out: no wait */

        hws_tstat_max(&ts->wq_lat_max, lock_ns - frame_ts);
        if (timing_debug >= 2 && lock_ns - frame_ts > hws_frame_period_ns(pdx, nDecoder))
            pr_warn_ratelimited("HwsCapture ch%d: worker late, frame waited %llu us (backlog %d)\n",
                                nDecoder, div_u64(lock_ns - frame_ts, NSEC_PER_USEC), backlog);

        if (pdx->m_Device_SupportYV12 == 1)
            nCopySize = ((in_width * 12 * in_height) / 8);
        else if (pdx->m_Device_SupportYV12 == 2)
            nCopySize = ((in_width * 5 * in_height) / 4);
        else
            nCopySize = (in_width * in_height * 2);

        if (interlace == 1)
            in_height = in_height * 2;

        in_vsize = in_width * in_height * 2;

        if ((in_width == 1280) && (in_height == 720) && (nWidth == 1080) && (nHeight == 1920))
            needRotateVideo = 1;
        else if ((in_width == 960) && (in_height == 540) && (nWidth == 1080) && (nHeight == 1920))
            needRotateVideo = 1;

        delivered = 0;
        if (hws_take_buffers(videodev, &bufs)) {
            struct hwsvideo_buffer *first = list_first_entry(&bufs, struct hwsvideo_buffer, queue);

            /* the 4:2:0 converters work in 16-pixel packets; other widths make
             * them read before the start of the output buffer */
            ok = bBuf && first->mem &&
                 in_width > 0 && in_height > 0 &&
                 (pdx->m_Device_SupportYV12 == 0 || (in_width % 16) == 0) &&
                 in_width <= MAX_VIDEO_HW_W && in_height <= MAX_VIDEO_HW_H &&
                 nCopySize <= pdx->m_MaxHWVideoBufferSize &&
                 vb2_plane_size(&first->vb.vb2_buf, 0) >= out_size;
            /*
             * The 4:2:0 converters look ahead a few lines past the end of the
             * frame on their last line group; give them zeros there rather
             * than an older frame's bytes (same output as with a staging copy).
             */
            if (ok && pdx->m_Device_SupportYV12 != 0 &&
                nCopySize + 4 * in_width * 2 <= pdx->m_MaxHWVideoBufferSize)
                memset(bBuf + nCopySize, 0, 4 * in_width * 2);
            if (ok)
                ok = hws_convert_frame(pdx, nDecoder, bBuf, first->mem, in_width, in_height,
                                       in_vsize, nCopySize, interlace, nWidth, nHeight,
                                       out_size, needRotateVideo) == 0;
            delivered = hws_complete_buffers(&bufs, ok, out_size, frame_ts, frame_seq);
        }

        spin_lock_irqsave(&pdx->videoslock[nDecoder], devflags);
        if (vi->pStatusInfo[nVindex].byLock == MEM_READING)
            vi->pStatusInfo[nVindex].byLock = MEM_UNLOCK;
        spin_unlock_irqrestore(&pdx->videoslock[nDecoder], devflags);

        if (delivered) {
            atomic_inc(&ts->frames_out);
        } else {
            atomic_inc(&ts->no_app_buf);
            if (timing_debug >= 2)
                pr_warn_ratelimited("HwsCapture ch%d: frame dropped, app had no buffer queued\n", nDecoder);
        }
    }
}

static void hws_get_video_param(struct hws_pcie_dev *dev,int index)
{

	int width,height;
	width= dev->m_pVCAPStatus[index][0].dwWidth;
	height=dev->m_pVCAPStatus[index][0].dwHeight;
	dev->video[index].current_out_pixfmt =0;
	dev->video[index].current_out_size_index = 0;
	dev->video[index].current_out_width = width;
	dev->video[index].curren_out_height = height;
	dev->video[index].current_out_framerate = 60;
	dev->video[index].Interlaced = 0;

}

static void hws_adapters_init(struct hws_pcie_dev *dev)
{
	int i;
	for (i = 0; i <MAX_VID_CHANNELS; i++) {
		hws_get_video_param(dev,i);
	}
}
/*
 * /sys/class/video4linux/videoN/stats: what the driver sees on this channel,
 * for monitoring while streaming (tools/stutter-monitor.py). Counts are totals
 * since the driver loaded; reading costs a few counter reads, nothing on the
 * frame path.
 */
static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hws_video *v = video_get_drvdata(to_video_device(dev));
	struct hws_pcie_dev *pdx = v->dev;
	int ch = v->index;
	struct hws_timing_stats *ts = &pdx->tstats[ch];
	struct hws_audio *a = &pdx->audio[ch];
	atomic_t *c[HWS_ST_N];
	u64 t[HWS_ST_N], tpf = atomic64_read(&v->tpf);
	struct hws_vfh_ctx *ctx;
	int open = 0, streaming = 0, i;
	unsigned long flags;

	hws_tstat_counters(ts, c);
	spin_lock_irqsave(&ts->fold_lock, flags);
	for (i = 0; i < HWS_ST_N; i++)
		t[i] = ts->tot[i] + (u32)atomic_read(c[i]);
	spin_unlock_irqrestore(&ts->fold_lock, flags);
	spin_lock_irqsave(&v->consumers_lock, flags);
	list_for_each_entry(ctx, &v->consumers, node) {
		open++;
		streaming += ctx->streaming;
	}
	spin_unlock_irqrestore(&v->consumers_lock, flags);

	return scnprintf(buf, PAGE_SIZE,
		"channel %d\nstreaming %d\nsignal %d\n"
		"input %ux%u%s\ninput_fps %u\nframe_interval %u/%u\noutput %dx%d\n"
		"apps_open %d\napps_streaming %d\n"
		"card_frames %u\nframes_in %llu\nframes_out %llu\ndirect %llu\n"
		"dma_overrun %llu\nstale_half %llu\nunpaired %llu\nring_overwrite %llu\nring_skip %llu\n"
		"aborted %llu\ncopy_missed %llu\nirq_lost %llu\ncard_late %llu\ncard_late_start %llu\n"
		"no_app_buf %llu\nirq_busy %llu\nwq_coalesced %llu\nworker_empty %llu\n"
		"audio_running %d\naudio_card_packets %u\naudio_silence_packets %u\n"
		"card_lost %d\nirq_paused %d\ncapture_starts %u\nstalled %d\n",
		ch, pdx->m_bVCapStarted[ch] ? 1 : 0, READ_ONCE(pdx->m_curr_No_Video[ch]) ? 0 : 1,
		pdx->m_pVCAPStatus[ch][0].dwWidth, pdx->m_pVCAPStatus[ch][0].dwHeight,
		pdx->m_pVCAPStatus[ch][0].dwinterlace ? "i" : "p",
		pdx->m_pVCAPStatus[ch][0].dwFrameRate, (u32)(tpf >> 32), (u32)tpf,
		v->current_out_width, v->curren_out_height,
		open, streaming,
		READ_ONCE(pdx->m_half_cnt[ch][0]), t[HWS_ST_IN], t[HWS_ST_OUT], t[HWS_ST_DIRECT],
		t[HWS_ST_OVERRUN], t[HWS_ST_STALE], t[HWS_ST_UNPAIRED], t[HWS_ST_OVERWRITE], t[HWS_ST_SKIP],
		t[HWS_ST_ABORTED], t[HWS_ST_MISSED],
		t[HWS_ST_IRQ_LOST], t[HWS_ST_CARD_LATE], t[HWS_ST_LATE_START],
		t[HWS_ST_NO_BUF], t[HWS_ST_IRQ_BUSY], t[HWS_ST_COALESCED], t[HWS_ST_EMPTY],
		READ_ONCE(a->running) ? 1 : 0, atomic_read(&a->pkts_card), atomic_read(&a->pkts_silence),
		READ_ONCE(pdx->m_PciDeviceLost) ? 1 : 0, READ_ONCE(pdx->irq_storm_off) ? 1 : 0,
		READ_ONCE(pdx->capture_starts[ch]), READ_ONCE(pdx->wd_stalled[ch]) ? 1 : 0);
}
static DEVICE_ATTR_RO(stats);

static int hws_video_register(struct hws_pcie_dev *dev)
{
	int i;
	int err=-1;

	if (dev->m_DeviceHW_Version == 0)
		dev_info(&dev->pdev->dev, "no picture controls on this hardware version\n");
	for(i=0;i<dev->m_nCurreMaxVideoChl;i++){
		struct hws_video *video = &dev->video[i];
		struct video_device *vdev = &video->vdev;

		video->v4l2_dev.release = hws_v4l2_dev_release;
		err = v4l2_device_register(&dev->pdev->dev, &video->v4l2_dev);
		if(err<0){
			dev_err(&dev->pdev->dev, "v4l2_device_register %d failed\n", i);
			goto fail;
		}
		/* dropped by hws_v4l2_dev_release() */
		kref_get(&dev->kref);
		dev->nr_v4l2_registered++;

		video->fileindex =0;
		video->pixfmt = V4L2_PIX_FMT_YUYV;
		err = hws_init_controls(video);
		if (err) {
			dev_err(&dev->pdev->dev, "control init %d failed: %d\n", i, err);
			goto fail;
		}
		vdev->ctrl_handler = &video->ctrl_handler;
		vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
		vdev->v4l2_dev = &video->v4l2_dev;
		/*
		 * No core lock: buffer ioctls lock their own per-file queue, format
		 * ioctls take video_lock. A core lock would be held across a blocking
		 * DQBUF and stall every other user of the node.
		 */
		vdev->lock = NULL;
		vdev->queue = NULL;	/* every open file has its own queue */
		vdev->fops = &hws_fops;
		strscpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));
		vdev->release = video_device_release_empty;	/* embedded; lifetime via v4l2_dev */
		vdev->vfl_dir = VFL_DIR_RX;
		vdev->ioctl_ops = &hws_ioctl_fops;
		video_set_drvdata(vdev, video);

		#if (LINUX_VERSION_CODE < KERNEL_VERSION(5,7,0))
		err = video_register_device(vdev, VFL_TYPE_GRABBER,-1);
		#else
		err = video_register_device(vdev, VFL_TYPE_VIDEO,-1);
		#endif
		if(err!=0){
			dev_err(&dev->pdev->dev, "video_register_device %d failed\n", i);
			goto fail;
		}
		/* monitoring only: the channel works without it */
		if (device_create_file(&vdev->dev, &dev_attr_stats))
			dev_warn(&dev->pdev->dev, "no stats file for channel %d\n", i);
	}
	return 0;
fail:
	hws_video_unregister_nodes(dev);
	hws_video_release_v4l2(dev);
	return err;
}

/* HDMI 0x39[3:0] - CS_DATA[27:24] 0 for reserved values*/
static struct snd_pcm_hardware audio_pcm_hardware = {
	.info 				=	(SNDRV_PCM_INFO_MMAP |
	SNDRV_PCM_INFO_INTERLEAVED |
	SNDRV_PCM_INFO_BLOCK_TRANSFER |
	SNDRV_PCM_INFO_BATCH |		/* the position moves in 1024-frame packets */
	SNDRV_PCM_INFO_MMAP_VALID),
	.formats 			=	SNDRV_PCM_FMTBIT_S16_LE,
	.rates 				=   SNDRV_PCM_RATE_48000,
	.rate_min 			=	48000,
	.rate_max 			=	48000,
	.channels_min 		=	2,
	.channels_max 		=	2,
	.buffer_bytes_max 	=	128*1024,	/* ~680 ms */
	/* the card delivers 4096-byte packets; smaller periods can't be honoured */
	.period_bytes_min 	=	4096,
	.period_bytes_max 	=	16*1024,
	.periods_min 		=	2,
	.periods_max 		=	255,
};
static int hws_pcie_audio_open(struct snd_pcm_substream *substream)
{
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;
	int err;

	runtime->hw = audio_pcm_hardware;
	/* the card delivers 1024-frame packets: periods of whole packets, whole periods per buffer */
	err = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0)
		return err;
	err = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 1024);
	if (err < 0)
		return err;
	spin_lock_irqsave(&drv->ring_lock, flags);
	drv->sample_rate_out        = 48000;
	drv->channels               = 2;
	drv->running                = false;
	drv->substream = substream;
	spin_unlock_irqrestore(&drv->ring_lock, flags);
	return 0;
}


/*
 * While a capture runs: if the card sends no audio (no signal, or a source
 * without audio), deliver silent packets at the card's own pace so the stream
 * keeps moving - otherwise apps see it stall and fail with an I/O error.
 * Stops by itself once real packets arrive again or the stream stops.
 */
/*
 * Frames in the ALSA ring the app hasn't read yet: ALSA's count up to its
 * last look at our pointer, plus what we wrote since (that look also happens
 * mid-period, on every read and avail/delay query). ring_lock held.
 */
static long hws_audio_unread(struct hws_audio *drv, struct snd_pcm_runtime *runtime)
{
	long ring = drv->ring_size_byframes;
	long hw = (long)(READ_ONCE(runtime->status->hw_ptr) % ring);

	return (long)snd_pcm_capture_avail(runtime) + (drv->ring_wpos_byframes - hw + ring) % ring;
}

static void hws_audio_silence_work(struct work_struct *work)
{
	struct hws_audio *drv = container_of(to_delayed_work(work), struct hws_audio, silence_work);
	struct hws_pcie_dev *pdx = drv->dev;
	unsigned int bytes = min_t(unsigned int, pdx->m_dwAudioPTKSize, sizeof(hws_silence));
	long pkt_frames = bytes / 4;	/* S16 stereo: 4 bytes per frame */
	u64 pkt_ns = div_u64((u64)pkt_frames * NSEC_PER_SEC, 48000);
	/*
	 * The card has stopped sending when its last packet's interrupt (the hard
	 * IRQ's time: a packet merely delivered late still counts as sent) is 2.5
	 * packets old - no signal, or a source without audio. As a backstop, also
	 * when nothing was delivered for 250 ms although interrupts come.
	 */
	u64 gap_ns = 5 * pkt_ns / 2, lr, card, ref, now;
	unsigned long flags;
	bool running, filling;
	long room = 0;
	s64 delay_ns;
	int n = 0, max_n = 0;

	/* under the lock: START sets the timing and running together */
	spin_lock_irqsave(&drv->ring_lock, flags);
	running = drv->running;
	lr = atomic64_read(&drv->last_real_ns);
	if (running && drv->substream && drv->substream->runtime && drv->ring_size_byframes)
	{
		/* a margin of one or two packets, less with a ring of only two periods */
		long margin = clamp_t(long, (long)drv->ring_size_byframes - (long)drv->period_size_byframes,
				      pkt_frames, 2 * pkt_frames);

		room = (long)drv->ring_size_byframes - hws_audio_unread(drv, drv->substream->runtime) - margin;
	}
	spin_unlock_irqrestore(&drv->ring_lock, flags);
	if (!running || !pkt_frames)
		return;
	/* the newest packet the card completed (START set lr after older ones) */
	card = max3(lr, (u64)atomic64_read(&pdx->m_audio_half_ts[drv->index][0]),
		    (u64)atomic64_read(&pdx->m_audio_half_ts[drv->index][1]));
	now = ktime_get_ns();	/* after the reads: now >= card */
	filling = (s64)(now - card) > (s64)gap_ns || (s64)(now - lr) > (s64)(250 * NSEC_PER_MSEC);
	if (filling)
	{
		/*
		 * Silence continues the card's packet grid from its last packet and
		 * fills the whole gap, so the audio stays in step with the video. It
		 * catches up a few packets at a time, never more than the app's buffer
		 * has room for (a full buffer is an xrun: the stream stops, the app
		 * restarts it). Only a debt beyond half a second (the system stalled,
		 * the app stopped reading) is given up.
		 */
		ref = (s64)(now - card) > (s64)gap_ns ? card : lr;
		if ((s64)(drv->silence_next_ns - (ref + pkt_ns)) < 0)
			drv->silence_next_ns = ref + pkt_ns;
		if ((s64)(now - drv->silence_next_ns) > (s64)(500 * NSEC_PER_MSEC))
			drv->silence_next_ns = now - 500 * NSEC_PER_MSEC;
		max_n = clamp_t(long, room / pkt_frames, 0, 8);
		for (n = 0; n < max_n && (s64)(now - drv->silence_next_ns) >= 0; n++)
		{
			if (!_deliver_samples(drv, hws_silence, bytes, lr, drv->silence_next_ns))
			{
				filling = false;	/* the card is sending again */
				break;
			}
			atomic_inc(&drv->pkts_silence);
			drv->silence_next_ns += pkt_ns;
		}
	}
	if (filling)
	{
		now = ktime_get_ns();
		if ((s64)(drv->silence_next_ns - now) > 0)
			delay_ns = drv->silence_next_ns - now;	/* caught up: the next packet's time */
		else if (n == max_n && max_n < 8)
			/* behind, the app's buffer full: it reads soon after each period
			 * while it is reading at all; one that stopped is asked once a packet */
			delay_ns = n ? pkt_ns / 4 : pkt_ns;
		else
			delay_ns = 1;		/* behind: next jiffy */
	}
	else
	{
		/* real audio flowing: look again when the gap could first be long enough */
		delay_ns = max_t(s64, (s64)(card + gap_ns - now), (s64)(pkt_ns / 2));
	}
	queue_delayed_work(pdx->wq, &drv->silence_work,
			   max_t(unsigned long, 1, DIV_ROUND_UP_ULL((u64)delay_ns, NSEC_PER_SEC / HZ)));
}

/* deliveries run in the IRQ thread and the silence work: wait for one in progress */
static void hws_audio_sync(struct hws_audio *drv)
{
	struct hws_pcie_dev *pdx = drv->dev;

	cancel_delayed_work_sync(&drv->silence_work);	/* running is false by now: it doesn't re-arm */

	/* irq_lock: remove can't free (and reuse) the IRQ in between */
	mutex_lock(&pdx->irq_lock);
	if (pdx->irq_line != -1)
		synchronize_irq(pdx->irq_line);
	mutex_unlock(&pdx->irq_lock);
}

static int hws_pcie_audio_close(struct snd_pcm_substream *substream)
{
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	unsigned long flags;

	spin_lock_irqsave(&drv->ring_lock, flags);
	drv->running = false;
	drv->substream = NULL;
	spin_unlock_irqrestore(&drv->ring_lock, flags);
	/* a delivery that already passed the check must finish before the runtime goes */
	hws_audio_sync(drv);
	return 0;
}
static int hws_pcie_audio_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

static int hws_pcie_audio_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int hws_pcie_audio_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	unsigned long flags;

	/* a delivery left over from the previous run must not land in the new one */
	hws_audio_sync(drv);
	spin_lock_irqsave(&drv->ring_lock, flags);
	drv->ring_size_byframes = runtime->buffer_size;
	drv->ring_wpos_byframes = 0;
	drv->period_size_byframes = runtime->period_size;
	drv->period_used_byframes = 0;
	drv->ring_offsize =0;
	drv->ring_over_size =0;
	spin_unlock_irqrestore(&drv->ring_lock, flags);

	return 0;
}
static int hws_pcie_audio_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct hws_audio *chip = snd_pcm_substream_chip(substream);
	struct hws_pcie_dev *dev= chip->dev;
	unsigned long flags;
	switch(cmd){
		case SNDRV_PCM_TRIGGER_START:
			/* shut down / being removed: the card stays off */
			if (READ_ONCE(dev->removing))
				return -ENODEV;
			//start dma
			/* reset the capture queue before deliveries are accepted again */
			StartAudioCapture(dev,chip->index);
			spin_lock_irqsave(&chip->ring_lock, flags);
			chip->ring_wpos_byframes = 0;
			chip->period_used_byframes = 0;
			/* fresh timing before running is seen: a silence work queued late
			 * by the status thread must not fill from the last run's state */
			atomic64_set(&chip->last_real_ns, ktime_get_ns());
			chip->silence_next_ns = 0;
			chip->running = true;
			spin_unlock_irqrestore(&chip->ring_lock, flags);
			/* not racing a cancel: ALSA's prepare (which cancels synchronously)
			 * always comes between a STOP and the next START */
			mod_delayed_work(dev->wq, &chip->silence_work, msecs_to_jiffies(64));
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
			//stop dma
			/* not running first: the status thread restarts running streams
			 * after a card recovery, under ring_lock */
			spin_lock_irqsave(&chip->ring_lock, flags);
			chip->running = false;
			spin_unlock_irqrestore(&chip->ring_lock, flags);
			StopAudioCapture(dev,chip->index);
			break;
		default:
			return -EINVAL;
	}
	return 0;
}


static snd_pcm_uframes_t hws_pcie_audio_pointer(struct snd_pcm_substream *substream)
{
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	snd_pcm_uframes_t pos;
	unsigned long flags;
	spin_lock_irqsave(&drv->ring_lock,flags);  //spin_lock
	pos = drv->ring_wpos_byframes;
	spin_unlock_irqrestore(&drv->ring_lock,flags); //spin_unlock
	return pos;
}

static const struct snd_pcm_ops hws_pcie_pcm_ops ={
	.open =			hws_pcie_audio_open,
	.close = 		hws_pcie_audio_close,
	.ioctl =		snd_pcm_lib_ioctl,
	.hw_params = 	hws_pcie_audio_hw_params,
	.hw_free =		hws_pcie_audio_hw_free,
	.prepare =		hws_pcie_audio_prepare,
	.trigger =		hws_pcie_audio_trigger,
	.pointer =		hws_pcie_audio_pointer
};

static int hws_audio_register(struct hws_pcie_dev *dev)
{
	struct snd_pcm		*pcm;
	struct snd_card 	*card;
	int ret;
	int i;
	int ai_index;
	char card_id[16];
	char audioname[32];
	char pcm_name[32];
	ai_index = dev->m_Device_PortID*dev->m_nCurreMaxVideoChl+1;
	for(i=0;i<dev->m_nCurreMaxVideoChl;i++){
		/* ALSA card ids must not contain spaces; names may */
		snprintf(card_id, sizeof(card_id), "%s%d", HWS_AUDOI_NAME, i+ai_index);
		snprintf(audioname, sizeof(audioname), "%s %d",HWS_AUDOI_NAME,i+ai_index);
		snprintf(pcm_name, sizeof(pcm_name), "%s %d PCM", HWS_AUDOI_NAME, i+ai_index);
		ret = snd_card_new(&dev->pdev->dev, -1, card_id, THIS_MODULE, 0, &card);
		if (ret < 0){
			dev_err(&dev->pdev->dev, "snd_card_new failed: %d\n", ret);
			goto fail;
		}
		/* from here on freeing the card drops this reference */
		kref_get(&dev->kref);
		card->private_data = &dev->audio[i];
		card->private_free = hws_snd_card_free;
		strscpy(card->driver, KBUILD_MODNAME, sizeof(card->driver));
		strscpy(card->shortname, audioname, sizeof(card->shortname));
		snprintf(card->longname, sizeof(card->longname), "%s at %s", audioname, pci_name(dev->pdev));

		ret = snd_pcm_new(card,pcm_name,0,0,1,&pcm);
		if (ret < 0){
			dev_err(&dev->pdev->dev, "snd_pcm_new failed: %d\n", ret);
			snd_card_free(card);
			goto fail;
		}
		pcm->private_data = &dev->audio[i];
		strscpy(pcm->name, pcm_name, sizeof(pcm->name));
		snd_pcm_set_ops(pcm,SNDRV_PCM_STREAM_CAPTURE,&hws_pcie_pcm_ops);
		snd_pcm_lib_preallocate_pages_for_all(
			pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,1,0))
			card->dev,
			#else
			snd_dma_continuous_data(GFP_KERNEL),
			#endif
			audio_pcm_hardware.buffer_bytes_max,
			audio_pcm_hardware.buffer_bytes_max
		);
		dev->audio[i].sample_rate_out        = 48000;
		dev->audio[i].channels               = 2;
		dev->audio[i].resampled_buf_size = dev->audio[i].sample_rate_out * 2/* sample bytes */ * dev->audio[i].channels /* channels */;
		ret = snd_card_register(card);
		if ( ret < 0) {
			dev_err(&dev->pdev->dev, "snd_card_register failed\n");
			snd_card_free(card);
			goto fail;
		}
		dev->audio[i].card =card;
	}
	return 0;
fail:
	hws_audio_unregister(dev);
	return ret;
}


static void  WRITE_REGISTER_ULONG (struct hws_pcie_dev *pdx,u32 RegisterOffset,u32 Value)
{
	iowrite32(Value, pdx->map_bar0_addr + RegisterOffset);

}

static u32 READ_REGISTER_ULONG (struct hws_pcie_dev *pdx,u32 RegisterOffset)
{
	return ioread32(pdx->map_bar0_addr + RegisterOffset);
}
static int Check_Busy(struct hws_pcie_dev *pdx)
{
	u32  statusreg;
	u32 TimeOut = 0;

	/* wait up to ~1s for the DSP to go idle; a wedged card must not hang removal */
	for (TimeOut = 0; TimeOut < 100; TimeOut++)
	{
		statusreg = READ_REGISTER_ULONG(pdx,(u32)(CVBS_IN_BASE));
		if (statusreg == 0xFFFFFFFF)
		{
			return -ENODEV;
		}
		if ((statusreg & 0x08) == 0x00)
		{
			return 0;
		}
		usleep_range(10000, 11000);	/* msleep(10) may take 20 ms: keep the timeout honest */
	}
	dev_warn(&pdx->pdev->dev, "DSP did not go idle, status %X\n", statusreg);
	return -ETIMEDOUT;
}

static void StopDsp(struct hws_pcie_dev *pdx)
{
	u32  statusreg;
	statusreg = READ_REGISTER_ULONG(pdx,(u32)(CVBS_IN_BASE));
	if (statusreg == 0xFFFFFFFF)
	{
		return;
	}
	WRITE_REGISTER_ULONG(pdx,(u32)(CVBS_IN_BASE), 0x10);
	Check_Busy(pdx);
	WRITE_REGISTER_ULONG(pdx,( CVBS_IN_BASE + (2 * PCIE_BARADDROFSIZE)), 0x00);


}
static void EnableVideoCapture(struct hws_pcie_dev *pdx,int index,int en)
{
	ULONG status;
	int enable;
	unsigned long flags;
	WRITE_ONCE(pdx->m_bVCapStarted[index], en);
	if(pdx->m_PciDeviceLost) return;
	/* one register holds the enable bits of all channels: serialize the RMW */
	spin_lock_irqsave(&pdx->reg_lock, flags);
	status = READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE+2*PCIE_BARADDROFSIZE));
	/* read under reg_lock: see hws_park_channel() */
	if(en && !READ_ONCE(pdx->mode_unsupported[index]))
	{
		enable =1;
		enable = enable<<index;
		status =  status|enable;
	}
	else
	{
		enable =1;
		enable = enable<<index;
		enable = ~enable;
		status =  status&enable;
	}
	WRITE_REGISTER_ULONG(pdx,( CVBS_IN_BASE + (2 * PCIE_BARADDROFSIZE)), status);
	status = READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE+2*PCIE_BARADDROFSIZE));
	spin_unlock_irqrestore(&pdx->reg_lock, flags);
}
/*
 * An input mode the buffers aren't made for (status thread): the channel's
 * DMA goes off while capture stays "started" - apps get the no-signal
 * picture - and comes back on once the mode fits. The bit follows both
 * m_bVCapStarted and mode_unsupported, each written before and read inside
 * reg_lock here and in EnableVideoCapture(), so the last writer of either
 * leaves the right bit.
 */
static void hws_park_channel(struct hws_pcie_dev *pdx, int ch, bool park)
{
	ULONG status;
	unsigned long flags;

	WRITE_ONCE(pdx->mode_unsupported[ch], park);
	/* lost, or back but not brought up yet (its address table is empty):
	 * bring-up and StartVideoCapture apply the flag */
	if (READ_ONCE(pdx->m_PciDeviceLost) || READ_ONCE(pdx->need_bringup))
		return;
	spin_lock_irqsave(&pdx->reg_lock, flags);
	status = READ_REGISTER_ULONG(pdx, CVBS_IN_BASE + 2 * PCIE_BARADDROFSIZE);
	if (!park && READ_ONCE(pdx->m_bVCapStarted[ch]))
		status |= 1u << ch;
	else
		status &= ~(1u << ch);
	WRITE_REGISTER_ULONG(pdx, CVBS_IN_BASE + 2 * PCIE_BARADDROFSIZE, status);
	spin_unlock_irqrestore(&pdx->reg_lock, flags);
}

static void EnableAudioCapture(struct hws_pcie_dev *pdx,int index,int en)
{
	ULONG status;
	int enable;
	unsigned long flags;
	pdx->m_bACapStarted[index] = en;
	if(pdx->m_PciDeviceLost) return;
	spin_lock_irqsave(&pdx->reg_lock, flags);
	status =  READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE+3*PCIE_BARADDROFSIZE));

	if(en)
	{
		enable =1;
		enable = enable<<index;
		status =  status|enable;
	}
	else
	{
		enable =1;
		enable = enable<<index;
		enable = ~enable;
		status =  status&enable;
	}
	WRITE_REGISTER_ULONG(pdx,( CVBS_IN_BASE + (3 * PCIE_BARADDROFSIZE)), status);
	spin_unlock_irqrestore(&pdx->reg_lock, flags);
}

/*
 * Card interrupts on, unless the storm guard paused them (the status thread
 * re-enables those). The guard may trip right between the check and the
 * write - it can't take reg_lock in hard IRQ context - so look again after
 * the write and undo it then.
 */
static void hws_enable_card_irqs(struct hws_pcie_dev *pdx)
{
	if (READ_ONCE(pdx->irq_storm_off))
		return;
	WRITE_REGISTER_ULONG(pdx, INT_EN_REG_BASE, 0x3ffff);
	smp_mb();
	if (READ_ONCE(pdx->irq_storm_off))
		WRITE_REGISTER_ULONG(pdx, INT_EN_REG_BASE, 0);
}

static int SetVideoFormteSize(struct hws_pcie_dev *pdx,int ch,int w,int h)
{
	int hf_size;
	int down_size;

	if(pdx->m_Device_SupportYV12 ==0)
	{
		hf_size = (w*h)/(16*128);
		hf_size = hf_size*16*128;
		down_size = (w*h*2)-hf_size;
	}
	else if(pdx->m_Device_SupportYV12 ==1)
	{
		hf_size = (w*h*12)/(16*16*128);
		hf_size = hf_size*16*128;
		down_size = ((w*h*12)/8)-hf_size;
	}
	else if(pdx->m_Device_SupportYV12 ==2)
	{
		hf_size = (w*h*5)/(8*16*128);
		hf_size = hf_size*16*128;
		down_size = ((w*h*5)/4)-hf_size;
	}
	else
	{
		/* unknown layout code: the copy path and worker treat it as packed 4:2:2 */
		hf_size = (w*h)/(16*128);
		hf_size = hf_size*16*128;
		down_size = (w*h*2)-hf_size;
	}
	pdx->m_format[ch].dwWidth = w;		 // Image Width
	pdx->m_format[ch].dwWidth = w;		 // Image Width
	pdx->m_format[ch].HLAF_SIZE = hf_size;
	pdx->m_format[ch].DWON_SIZE =down_size;

	return 1;

}

/* [a, a + len) not inside one 512 MiB window of the card's address table */
static bool hws_dma_crosses_window(dma_addr_t a, size_t len)
{
	return ((u64)a & ~(u64)PCI_E_BAR_ADD_LOWMASK) !=
	       (((u64)a + len - 1) & ~(u64)PCI_E_BAR_ADD_LOWMASK);
}

static void DmaMemFreePool(struct hws_pcie_dev *pdx)
{
	int i, k;

	if(pdx->m_bBufferAllocate != TRUE)
		return;

	for(i=0; i<pdx->m_nMaxChl; i++)
	{
		if(pdx->m_pbyVideoBuffer[i])
		{
			dma_free_coherent(&pdx->pdev->dev, pdx->m_MaxHWVideoBufferSize, pdx->m_pbyVideoBuffer[i], pdx->m_pbyVideo_phys[i]);
			pdx->m_pbyVideoBuffer[i] = NULL;
			pdx->m_pbyAudioBuffer[i] = NULL;
		}
	}
	for(i=0; i<pdx->m_nCurreMaxVideoChl;i++)
	{
		vfree(pdx->m_VideoInfo[i].m_pVideoScalerBuf);
		pdx->m_VideoInfo[i].m_pVideoScalerBuf = NULL;
		vfree(pdx->m_VideoInfo[i].m_pScaler);
		pdx->m_VideoInfo[i].m_pScaler = NULL;
		vfree(pdx->m_VideoInfo[i].m_pVideoYUV2Buf);
		pdx->m_VideoInfo[i].m_pVideoYUV2Buf = NULL;
		vfree(pdx->m_VideoInfo[i].m_pRotateVideoBuf);
		pdx->m_VideoInfo[i].m_pRotateVideoBuf = NULL;
		for( k=0; k<MAX_VIDEO_QUEUE;k++)
		{
			kvfree(pdx->m_VideoInfo[i].m_pVideoBufData[k]);
			pdx->m_VideoInfo[i].m_pVideoBufData[k]= NULL;
		}
		//----audio release
		for( k=0; k<MAX_AUDIO_QUEUE;k++)
		{
			kfree(pdx->m_AudioInfo[i].m_pAudioBufData[k]);
			pdx->m_AudioInfo[i].m_pAudioBufData[k] = NULL;
		}
	}
	pdx->m_bBufferAllocate = FALSE;
}


static int  DmaMemAllocPool(struct hws_pcie_dev *pdx)
{
	int i, k;
	dma_addr_t phy_addr;

	if(pdx->m_bBufferAllocate == TRUE)
	{
		DmaMemFreePool(pdx);
	}
	/* from here on DmaMemFreePool() releases whatever got allocated */
	pdx->m_bBufferAllocate = TRUE;
	for(i=0; i<pdx->m_nMaxChl; i++)
	{
		/*
		 * The card reaches a buffer through one 512 MiB window of its address
		 * table (SetDMAAddress): one that crosses a window boundary would have
		 * the card write its end into unrelated memory. Normal and IOMMU
		 * allocations are aligned to their size; CMA's only to 1 MiB. Hold a
		 * crossing one while asking again, so the next one lands elsewhere.
		 */
		void *rej_va[8];
		dma_addr_t rej_pa[8], pa = 0;
		void *va;
		int nrej = 0;

		for (;;)
		{
			va = dma_alloc_coherent(&pdx->pdev->dev, pdx->m_MaxHWVideoBufferSize, &pa, GFP_KERNEL);
			if (!va || !hws_dma_crosses_window(pa, pdx->m_MaxHWVideoBufferSize))
				break;
			if (nrej == ARRAY_SIZE(rej_va))
			{
				dma_free_coherent(&pdx->pdev->dev, pdx->m_MaxHWVideoBufferSize, va, pa);
				va = NULL;
				break;
			}
			rej_va[nrej] = va;
			rej_pa[nrej++] = pa;
		}
		while (nrej--)
			dma_free_coherent(&pdx->pdev->dev, pdx->m_MaxHWVideoBufferSize, rej_va[nrej], rej_pa[nrej]);
		pdx->m_pbyVideoBuffer[i] = va;
		pdx->m_pbyVideo_phys[i] = pa;
		if(pdx->m_pbyVideoBuffer[i]== NULL)
		{
			dev_err(&pdx->pdev->dev, "DMA buffer %d allocation failed\n", i);
			goto fail;
		}
		phy_addr = 	pdx->m_pbyVideo_phys[i];

		pdx->m_dwVideoBuffer[i] = 	  ((u64)phy_addr)&0xFFFFFFFF;
		pdx->m_dwVideoHighBuffer[i] = ((u64)phy_addr>>32)&0xFFFFFFFF;

		/* audio lives in the last MAX_AUDIO_CAP_SIZE bytes of the channel's DMA buffer */
		pdx->m_pbyAudioBuffer[i] = (BYTE *)(pdx->m_pbyVideoBuffer[i] + pdx->m_MaxHWVideoBufferSize -MAX_AUDIO_CAP_SIZE );
		phy_addr = pdx->m_pbyVideo_phys[i] + (pdx->m_MaxHWVideoBufferSize -MAX_AUDIO_CAP_SIZE);
		pdx->m_pbyAudio_phys[i] = phy_addr;

		pdx->m_dwAudioBuffer[i] =    pdx->m_dwVideoBuffer[i]+ pdx->m_MaxHWVideoBufferSize -MAX_AUDIO_CAP_SIZE;
		pdx->m_dwAudioBufferHigh[i] = pdx->m_dwVideoHighBuffer[i];
	}

	//-------------- video buffer
	for(i=0; i<pdx->m_nCurreMaxVideoChl; i++)
	{
		pdx->m_VideoInfo[i].m_pVideoScalerBuf = vmalloc(MAX_VIDEO_HW_W*MAX_VIDEO_HW_H*2);
		pdx->m_VideoInfo[i].m_pScaler = vmalloc(sizeof(struct hws_scaler));	/* optional: 19 KB */
		pdx->m_VideoInfo[i].m_pRotateVideoBuf = vmalloc(MAX_VIDEO_HW_W*MAX_VIDEO_HW_H*2);
		if (!pdx->m_VideoInfo[i].m_pVideoScalerBuf || !pdx->m_VideoInfo[i].m_pRotateVideoBuf)
			goto fail;

		for( k=0; k<MAX_VIDEO_QUEUE;k++)
		{
			/* 4 MB each: kvmalloc falls back to vmalloc when memory is fragmented */
			pdx->m_VideoInfo[i].m_pVideoBufData[k] = kvzalloc(pdx->m_MaxHWVideoBufferSize, GFP_KERNEL);
			if(!pdx->m_VideoInfo[i].m_pVideoBufData[k])
				goto fail;
		}
	}

	//----------audio alloc
	for(i=0; i<pdx->m_nCurreMaxVideoChl; i++)
	{
		for( k=0; k<MAX_AUDIO_QUEUE;k++)
		{
			pdx->m_AudioInfo[i].m_pAudioBufData[k] =kzalloc(pdx->m_dwAudioPTKSize, GFP_KERNEL);
			if(!pdx->m_AudioInfo[i].m_pAudioBufData[k])
				goto fail;
			pdx->m_AudioInfo[i].pStatusInfo[k].byLock = MEM_UNLOCK;
		}
	}
	return 0;

fail:
	DmaMemFreePool(pdx);
	return -ENOMEM;
}

/* stop all capture and interrupts on the card; safe on a card that is gone */
static void hws_hw_stop(struct hws_pcie_dev *pdx)
{
	int i;
	u32  statusreg;

	/* a card known to be gone isn't read at all (arm64: SError on a removed device) */
	statusreg = READ_ONCE(pdx->m_PciDeviceLost) ? 0xFFFFFFFF : READ_REGISTER_ULONG(pdx,(0x4000));
	if (statusreg == 0xFFFFFFFF)
	{
		pdx->m_PciDeviceLost = 1;
	}
	else
	{
		StopDsp(pdx);
	}
	pdx->m_bStartRun = 0;
	for (i = 0; i<MAX_VID_CHANNELS; i++)
	{
		EnableVideoCapture(pdx,i,0);
		EnableAudioCapture(pdx,i,0);
	}
	if (!pdx->m_PciDeviceLost)
		WRITE_REGISTER_ULONG(pdx,INT_EN_REG_BASE, 0);
}

static void irq_teardown(struct hws_pcie_dev *lro)
{


	//else

	if (lro->irq_line != -1) {
		free_irq(lro->irq_line, lro);
	}
}
static void StopKSThread(struct hws_pcie_dev *pdx)
{
	if(pdx->mMain_tsk)
	{
		kthread_stop(pdx->mMain_tsk);
		pdx->mMain_tsk = NULL;
	}
}

/*
 * Lifetime: the device struct holds the per-channel video/audio state that
 * open file handles keep using after a PCI unbind. One reference belongs to
 * probe/remove, one to each registered v4l2_device; the v4l2 core drops the
 * latter once the last file handle of that channel is closed.
 */
static void hws_dev_release(struct kref *kref)
{
	struct hws_pcie_dev *pdx = container_of(kref, struct hws_pcie_dev, kref);
	int i;

	/* the copy threads queue conversions: gone first (idle by now; this also flushes) */
	for (i = 0; i < MAX_VID_CHANNELS; i++)
		if (pdx->copy_worker[i])
			kthread_destroy_worker(pdx->copy_worker[i]);
	/*
	 * Conversion threads: let a running pass finish (it may re-arm the
	 * no-signal timer), stop the timer, and only then destroy the thread,
	 * so a late timer can never queue onto a freed worker.
	 */
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		if (!pdx->conv_worker[i])
			continue;
		kthread_flush_worker(pdx->conv_worker[i]);
		kthread_cancel_delayed_work_sync(&pdx->video[i].nosig_work);
		kthread_destroy_worker(pdx->conv_worker[i]);
		pdx->conv_worker[i] = NULL;
	}
	for (i = 0; pdx->wq && i < MAX_VID_CHANNELS; i++)
		cancel_delayed_work_sync(&pdx->audio[i].silence_work);
	if (pdx->wq)
		destroy_workqueue(pdx->wq);
	kfree(pdx->pci_state);
	pci_dev_put(pdx->pdev);
	kfree(pdx);
}

static void hws_v4l2_dev_release(struct v4l2_device *v4l2_dev)
{
	struct hws_video *video = container_of(v4l2_dev, struct hws_video, v4l2_dev);

	/* last user of this channel is gone: controls can go too */
	if (video->ctrl_ready) {
		v4l2_ctrl_handler_free(&video->ctrl_handler);
		video->ctrl_ready = false;
	}
	kref_put(&video->dev->kref, hws_dev_release);
}

static void hws_video_unregister_nodes(struct hws_pcie_dev *dev)
{
	int i;

	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		/* first: waits for a running read of it */
		if (video_is_registered(&dev->video[i].vdev))
			device_remove_file(&dev->video[i].vdev.dev, &dev_attr_stats);
		video_unregister_device(&dev->video[i].vdev);	/* no-op if never registered */
	}
}

static void hws_video_release_v4l2(struct hws_pcie_dev *dev)
{
	int i;

	for (i = 0; i < dev->nr_v4l2_registered; i++) {
		v4l2_device_unregister(&dev->video[i].v4l2_dev);
		v4l2_device_put(&dev->video[i].v4l2_dev);
	}
	dev->nr_v4l2_registered = 0;
}

/* the card's reference on the device; dropped when ALSA frees the card */
static void hws_snd_card_free(struct snd_card *card)
{
	struct hws_audio *audio = card->private_data;

	kref_put(&audio->dev->kref, hws_dev_release);
}

static void hws_audio_unregister(struct hws_pcie_dev *dev)
{
	int i;

	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		if (dev->audio[i].card) {
			/* doesn't wait for open PCM handles; the card's kref keeps dev alive */
			snd_card_free_when_closed(dev->audio[i].card);
			dev->audio[i].card = NULL;
		}
	}
}

/* stop IRQ sources and every deferred context that touches the DMA buffers */
static void hws_quiesce(struct hws_pcie_dev *dev)
{
	int i;

	StopKSThread(dev);
	hws_hw_stop(dev);
	/* a card that ignored the stop must not DMA into memory we are about to free */
	pci_clear_master(dev->pdev);
	mutex_lock(&dev->irq_lock);
	irq_teardown(dev);	/* also waits for the IRQ thread */
	WRITE_ONCE(dev->irq_line, -1);
	mutex_unlock(&dev->irq_lock);
	for (i = 0; i < MAX_VID_CHANNELS; i++)
		if (dev->copy_worker[i])	/* nothing queues them any more */
			kthread_flush_work(&dev->video[i].copy_work);
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		hws_direct_reclaim(dev, i, NULL);
		kthread_flush_work(&dev->video[i].fanout_work);	/* hands its buffers back */
	}
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		/*
		 * A worker run that saw capture still on may re-arm nosig_work after
		 * the first cancel; once videowork is idle nothing can, so cancel again.
		 */
		kthread_cancel_delayed_work_sync(&dev->video[i].nosig_work);
		kthread_cancel_work_sync(&dev->video[i].videowork);
		kthread_cancel_delayed_work_sync(&dev->video[i].nosig_work);
		cancel_work_sync(&dev->audio[i].audiowork);
		cancel_delayed_work_sync(&dev->audio[i].silence_work);
	}
}

/*
 * New streams fail from now on, the video nodes go away and anyone blocked in
 * DQBUF is woken. Taking each engine_lock once waits for a start_streaming
 * that is already touching the hardware.
 */
static void hws_block_streams(struct hws_pcie_dev *dev)
{
	unsigned long flags;
	int i;

	WRITE_ONCE(dev->removing, true);
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		mutex_lock(&dev->video[i].engine_lock);
		mutex_unlock(&dev->video[i].engine_lock);
	}
	/* and for a control write that is about to touch the registers */
	mutex_lock(&dev->pm_lock);
	mutex_unlock(&dev->pm_lock);
	hws_video_unregister_nodes(dev);
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		struct hws_vfh_ctx *ctx;

		spin_lock_irqsave(&dev->video[i].consumers_lock, flags);
		list_for_each_entry(ctx, &dev->video[i].consumers, node)
			vb2_queue_error(&ctx->vbq);
		spin_unlock_irqrestore(&dev->video[i].consumers_lock, flags);
	}
}

static void hws_remove(struct pci_dev *pdev)
{
	int i;
	struct hws_pcie_dev *dev =
	(struct hws_pcie_dev*) pci_get_drvdata(pdev);
	if(dev == NULL || dev->map_bar0_addr == NULL) return;

	/* surprise removal: don't touch registers of a card that is gone (some
	 * arm64 root complexes raise an SError on accesses to a removed device) */
	if (!pci_device_is_present(pdev) ||
	    READ_REGISTER_ULONG(dev, CVBS_IN_BASE) == 0xFFFFFFFF)
		WRITE_ONCE(dev->m_PciDeviceLost, 1);

	hws_block_streams(dev);
	for (i = 0; i < MAX_VID_CHANNELS; i++)
		if (dev->audio[i].card)
			snd_card_disconnect(dev->audio[i].card);

	hws_quiesce(dev);
	hws_audio_unregister(dev);
	DmaMemFreePool(dev);

	iounmap(dev->info.mem[0].internal_addr);
	dev->info.mem[0].internal_addr = NULL;
	dev->map_bar0_addr = NULL;

	if (dev->msix_enabled)
	{
		pci_disable_msix(pdev);
		dev->msix_enabled = 0;
	}
	else if (dev->msi_enabled)
	{
		pci_disable_msi(pdev);
		dev->msi_enabled = 0;
	}
	if (dev->regions_requested)
		pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);

	/* dev itself goes away once the last open video file handle is closed */
	hws_video_release_v4l2(dev);
	kref_put(&dev->kref, hws_dev_release);
}
static void CheckCardStatus(struct hws_pcie_dev *pdx)
{
	ULONG status;
	unsigned long flags;
	if(pdx->m_PciDeviceLost) return;
	/* audio trigger (atomic) and video start can both get here */
	spin_lock_irqsave(&pdx->reg_lock, flags);
	status = READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE+0*PCIE_BARADDROFSIZE));
	if((status != 0xFFFFFFFF) && ((status&0x01) != 0x01))
	{
		/* set=1 does not call Enable*Capture, which take reg_lock themselves */
		InitVideoSys(pdx,1);
	}
	spin_unlock_irqrestore(&pdx->reg_lock, flags);

}
static int CheckVideoCapture(struct hws_pcie_dev *pdx,int index)
{
	ULONG status;
	int enable;
	if (READ_ONCE(pdx->m_PciDeviceLost))
		return 0;
	status = READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE+2*PCIE_BARADDROFSIZE));
	enable = (status >>index)&0x01;
	return enable;
}
static int CheckAudioCapture(struct hws_pcie_dev *pdx,int index)
{
	ULONG status;
	int enable;
	if (READ_ONCE(pdx->m_PciDeviceLost))
		return 0;
	status = READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE+3*PCIE_BARADDROFSIZE));
	enable = (status >>index)&0x01;
	return enable;
}

static int  StartAudioCapture(struct hws_pcie_dev *pdx,int index)
{
	int j;
	unsigned long flags;

	if(pdx->m_bACapStarted[index]==1)
	{
		if(CheckAudioCapture(pdx,index) ==0)
		{
			CheckCardStatus(pdx);
			EnableAudioCapture(pdx,index,1);
		}
		return -1;
	}
	CheckCardStatus(pdx);
	pdx->m_bAudioRun[index] = 1;
	pdx->m_bAudioStop[index] = 0;
	pdx->m_nAudioBufferIndex[index] =0;
	pdx->audio_data[index]=0;
	WRITE_ONCE(pdx->m_audio_pending[index], 0);
	spin_lock_irqsave(&pdx->audiolock[index], flags);
	pdx->m_nRDAudioIndex[index] =0;
	pdx->m_AudioInfo[index].m_nAudioIndex =0;
	for(j=0; j<MAX_AUDIO_QUEUE;j++)
	{
		pdx->m_AudioInfo[index].pStatusInfo[j].byLock = MEM_UNLOCK;
	}
	pdx->m_AudioInfo[index].dwisRuning =1;
	spin_unlock_irqrestore(&pdx->audiolock[index], flags);
	EnableAudioCapture(pdx,index,1);
	return 0;
}

static int StartVideoCapture(struct hws_pcie_dev *pdx,int index)
{
	int j;
	unsigned long flags;
	if(pdx->m_bVCapStarted[index]==1)
	{

		CheckCardStatus(pdx);
		if(CheckVideoCapture(pdx,index) ==0)
		{
			EnableVideoCapture(pdx,index,1);
		}
		return -1;
	}
	CheckCardStatus(pdx);
	spin_lock_irqsave(&pdx->videoslock[index], flags);
	for (j = 0; j<MAX_VIDEO_QUEUE; j++)
	{
		pdx->m_pVCAPStatus[index][j].byLock = MEM_UNLOCK;
		pdx->m_pVCAPStatus[index][j].byPath = 2;
		pdx->m_VideoInfo[index].pStatusInfo[j].byLock = MEM_UNLOCK;
	}
	pdx->m_nRDVideoIndex[index]=0;
	pdx->tstats[index].last_frame_ns = 0;
	pdx->tstats[index].frames_since_start = 0;
	pdx->tstats[index].lost_since_frame = false;
	pdx->tstats[index].interval_min = U64_MAX;
	pdx->tstats[index].window_start_ns = ktime_get_ns();
	{
		/* frames the card completed while capture was off (in flight at the
		 * stop, bringup): given up; the copy thread starts counting from here */
		u32 c0 = READ_ONCE(pdx->m_half_cnt[index][0]);

		atomic_add(c0 - pdx->m_half_done_cnt[index][0], &pdx->tstats[index].aborted);
		WRITE_ONCE(pdx->capture_starts[index], pdx->capture_starts[index] + 1);
		pdx->m_half_done_cnt[index][0] = c0;
		pdx->frame_open[index] = false;
		pdx->half1_drop[index] = NULL;
	}
	/* counted from the same moment, or the first report shows inflated rates */
	hws_tstat_fold(&pdx->tstats[index], NULL);
	pdx->tstats[index].last_half0_cnt = READ_ONCE(pdx->m_half_cnt[index][0]);
	pdx->m_VideoInfo[index].dwisRuning = 1;
	pdx->m_VideoInfo[index].m_nVideoIndex =0;
	pdx->m_nVideoHalfDone[index] = 0;
	WRITE_ONCE(pdx->m_half_pending[index], 0);
	pdx->direct_frame[index] = false;
	spin_unlock_irqrestore(&pdx->videoslock[index], flags);

	pdx->m_bChangeVideoSize[index] = 0;
	pdx->m_bVCapIntDone[index] = 1;
	pdx->m_pVideoEvent[index] = 1;
	pdx->m_nVideoBusy[index] =0;
	pdx->video_data[index]  =0;
	/* the frame-rate window starts with the capture, not at the last stop */
	atomic_set(&pdx->m_dwSWFrameRate[index], 0);
	pdx->m_sw_rate_t0[index] = ktime_get_ns();
	EnableVideoCapture(pdx,index,1);
	return 0;
}

static void StopVideoCapture(struct hws_pcie_dev *pdx,int index)
{

	if(pdx->m_bVCapStarted[index] ==0) return;
	pdx->m_VideoInfo[index].dwisRuning = 0;
	pdx->m_bVideoStop[index] = 1;
	pdx->m_pVideoEvent[index] = 0;
	pdx->m_bChangeVideoSize[index] = 0;
EnableVideoCapture(pdx,index,0);
pdx->m_bVCapIntDone[index] = 0;
}
static void StopAudioCapture(struct hws_pcie_dev *pdx,int index)
{
	if(pdx->m_bAudioRun[index] ==0) return;
	pdx->m_bAudioRun[index] = 0;
	pdx->m_bAudioStop[index] = 1;
	pdx->m_nAudioBufferIndex[index] =0;
	pdx->m_AudioInfo[index].dwisRuning =0;
EnableAudioCapture(pdx,index,0);
}


/* the "not yet rewritten" marker can sit at any byte offset for odd line pitches */
static inline u32 hws_get_mark(const void *p)
{
	u32 v;

	memcpy(&v, p, sizeof(v));
	return v;
}

static inline void hws_put_mark(void *p)
{
	u32 v = 0x55AAAA55;

	memcpy(p, &v, sizeof(v));
}

/*
 * Direct-to-buffer path. When no conversion is needed (packed YUYV, progressive,
 * input size == output size) the IRQ thread copies both halves straight from
 * DMA memory into the apps' buffers: one copy per frame instead of two, and
 * the frame is ready when its second half lands, without waiting for the
 * worker to be scheduled. Everything else uses the ring + worker.
 *
 * direct_bufs holds the buffers taken for the frame in flight (the first gets
 * the DMA copy, the others a copy of it on completion); direct_lock covers it
 * and the copies into it, so stop_streaming can take its buffers back.
 */

/* give the held buffers back to the front of their queues, unfilled */
static void hws_direct_requeue_locked(struct hws_pcie_dev *pdx, int ch, struct hws_vfh_ctx *only)
{
	struct hwsvideo_buffer *buf, *tmp;
	unsigned long flags;
	struct hwsvideo_buffer *first;

	/*
	 * The first buffer holds the first half of the frame in flight; if its
	 * owner leaves, nobody else has that half, so give back the whole frame.
	 */
	first = list_first_entry_or_null(&pdx->direct_bufs[ch], struct hwsvideo_buffer, queue);
	if (only && first && first->vb.vb2_buf.vb2_queue->drv_priv == only)
		only = NULL;

	list_for_each_entry_safe_reverse(buf, tmp, &pdx->direct_bufs[ch], queue) {
		struct hws_vfh_ctx *ctx = buf->vb.vb2_buf.vb2_queue->drv_priv;

		if (only && ctx != only)
			continue;
		spin_lock_irqsave(&ctx->qlock, flags);
		list_move(&buf->queue, &ctx->buf_queue);
		spin_unlock_irqrestore(&ctx->qlock, flags);
	}
}

static void hws_direct_reclaim(struct hws_pcie_dev *pdx, int ch, struct hws_vfh_ctx *ctx)
{
	rt_mutex_lock(&pdx->direct_lock[ch]);
	hws_direct_requeue_locked(pdx, ch, ctx);
	rt_mutex_unlock(&pdx->direct_lock[ch]);
}

/*
 * Fan-out of direct-path frames to further consumers. The IRQ thread copies
 * DMA into the first buffer only; copying that frame into the other consumers'
 * buffers (about 0.2 ms each at 1080p) runs here, so it never delays the
 * half-frame copies of the other channels. While a frame is in flight here,
 * later frames of the channel follow the same way, so each consumer still
 * sees frames in order.
 */
/*
 * Extra consumers' copies of direct-path frames. Runs on the channel's
 * conversion thread (RT), queued or called inline by the conversion work
 * itself, so frames keep their order without waiting on another thread.
 */
static void hws_fanout_drain(struct hws_video *v)
{
	struct hws_timing_stats *ts = &v->dev->tstats[v->index];
	struct hws_fanout *f;
	unsigned long flags;

	for (;;) {
		spin_lock_irqsave(&v->fanout_lock, flags);
		f = list_first_entry_or_null(&v->fanout_q, struct hws_fanout, node);
		if (f)
			list_del(&f->node);
		spin_unlock_irqrestore(&v->fanout_lock, flags);
		if (!f)
			break;

		if (hws_complete_buffers(&f->bufs, 1, f->out_size, f->ts, f->seq)) {
			atomic_inc(&ts->frames_out);
			atomic_inc(&ts->direct);
		} else {
			atomic_inc(&ts->no_app_buf);
		}

		spin_lock_irqsave(&v->fanout_lock, flags);
		list_add_tail(&f->node, &v->fanout_free);
		v->fanout_inflight--;
		spin_unlock_irqrestore(&v->fanout_lock, flags);
	}
}

static void hws_fanout_work(struct kthread_work *work)
{
	hws_fanout_drain(container_of(work, struct hws_video, fanout_work));
}

/* a copy thread serving extra apps delays other channels' copies on few cores */
static int hws_inline_fanout(void)
{
	return num_online_cpus() >= 8 ? HWS_INLINE_FANOUT : 1;
}

static int hws_list_count(struct list_head *list)
{
	struct list_head *pos;
	int n = 0;

	list_for_each(pos, list)
		n++;
	return n;
}

/* complete a direct-path frame (direct_lock held): inline for a few consumers
 * if nothing is queued ahead of it, else via the fan-out worker */
static void hws_direct_complete(struct hws_pcie_dev *pdx, int ch, struct list_head *bufs,
				int out_size, u64 frame_ts, u32 frame_seq)
{
	struct hws_video *v = &pdx->video[ch];
	struct hws_timing_stats *ts = &pdx->tstats[ch];
	struct hws_fanout *f = NULL;
	unsigned long flags;
	bool queued_ahead;

	spin_lock_irqsave(&v->fanout_lock, flags);
	queued_ahead = v->fanout_inflight > 0;
	/*
	 * A few apps: copy for them right here (~0.3 ms each at 1080p, far inside
	 * the copy deadline), so nobody waits for another thread. More apps, or
	 * older frames still being fanned out: the conversion thread does it.
	 */
	if ((queued_ahead || hws_list_count(bufs) > hws_inline_fanout()) && !list_empty(&v->fanout_free)) {
		f = list_first_entry(&v->fanout_free, struct hws_fanout, node);
		list_del(&f->node);
		list_splice_init(bufs, &f->bufs);
		f->out_size = out_size;
		f->ts = frame_ts;
		f->seq = frame_seq;
		list_add_tail(&f->node, &v->fanout_q);
		v->fanout_inflight++;
	}
	spin_unlock_irqrestore(&v->fanout_lock, flags);

	if (f) {
		kthread_queue_work(pdx->conv_worker[ch], &v->fanout_work);
		return;
	}
	if (queued_ahead) {
		/* fan-out worker is HWS_FANOUT_POOL frames behind: drop this frame
		 * rather than let it overtake the queued ones */
		hws_direct_requeue_locked(pdx, ch, NULL);
		atomic_inc(&ts->ring_skip);
		return;
	}
	if (hws_complete_buffers(bufs, 1, out_size, frame_ts, frame_seq)) {
		atomic_inc(&ts->frames_out);
		atomic_inc(&ts->direct);
	} else {
		atomic_inc(&ts->no_app_buf);
	}
}

/*
 * Interlaced input: bytes [off, off + len) of the field go to the frame with
 * every line doubled, exactly as SetDeInterlace() does. Half-frames aren't
 * line-aligned, so a range can start and end mid-line.
 */
static void hws_copy_doubled(BYTE *dst, const BYTE *src, int off, int len, int pitch, bool nt)
{
	while (len > 0)
	{
		int line = off / pitch, col = off % pitch;
		int n = min(len, pitch - col);
		BYTE *d = dst + (size_t)line * 2 * pitch + col;

		hws_dcopy(d, src, n, nt);
		hws_dcopy(d + pitch, src, n, nt);
		src += n;
		off += n;
		len -= n;
	}
}

/*
 * Card frames the driver drops still use up a sequence number, so apps see
 * the gap (V4L2: a gap in the sequence is a lost frame). Not while the
 * channel shows no signal: the frames then are the "no signal" pictures,
 * numbered as they go out. Copy thread only.
 */
static void hws_seq_skip(struct hws_pcie_dev *pdx, int ch, u32 n)
{
	unsigned long flags;

	if (n)
		pdx->tstats[ch].lost_since_frame = true;	/* the next gap is ours, not the card's */
	if (!n || READ_ONCE(pdx->m_curr_No_Video[ch]))
		return;
	spin_lock_irqsave(&pdx->videoslock[ch], flags);
	pdx->m_VideoInfo[ch].seq_counter += n;
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);
}

/*
 * Every frame the card completes (its second-half interrupt, card_frames in
 * the stats file) ends in exactly one counter: frames_in or one reason for
 * dropping it, so the stats add up and a frame that vanishes stands out. A
 * first half that fails is charged once its frame's second half arrives.
 * Copy thread only.
 */
static void hws_frame_drop(struct hws_pcie_dev *pdx, int ch, int half, atomic_t *why)
{
	if (half == 1)
	{
		pdx->half1_drop[ch] = why;
		return;
	}
	pdx->half1_drop[ch] = NULL;
	if (pdx->frame_open[ch])
	{
		pdx->frame_open[ch] = false;
		atomic_inc(why);
		hws_seq_skip(pdx, ch, 1);
	}
}

/*
 * Handle one half on the direct path. Returns -1 if the ring path must take
 * it, otherwise 1 (half consumed; a completed frame is already delivered).
 */
static int hws_direct_half(struct hws_pcie_dev *pdx, int ch, int half, BYTE *src,
			   int halfsize, int copysize, int nw, int nh, int interlace,
			   u32 cnt_self, u32 peer_at_irq, u64 t_irq)
{
	struct hws_video *videodev = &pdx->video[ch];
	ACAP_VIDEO_INFO *vi = &pdx->m_VideoInfo[ch];
	struct list_head *bufs = &pdx->direct_bufs[ch];
	struct hwsvideo_buffer *first;
	unsigned long flags;
	int out_size, ok, i;
	u64 frame_ts, t_copy;
	u32 frame_seq;

	rt_mutex_lock(&pdx->direct_lock[ch]);
	if (half == 1)
	{
		/* buffers of a frame whose second half never came go back first */
		hws_direct_requeue_locked(pdx, ch, NULL);
		pdx->direct_frame[ch] = false;
		if (!READ_ONCE(direct_io))
			goto use_ring;

		spin_lock_irqsave(&pdx->videoslock[ch], flags);
		out_size = videodev->current_out_width * videodev->curren_out_height * 2;
		/* interlaced: nh is the field height; each field line is shown twice */
		ok = pdx->m_Device_SupportYV12 == 0 &&
		     nw == videodev->current_out_width &&
		     (interlace ? 2 * nh : nh) == videodev->curren_out_height &&
		     !READ_ONCE(pdx->m_curr_No_Video[ch]) &&
		     halfsize > 0 && halfsize < (interlace ? out_size / 2 : out_size);
		/* frames still in the ring are older; deliver them first, in order */
		for (i = 0; ok && i < MAX_VIDEO_QUEUE; i++)
			if (vi->pStatusInfo[i].byLock != MEM_UNLOCK)
				ok = 0;
		spin_unlock_irqrestore(&pdx->videoslock[ch], flags);

		if (!ok || !hws_take_buffers(videodev, bufs))
			goto use_ring;
		first = list_first_entry(bufs, struct hwsvideo_buffer, queue);
		if (!first->mem || vb2_plane_size(&first->vb.vb2_buf, 0) < out_size)
		{
			/* let the worker deal with (and error out) odd buffers */
			hws_direct_requeue_locked(pdx, ch, NULL);
			goto use_ring;
		}
		pdx->direct_out_size[ch] = out_size;
		pdx->direct_il[ch] = interlace != 0;
		/* the same for both halves. Interlaced: each line written twice, line
		 * by line, where cached stores read every line first (~40% slower) */
		pdx->direct_nt[ch] = READ_ONCE(nt_copy) != 0 || (IS_ENABLED(CONFIG_X86) && interlace);
		pdx->direct_frame[ch] = true;
		t_copy = ktime_get_ns();
		if (interlace)
			hws_copy_doubled(first->mem, src, 0, halfsize, nw * 2, pdx->direct_nt[ch]);
		else
			hws_dcopy(first->mem, src, halfsize, pdx->direct_nt[ch]);
		hws_tstat_max(&pdx->tstats[ch].copy_dur_max, ktime_get_ns() - t_copy);
		if (pdx->direct_nt[ch])
			wmb();	/* the second half may be copied on another CPU, or the ring take the buffer */

		/* see the overrun check in MemCopyVideoToSteam() */
		smp_rmb();
		if (READ_ONCE(pdx->m_half_cnt[ch][0]) != peer_at_irq || hws_copy_too_late(pdx, ch, t_irq))
		{
			hws_frame_drop(pdx, ch, 1, &pdx->tstats[ch].dma_overrun);
			pdx->m_nVideoHalfDone[ch] = 0;
			hws_direct_requeue_locked(pdx, ch, NULL);
		}
		else
		{
			pdx->m_half1_cnt[ch] = cnt_self;
		}
		rt_mutex_unlock(&pdx->direct_lock[ch]);
		return 1;
	}

	/* second half: only if the first half went direct */
	if (!pdx->direct_frame[ch])
		goto use_ring;
	pdx->direct_frame[ch] = false;
	if (list_empty(bufs))
	{
		/* every consumer stopped mid-frame; the ring never saw the first half */
		rt_mutex_unlock(&pdx->direct_lock[ch]);
		return 1;
	}
	first = list_first_entry(bufs, struct hwsvideo_buffer, queue);
	out_size = pdx->direct_out_size[ch];
	if (pdx->direct_il[ch] != (interlace != 0) ||
	    (halfsize + copysize) * (interlace ? 2 : 1) != out_size)
	{
		hws_direct_requeue_locked(pdx, ch, NULL);	/* format moved under us */
		rt_mutex_unlock(&pdx->direct_lock[ch]);
		return 1;
	}
	t_copy = ktime_get_ns();
	if (interlace)
		hws_copy_doubled(first->mem, src, halfsize, copysize, nw * 2, pdx->direct_nt[ch]);
	else
		hws_dcopy(first->mem + halfsize, src, copysize, pdx->direct_nt[ch]);
	hws_tstat_max(&pdx->tstats[ch].copy_dur_max, ktime_get_ns() - t_copy);
	if (pdx->direct_nt[ch])
		wmb();	/* non-temporal stores are weakly ordered: done before the buffer is */
	smp_rmb();
	if (READ_ONCE(pdx->m_half_cnt[ch][1]) != peer_at_irq || hws_copy_too_late(pdx, ch, t_irq))
	{
		hws_frame_drop(pdx, ch, 0, &pdx->tstats[ch].dma_overrun);
		hws_direct_requeue_locked(pdx, ch, NULL);
		rt_mutex_unlock(&pdx->direct_lock[ch]);
		return 1;
	}

	frame_ts = t_irq;	/* this second half's interrupt */
	spin_lock_irqsave(&pdx->videoslock[ch], flags);
	frame_seq = vi->seq_counter++;
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);
	hws_tstat_frame_in(pdx, ch, frame_ts);
	/* apps that queued a buffer since the first half get a copy of this frame too */
	hws_take_late_buffers(videodev, bufs);
	hws_direct_complete(pdx, ch, bufs, out_size, frame_ts, frame_seq);
	rt_mutex_unlock(&pdx->direct_lock[ch]);
	return 1;

use_ring:
	rt_mutex_unlock(&pdx->direct_lock[ch]);
	return -1;
}

/*
 * Copy the half-frame the hardware just finished from the DMA buffer into the
 * ring. Runs in the IRQ thread. Returns 0 when a complete frame was
 * committed to the ring, non-zero otherwise.
 */
static int hws_copy_half(struct hws_pcie_dev *pdx, int nDecoder, int mVideoBufIndex);

/*
 * dma_probe: when exactly does the card rewrite a half? The copy deadline
 * (hws_copy_too_late) assumes the rewrite starts half a frame period after
 * the half's interrupt; this measures it. Right after a half is copied, marks
 * go into it at 0/8 .. 8/8 of its length, and the status thread watches when
 * each one gets overwritten. Diagnostics only: the status thread busy-waits
 * up to ~3 frame periods per channel every 2 s.
 */
static int dma_probe;
module_param(dma_probe, int, 0644);
MODULE_PARM_DESC(dma_probe, "Diagnostics: every 2 s, log when the card rewrites each part of a half-frame (default 0)");

static u32 hws_probe_off(u32 size, int k)
{
	if (k == HWS_PROBE_N - 1)
		return (size - 4) & ~3u;
	return (u32)div_u64((u64)size * k, HWS_PROBE_N - 1) & ~3u;
}

/* copy thread, once a half is copied: plant the marks the status thread asked for */
static void hws_probe_plant(struct hws_pcie_dev *pdx, int ch, int half)
{
	u32 hs, ds, size, mark = HWS_PROBE_MARK;
	unsigned long flags;
	u8 *base;
	int k;

	if (atomic_read(&pdx->probe_state[ch]) != 1 || READ_ONCE(pdx->m_curr_No_Video[ch]))
		return;
	spin_lock_irqsave(&pdx->videoslock[ch], flags);
	hs = pdx->m_format[ch].HLAF_SIZE;
	ds = pdx->m_format[ch].DWON_SIZE;
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);
	size = half ? hs : ds;
	if (size < 64 || (u64)hs + ds > pdx->m_MaxHWVideoBufferSize - MAX_AUDIO_CAP_SIZE)
		return;
	base = pdx->m_pbyVideoBuffer[ch] + (half ? 0 : hs);
	/*
	 * Only well before the card comes back to this half: a mark planted
	 * where it has already rewritten would stay there and show up in the
	 * next frame of this half.
	 */
	if (ktime_get_ns() - atomic64_read(&pdx->m_half_ts[ch][half]) > hws_frame_period_ns(pdx, ch) / 4)
		return;
	for (k = 0; k < HWS_PROBE_N; k++)
		memcpy(base + hws_probe_off(size, k), &mark, sizeof(mark));
	pdx->probe_base[ch] = base;
	pdx->probe_size[ch] = size;
	pdx->probe_half[ch] = half;
	pdx->probe_tirq[ch] = atomic64_read(&pdx->m_half_ts[ch][half]);
	smp_wmb();	/* marks and their place before the state */
	atomic_cmpxchg(&pdx->probe_state[ch], 1, 2);
}

/* status thread: one measurement per streaming channel */
static void hws_dma_probe(struct hws_pcie_dev *pdx)
{
	int ch, k;

	for (ch = 0; ch < pdx->m_nCurreMaxVideoChl; ch++)
	{
		u64 period = hws_frame_period_ns(pdx, ch), t0 = ktime_get_ns(), tirq, seen[HWS_PROBE_N] = { 0 };
		s64 other = 0, next = 0, o0, n0;
		char txt[HWS_PROBE_N * 8];
		int left = HWS_PROBE_N, len = 0, half;
		u32 size;
		u8 *base;

		if (!READ_ONCE(pdx->m_bVCapStarted[ch]) || READ_ONCE(pdx->m_curr_No_Video[ch]))
			continue;
		atomic_set(&pdx->probe_state[ch], 1);
		while (atomic_read(&pdx->probe_state[ch]) != 2)
		{
			if (ktime_get_ns() - t0 > 3 * period)
				goto next_ch;
			cond_resched();	/* not preemptible everywhere: let others run meanwhile */
		}
		smp_rmb();
		base = pdx->probe_base[ch];
		size = pdx->probe_size[ch];
		half = pdx->probe_half[ch];
		tirq = pdx->probe_tirq[ch];
		/* the first interrupt of each half after the one probed */
		o0 = atomic64_read(&pdx->m_half_ts[ch][!half]);
		n0 = atomic64_read(&pdx->m_half_ts[ch][half]);
		if (o0 > (s64)tirq)
			other = o0 - tirq;	/* already came before the marks were in */
		while (left && ktime_get_ns() - tirq < 2 * period + period / 2)
		{
			u64 now = ktime_get_ns();
			s64 v;

			if (!other && (v = atomic64_read(&pdx->m_half_ts[ch][!half])) != o0)
				other = v - tirq;
			if (!next && (v = atomic64_read(&pdx->m_half_ts[ch][half])) != n0)
				next = v - tirq;

			for (k = 0; k < HWS_PROBE_N; k++)
				if (!seen[k] && hws_get_mark(base + hws_probe_off(size, k)) != HWS_PROBE_MARK)
				{
					seen[k] = max_t(u64, now - tirq, 1);
					left--;
				}
			cond_resched();
		}
		for (k = 0; k < HWS_PROBE_N; k++)
			len += scnprintf(txt + len, sizeof(txt) - len, seen[k] ? " %llu" : " -",
					 div_u64(seen[k], NSEC_PER_USEC));
		pr_info("HwsCapture ch%d: dma probe, half %d (%u bytes), 0/8..8/8 rewritten after us:%s | other half's interrupt %lld us, this half's next %lld us, period %llu us\n",
			ch, half, size, txt,
			other > 0 ? div_s64(other, NSEC_PER_USEC) : -1LL,
			next > 0 ? div_s64(next, NSEC_PER_USEC) : -1LL,
			div_u64(period, NSEC_PER_USEC));
next_ch:
		atomic_set(&pdx->probe_state[ch], 0);
		cond_resched();
	}
}

static int MemCopyVideoToSteam(struct hws_pcie_dev *pdx, int nDecoder, int mVideoBufIndex)
{
	int ret = hws_copy_half(pdx, nDecoder, mVideoBufIndex);

	if (READ_ONCE(dma_probe))
		hws_probe_plant(pdx, nDecoder, mVideoBufIndex);

	/* a frame neither delivered nor dropped for a counted reason */
	if (mVideoBufIndex == 0 && pdx->frame_open[nDecoder])
		hws_frame_drop(pdx, nDecoder, 0, &pdx->tstats[nDecoder].aborted);
	return ret;
}

static int hws_copy_half(struct hws_pcie_dev *pdx, int nDecoder, int mVideoBufIndex)
{
	ACAP_VIDEO_INFO *vi = &pdx->m_VideoInfo[nDecoder];
	int nIndex = -1;
	int committed = 0;
	BYTE *bBuf = NULL;
	BYTE *pSrcBuf= NULL;
	BYTE *pDmaSrcBuf= NULL;
	int dwSrcPitch;
	int copysize;
	int nw,nh;
	int interlace;
	int halfsize;
	BYTE *pMask;
	int  line_cnt=0;
	int downsize;
	u32 gen, fmt_cnt0;
	bool fmt_armed;
	u32 cnt_self, peer_at_irq, prev_cnt;
	u64 t_irq;
	unsigned long flags;
	int i;

	/*
	 * Which completion of this half we are about to copy, and where the
	 * other half was then: one consistent snapshot. Bounded retries, as the
	 * writer may be preempted (forced-threaded IRQs on PREEMPT_RT); if it
	 * stays busy, a newer interrupt for this half is on its way anyway.
	 */
	for (i = 0; ; i++)
	{
		u32 seq = READ_ONCE(pdx->m_half_seq[nDecoder][mVideoBufIndex]);

		smp_rmb();
		cnt_self = READ_ONCE(pdx->m_half_cnt[nDecoder][mVideoBufIndex]);
		peer_at_irq = READ_ONCE(pdx->m_half_peer[nDecoder][mVideoBufIndex]);
		t_irq = atomic64_read(&pdx->m_half_ts[nDecoder][mVideoBufIndex]);
		smp_rmb();
		if (!(seq & 1) && READ_ONCE(pdx->m_half_seq[nDecoder][mVideoBufIndex]) == seq)
			break;
		if (i == 3)
			return -1;
		cpu_relax();
	}
	/*
	 * Already handled: an interrupt for this half came in after its pending
	 * bit was cleared but before the snapshot, so the newer completion was
	 * copied then and the bit is set again for it. Not a duplicate from the
	 * card (those have their own count); copying again would only trip the
	 * "not rewritten" mark and throw away the frame in progress.
	 */
	if (cnt_self == pdx->m_half_done_cnt[nDecoder][mVideoBufIndex])
		return -1;
	hws_tstat_max(&pdx->tstats[nDecoder].copy_wake_max, ktime_get_ns() - t_irq);
	if (mVideoBufIndex == 0)
	{
		/* second halves skipped since the last one: interrupts that merged
		 * while this thread was busy (a big jump: the channel was stopped) */
		u32 gap = cnt_self - pdx->m_half_done_cnt[nDecoder][0] - 1;

		if (gap && gap < 0x10000)
		{
			atomic_add(gap, &pdx->tstats[nDecoder].copy_missed);
			hws_seq_skip(pdx, nDecoder, gap);
		}
		pdx->frame_open[nDecoder] = true;
	}
	else
	{
		pdx->half1_drop[nDecoder] = NULL;	/* a new frame: an older reason doesn't apply */
	}
	prev_cnt = pdx->m_half_done_cnt[nDecoder][mVideoBufIndex];
	pdx->m_half_done_cnt[nDecoder][mVideoBufIndex] = cnt_self;

	/*
	 * A new first half: buffers a direct-path frame took for a second half
	 * that never came (dropped pairing, lost IRQ, mode change) go back to
	 * their apps now, before any check below can return early. Otherwise
	 * an app whose only free buffer sits there would never get another
	 * frame: "nobody has a buffer" would hold forever.
	 */
	if (mVideoBufIndex == 1 && READ_ONCE(pdx->direct_frame[nDecoder]))
	{
		rt_mutex_lock(&pdx->direct_lock[nDecoder]);
		hws_direct_requeue_locked(pdx, nDecoder, NULL);
		pdx->direct_frame[nDecoder] = false;
		rt_mutex_unlock(&pdx->direct_lock[nDecoder]);
	}

	/* no signal: the worker shows the "no signal" picture; nothing to copy */
	if (READ_ONCE(pdx->m_curr_No_Video[nDecoder]))
	{
		pdx->m_nVideoHalfDone[nDecoder] = 0;
		return -1;
	}

	/* one consistent view of the input format (see ChangeVideoSize) */
	spin_lock_irqsave(&pdx->videoslock[nDecoder], flags);
	nw = pdx->m_pVCAPStatus[nDecoder][0].dwWidth ;
	nh = pdx->m_pVCAPStatus[nDecoder][0].dwHeight;
	interlace  = pdx->m_pVCAPStatus[nDecoder][0].dwinterlace;
	halfsize = pdx->m_format[nDecoder].HLAF_SIZE;
	downsize = pdx->m_format[nDecoder].DWON_SIZE;
	gen = pdx->m_fmt_gen[nDecoder];
	fmt_cnt0 = pdx->m_fmt_cnt0[nDecoder];
	fmt_armed = pdx->m_fmt_armed[nDecoder];
	spin_unlock_irqrestore(&pdx->videoslock[nDecoder], flags);
	if((nw <=0)||(nh<=0)||(nw>MAX_VIDEO_HW_W)||(nh>MAX_VIDEO_HW_H))
	{
		return -1;
	}
	if(pdx->m_Device_SupportYV12 ==1)
	{
		dwSrcPitch = nw*12/8;
	}
	else
	{
		dwSrcPitch = nw*2;
	}
	if(mVideoBufIndex== 1)
	{
		pDmaSrcBuf = pdx->m_pbyVideoBuffer[nDecoder];
		copysize = halfsize;
	}
	else
	{
		pDmaSrcBuf = pdx->m_pbyVideoBuffer[nDecoder]+ halfsize;
		copysize = downsize;
	}
	line_cnt = copysize/dwSrcPitch;
	/* the status thread may change the format under us; never index outside the buffers */
	if (line_cnt < 2 || copysize <= 0 ||
	    halfsize + copysize > pdx->m_MaxHWVideoBufferSize - MAX_AUDIO_CAP_SIZE)
	{
		return -1;
	}
	pMask = pDmaSrcBuf+(copysize-(line_cnt-2)*dwSrcPitch);
	/* order the marker/data reads after the IRQ that announced the DMA */
	rmb();
	if(hws_get_mark(pMask) == 0x55AAAA55)
	{
		/*
		 * Half not rewritten since we last copied it (a duplicate interrupt),
		 * or our mark landed in new data while the card was rewriting this
		 * half. The two can't be told apart, so drop the frame in progress.
		 */
		hws_frame_drop(pdx, nDecoder, mVideoBufIndex, &pdx->tstats[nDecoder].stale_half);
		pdx->m_nVideoHalfDone[nDecoder] =0;
		return -1;
	}
	else
	{
		if(mVideoBufIndex== 0)
		{
			/* second half without a first half, or the input size changed in
			 * between (halves of different formats): drop it */
			/* ...or this second half doesn't belong to the first half we copied
			 * (an interrupt was missed or the copy fell a frame behind) */
			/* ...or the halves are far apart: interrupts were off in between
			 * (storm pause, card recovery) and the counts couldn't move */
			if(pdx->m_nVideoHalfDone[nDecoder]  ==0 || pdx->m_half1_gen[nDecoder] != gen ||
			   peer_at_irq != pdx->m_half1_cnt[nDecoder] ||
			   t_irq - pdx->m_half1_ts[nDecoder] >
			   max_t(u64, 2 * hws_frame_period_ns(pdx, nDecoder), 50 * NSEC_PER_MSEC))
			{
				atomic_t *why = pdx->half1_drop[nDecoder];

				hws_frame_drop(pdx, nDecoder, 0, why ? why : &pdx->tstats[nDecoder].unpaired);
				pdx->m_nVideoHalfDone[nDecoder] =0;
				hws_put_mark(pMask);
				return -1;
			}
			else
			{
				pdx->m_nVideoHalfDone[nDecoder] =0;
			}
		}
		else if (fmt_armed && (s32)(peer_at_irq - fmt_cnt0) <= 0)
		{
			/*
			 * The card started this first half before the new format was
			 * programmed (no second-half interrupt since ChangeVideoSize), so
			 * it is still in the old layout: skip it.
			 */
			pdx->m_nVideoHalfDone[nDecoder] = 0;
			hws_put_mark(pMask);
			return -1;
		}
		else
		{
			/*
			 * The first half before this one is still waiting for its second
			 * half, and no second-half interrupt came since: the card's
			 * interrupt for it was lost (the card wrote it). That frame is
			 * gone; apps see it as a gap in the sequence, not a late frame.
			 * Interrupts that came but merged count as copy_missed instead.
			 */
			if (pdx->m_nVideoHalfDone[nDecoder] && peer_at_irq == pdx->m_half1_peer[nDecoder])
			{
				atomic_inc(&pdx->tstats[nDecoder].irq_lost);
				hws_seq_skip(pdx, nDecoder, 1);
				/*
				 * Which kind of loss: about a period since the last first
				 * half means no interrupt came in between; about half a
				 * period means one came but the card's half register still
				 * named this half (then the first-half count moved by 2).
				 */
				if (timing_debug)
					pr_info_ratelimited("HwsCapture ch%d: interrupt lost: previous first half %llu us before, other half's last interrupt %lld us before, first-half interrupts since %u, period %llu us\n",
							    nDecoder, div_u64(t_irq - pdx->m_half1_ts[nDecoder], NSEC_PER_USEC),
							    div_s64((s64)t_irq - atomic64_read(&pdx->m_half_ts[nDecoder][0]), NSEC_PER_USEC),
							    cnt_self - prev_cnt,
							    div_u64(hws_frame_period_ns(pdx, nDecoder), NSEC_PER_USEC));
			}
			pdx->m_half1_peer[nDecoder] = peer_at_irq;
			pdx->m_nVideoHalfDone[nDecoder] = 1;
			pdx->half1_drop[nDecoder] = NULL;
			pdx->m_half1_gen[nDecoder] = gen;
			pdx->m_half1_ts[nDecoder] = t_irq;
			/* one check per format change: a counter compared for months
			 * would wrap (2^31 second halves = 414 days at 60 fps) */
			if (fmt_armed)
			{
				spin_lock_irqsave(&pdx->videoslock[nDecoder], flags);
				if (pdx->m_fmt_gen[nDecoder] == gen)
					pdx->m_fmt_armed[nDecoder] = false;
				spin_unlock_irqrestore(&pdx->videoslock[nDecoder], flags);
			}
		}
	}
	if(vi->dwisRuning ==1)
	{
		int dret = hws_direct_half(pdx, nDecoder, mVideoBufIndex, pDmaSrcBuf, halfsize,
					   copysize, nw, nh, interlace, cnt_self, peer_at_irq, t_irq);
		if (dret >= 0)
		{
			hws_put_mark(pMask);
			return dret;	/* direct path: nothing for the worker */
		}
	}
	if(vi->dwisRuning ==1)
	{
		pSrcBuf = pDmaSrcBuf;

		spin_lock_irqsave(&pdx->videoslock[nDecoder], flags);
		nIndex = vi->m_nVideoIndex;
		if(mVideoBufIndex== 1)
		{
			/*
			 * First half: claim the write slot. The worker may be reading
			 * one slot (MEM_READING); never write that one. A ready but
			 * unread slot (MEM_LOCK) is the oldest frame: drop it.
			 */
			if (vi->pStatusInfo[nIndex].byLock == MEM_READING)
			{
				nIndex = (nIndex + 1) % MAX_VIDEO_QUEUE;
				vi->m_nVideoIndex = nIndex;
			}
			if (vi->pStatusInfo[nIndex].byLock == MEM_LOCK)
			{
				atomic_inc(&pdx->tstats[nDecoder].ring_overwrite);
				vi->pStatusInfo[nIndex].byLock = MEM_UNLOCK;
			}
		}
		else if (vi->pStatusInfo[nIndex].byLock != MEM_UNLOCK)
		{
			/* slot was reset or taken since the first half: drop this frame */
			nIndex = -1;
		}
		spin_unlock_irqrestore(&pdx->videoslock[nDecoder], flags);

		if (nIndex >= 0)
			bBuf = vi->m_pVideoBufData[nIndex];
		if(bBuf)
		{
			u64 t_copy = ktime_get_ns();

			if(mVideoBufIndex== 0)
			{
				bBuf += halfsize;
			}
			memcpy(bBuf,pSrcBuf,copysize);
			hws_tstat_max(&pdx->tstats[nDecoder].copy_dur_max, ktime_get_ns() - t_copy);

			/*
			 * The card writes the halves alternately without pause. Once the
			 * other half has completed again, it is already rewriting this one,
			 * so what we copied may mix two frames. Drop it rather than show a
			 * torn picture; only happens when we fall about a half-frame behind.
			 */
			smp_rmb();
			if (READ_ONCE(pdx->m_half_cnt[nDecoder][!mVideoBufIndex]) != peer_at_irq ||
			    hws_copy_too_late(pdx, nDecoder, t_irq))
			{
				hws_frame_drop(pdx, nDecoder, mVideoBufIndex, &pdx->tstats[nDecoder].dma_overrun);
				pdx->m_nVideoHalfDone[nDecoder] = 0;
				hws_put_mark(pMask);
				return 1;
			}
			if (mVideoBufIndex == 1)
				pdx->m_half1_cnt[nDecoder] = cnt_self;

			if(mVideoBufIndex== 0)
			{
				spin_lock_irqsave(&pdx->videoslock[nDecoder], flags);

				vi->m_nVideoIndex = nIndex+1;
				if(vi->m_nVideoIndex >= MAX_VIDEO_QUEUE)
				{
					vi->m_nVideoIndex =0;
				}
				vi->pStatusInfo[nIndex].dwWidth = nw;
				vi->pStatusInfo[nIndex].dwHeight = nh;
				vi->pStatusInfo[nIndex].dwinterlace = interlace;
				/* the frame is complete when its second half's IRQ fired */
				vi->frame_ts[nIndex] = t_irq;
				vi->frame_seq[nIndex] = vi->seq_counter++;
				vi->pStatusInfo[nIndex].byLock = MEM_LOCK;

				spin_unlock_irqrestore(&pdx->videoslock[nDecoder], flags);
				hws_tstat_frame_in(pdx, nDecoder, vi->frame_ts[nIndex]);
				committed = 1;
			}

		}
		else
		{
			pdx->m_nVideoHalfDone[nDecoder] =0;
		}
	}
	hws_put_mark(pMask);
	return committed ? 0 : 1;
}
static int MemCopyAudioToSteam( struct hws_pcie_dev  *pdx,int dwAudioCh,int half)
{
	int i=0;
	BYTE *bBuf = NULL;
	BYTE *pSrcBuf= NULL;
	int nIndex = -1;
	unsigned long flags;
	if(half== 0)
	{

		pSrcBuf = pdx->m_pbyAudioBuffer[dwAudioCh]+pdx->m_dwAudioPTKSize;
	}
	else
	{
		pSrcBuf =  pdx->m_pbyAudioBuffer[dwAudioCh];
	}

	nIndex = -1;
	if(pdx->m_AudioInfo[dwAudioCh].dwisRuning ==1)
	{
		/* only free slots; the worker never gives back a slot it is still reading */
		spin_lock_irqsave(&pdx->audiolock[dwAudioCh], flags);
		for( i = pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex;i<MAX_AUDIO_QUEUE;i++)
		{
			if(pdx->m_AudioInfo[dwAudioCh].pStatusInfo[i].byLock== MEM_UNLOCK)
			{
				nIndex =i;
				bBuf = pdx->m_AudioInfo[dwAudioCh].m_pAudioBufData[i];
				break;
			}
		}
		if(nIndex == -1)
		{
			for( i = 0 ;i<pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex;i++)
			{
				if(pdx->m_AudioInfo[dwAudioCh].pStatusInfo[i].byLock== MEM_UNLOCK)
				{
					nIndex =i;
					bBuf = pdx->m_AudioInfo[dwAudioCh].m_pAudioBufData[i];
					break;
				}

			}
		}
		spin_unlock_irqrestore(&pdx->audiolock[dwAudioCh], flags);

		if((nIndex!= -1)&& bBuf)
		{
			rmb();
			memcpy(bBuf, pSrcBuf, pdx->m_dwAudioPTKSize);

			pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex = nIndex+1;
			if(pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex>= MAX_AUDIO_QUEUE)
			{
				pdx->m_AudioInfo[dwAudioCh].m_nAudioIndex =0;
			}
			spin_lock_irqsave(&pdx->audiolock[dwAudioCh], flags);
			pdx->m_AudioInfo[dwAudioCh].pStatusInfo[nIndex].dwLength = pdx->m_dwAudioPTKSize ;
			pdx->m_AudioInfo[dwAudioCh].pStatusInfo[nIndex].byLock = MEM_LOCK;
			spin_unlock_irqrestore(&pdx->audiolock[dwAudioCh], flags);
			/* deliver right here in the IRQ thread: no extra wakeup per packet */
			audio_data_process(&pdx->audio[dwAudioCh].audiowork);
		}
		else
		{
			pr_warn_ratelimited("HwsCapture: audio ch%d ring full, packet dropped\n", dwAudioCh);
		}
	}
	return 0;
}

static int SetAudioQuene( struct hws_pcie_dev *pdx,int dwAudioCh,int half)
{
	int status =-1;
	if(!pdx->m_bACapStarted[dwAudioCh])
	{
		return -1 ;
	}
	if(!pdx->m_bAudioRun[dwAudioCh])
	{
		if(pdx->m_bAudioStop[dwAudioCh] == 1)
		{
			pdx->m_bAudioStop[dwAudioCh] =0;
		}
		pdx->m_nAudioBusy[dwAudioCh] =0;
		return status;
	}

	pdx->m_nAudioBusy[dwAudioCh]  = 1;

	status = MemCopyAudioToSteam(pdx,dwAudioCh,half);


	pdx->m_nAudioBusy[dwAudioCh] = 0;


	return status;

}

/* IRQ thread: copy every finished audio half, oldest first */
static void hws_audio_dpc(struct hws_pcie_dev *pdx, int index)
{
	unsigned long pending;
	int half;

	while ((pending = READ_ONCE(pdx->m_audio_pending[index])) != 0)
	{
		cond_resched();	/* an interrupt storm must not monopolise the CPU */
		smp_rmb();	/* pairs with smp_mb__before_atomic() in the IRQ handler */
		if (pending == 3)
			half = (s64)(atomic64_read(&pdx->m_audio_half_ts[index][1]) -
				     atomic64_read(&pdx->m_audio_half_ts[index][0])) <= 0 ? 1 : 0;
		else
			half = (pending & 2) ? 1 : 0;
		clear_bit(half, &pdx->m_audio_pending[index]);
		SetAudioQuene(pdx, index, half);
	}
}
static int SetQuene(struct hws_pcie_dev  *pdx,int nDecoder,int half)
{
	int status =-1;
	if(!pdx->m_bStartRun)
	{
		return -1 ;
	}
	if(!pdx->m_bVCapStarted[nDecoder])
	{
		if(pdx->m_bVideoStop[nDecoder] == 1)
		{
			pdx->m_bVideoStop[nDecoder] =0;
		}

		return -1 ;
	}
	if(pdx->m_DeviceHW_Version ==0)
	{
		atomic_inc(&pdx->m_dwSWFrameRate[nDecoder]);
	}
	pdx->m_nVideoBusy[nDecoder] = 1;
	if(pdx->m_bVCapStarted[nDecoder] == TRUE)
	{
		status = MemCopyVideoToSteam(pdx,nDecoder,half);
	}
	pdx->m_nVideoBusy[nDecoder] = 0;
	return status;
}

/*
 * IRQ thread: copy the finished half-frames; wake the worker once a whole
 * frame is in. Each half has its own pending bit, so two IRQs that arrive
 * before the thread runs are both processed, oldest first.
 */
static void hws_video_dpc(struct hws_pcie_dev *pdx, int i)
{
	unsigned long pending;
	int half;

	while ((pending = READ_ONCE(pdx->m_half_pending[i])) != 0)
	{
		cond_resched();	/* an interrupt storm must not monopolise the CPU */
		smp_rmb();	/* pairs with smp_mb__before_atomic() in the IRQ handler */
		if (pending == 3)
			half = (s64)(atomic64_read(&pdx->m_half_ts[i][1]) -
				     atomic64_read(&pdx->m_half_ts[i][0])) <= 0 ? 1 : 0;
		else
			half = (pending & 2) ? 1 : 0;
		clear_bit(half, &pdx->m_half_pending[i]);

		if (SetQuene(pdx, i, half) != 0)
			continue;

		if (pdx->m_bVCapStarted[i] == TRUE && pdx->m_pVideoEvent[i])
		{
			pdx->m_bVCapIntDone[i] = FALSE;
			if (pdx->m_bChangeVideoSize[i])
			{
				pdx->m_bChangeVideoSize[i] = 0;
				continue;
			}
			/* coalescing is harmless: the worker drains every ready slot */
			if (!kthread_queue_work(pdx->conv_worker[i], &pdx->video[i].videowork))
				atomic_inc(&pdx->tstats[i].wq_coalesced);
		}
	}
}
/*
 * Hard IRQ: acknowledge every source, note which DMA halves finished and when,
 * and wake the IRQ thread. The multi-megabyte copies run in the thread
 * (SCHED_FIFO, preemptible) instead of softirq context, so they no longer
 * hold up network, timer and block softirqs on the CPU taking this IRQ.
 */
static irqreturn_t irqhandler(int irq, void  *info)
{
	struct hws_pcie_dev *pdx = (struct hws_pcie_dev *)(info);
	u32 dma_status;
	u32 Int_Value;
	u32 IntState;
	u32 tmp;
	u32 cnt;
	u32 before;
	int i;
	bool wake = false;	/* audio: run the IRQ thread */
	bool ours = false;
	bool stuck = false;

	/*
	 * Card lost: a shared line's neighbour fired. Its registers aren't read
	 * until it is back on the bus (some platforms fault on reads of a gone
	 * device); the config-space check is atomic-safe and only runs then.
	 */
	if (READ_ONCE(pdx->m_PciDeviceLost))
	{
		u16 cmd;

		/* back after a reset, it doesn't decode memory until the status
		 * thread restores its config */
		if (READ_ONCE(pdx->removing) || !pci_device_is_present(pdx->pdev) ||
		    pci_read_config_word(pdx->pdev, PCI_COMMAND, &cmd) || !(cmd & PCI_COMMAND_MEMORY))
			return IRQ_NONE;
	}
	dma_status =  READ_REGISTER_ULONG(pdx,(u32)(CVBS_IN_BASE));
	if((dma_status == 0xffffffff) || ((dma_status&0x04) != 0x04))
	{
		return IRQ_NONE;
	}
	IntState= READ_REGISTER_ULONG(pdx,(u32)(CVBS_IN_BASE + 1 * PCIE_BARADDROFSIZE));
	if((IntState == 0) || (IntState == 0xffffffff))
	{
		return IRQ_NONE;
	}
	for(cnt =0; cnt <100 && IntState != 0 && IntState != 0xffffffff; cnt ++)
	{
		/* bits to acknowledge in this pass only */
		Int_Value = 0;
		/* any source we acknowledge is ours (also bits 16/17): returning
		 * IRQ_NONE for them would get the line disabled as "nobody cared" */
		if (IntState & 0x00ffffff)
			ours = true;
		for (i = 0; i < MAX_VID_CHANNELS; i++)
		{
			if(IntState & (0x01 << i)) // video ch done
			{
				Int_Value |= 0x01 << i;
				pdx->m_bVCapIntDone[i] = 1;
				if(pdx->m_nVideoBusy[i])
					atomic_inc(&pdx->tstats[i].irq_busy_skip);
				tmp = (READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE + (32+i) * PCIE_BARADDROFSIZE)))&0x01;
				/*
				 * Every completion counts, also the same half twice in a row:
				 * that means the other half's interrupt was lost and this half
				 * holds new data. Copying it keeps the next frame intact (the
				 * pairing checks see the counts); a duplicate without new data
				 * is caught by the "not rewritten" mark in MemCopyVideoToSteam.
				 */
				{
					u64 now = ktime_get_ns();

					pdx->video_data[i]= tmp;
					pdx->m_nVideoBufferIndex[i] = tmp;
					/* the copy thread reads ts/cnt/peer as one snapshot */
					WRITE_ONCE(pdx->m_half_seq[i][tmp], pdx->m_half_seq[i][tmp] + 1);
					smp_wmb();
					atomic64_set(&pdx->m_half_ts[i][tmp], now);
					WRITE_ONCE(pdx->m_half_cnt[i][tmp], pdx->m_half_cnt[i][tmp] + 1);
					WRITE_ONCE(pdx->m_half_peer[i][tmp], pdx->m_half_cnt[i][!tmp]);
					smp_wmb();
					WRITE_ONCE(pdx->m_half_seq[i][tmp], pdx->m_half_seq[i][tmp] + 1);
					smp_mb__before_atomic();	/* ts/counts visible before the bit */
					set_bit(tmp, &pdx->m_half_pending[i]);
					kthread_queue_work(pdx->copy_worker[i], &pdx->video[i].copy_work);
				}
			}
			if(IntState & (0x100 << i)) // audio ch done
			{
				Int_Value |= 0x100 << i;
				tmp = (READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE + (40+i) * PCIE_BARADDROFSIZE)))&0x01;
				pdx->m_nAudioBufferIndex[i] = tmp;
				pdx->audio_data[i]= tmp;
				atomic64_set(&pdx->m_audio_half_ts[i][tmp], ktime_get_ns());
				smp_mb__before_atomic();
				set_bit(tmp, &pdx->m_audio_pending[i]);
				wake = true;
			}
		}
		/*
		 * Acknowledge every source that is set, also ones we don't handle
		 * (e.g. bits 16/17, enabled by INT_EN): an unacknowledged level
		 * interrupt would re-fire forever.
		 */
		Int_Value = (Int_Value | IntState) & 0x00ffffff;
		before = IntState;
		WRITE_REGISTER_ULONG(pdx,(u32)(0x4000 + (PCIE_BARADDROFSIZE * 1)), Int_Value);
		IntState= READ_REGISTER_ULONG(pdx,(u32)(CVBS_IN_BASE + 1 * PCIE_BARADDROFSIZE));
		if (IntState == before)
		{
			/* the acknowledge had no effect: a stuck bit, don't spin on it */
			stuck = true;
			break;
		}
	}

	/*
	 * Storm guard: a status bit that never clears keeps a level-triggered
	 * line asserted. After many stuck passes in a row switch the card's
	 * interrupts off (no lock here: hard IRQ context, also on PREEMPT_RT);
	 * the status thread switches them back on a second later.
	 */
	if (!stuck && cnt < 100)
	{
		pdx->irq_stuck_passes = 0;
	}
	else
	{
		/*
		 * A storm is many stuck interrupts close together (1000 within
		 * 100 ms, i.e. >= 10k/s). Normal traffic with a bit that never
		 * clears (MSI) stays far below that, ~500/s at 4 channels; a level
		 * line that re-fires on a stuck bit - also a video bit, which
		 * looks "handled" every time - reaches it within milliseconds.
		 */
		u64 now = ktime_get_ns();

		if (now - pdx->irq_stuck_t0 > 100 * NSEC_PER_MSEC)
		{
			pdx->irq_stuck_t0 = now;
			pdx->irq_stuck_passes = 0;
		}
		/*
		 * A stuck bit counts once per call; a status that keeps changing ran
		 * the whole loop: count its passes, or a storm of 100-pass calls
		 * (~160 us each) never reaches the limit.
		 */
		if (READ_ONCE(pdx->irq_storm_off))
			return IRQ_NONE;	/* paused and still storming: the kernel's spurious-IRQ guard decides */
		pdx->irq_stuck_passes += stuck ? 1 : cnt;
		if (pdx->irq_stuck_passes >= 1000)
		{
			WRITE_REGISTER_ULONG(pdx, INT_EN_REG_BASE, 0);
			atomic64_set(&pdx->irq_storm_off_ns, ktime_get_ns());
			smp_store_release(&pdx->irq_storm_off, true);	/* time visible before the flag */
			pdx->irq_stuck_passes = 0;
			printk_ratelimited(KERN_WARNING "HwsCapture: interrupt status stuck at %08x, card interrupts paused\n", IntState);
		}
	}

	/* nothing the card could have raised: let the kernel's spurious-interrupt handling see it */
	if (!ours)
		return IRQ_NONE;
	/* a wake while the thread runs makes it run again, so nothing is lost */
	return wake ? IRQ_WAKE_THREAD : IRQ_HANDLED;
}

static irqreturn_t hws_irq_thread(int irq, void *info)
{
	struct hws_pcie_dev *pdx = (struct hws_pcie_dev *)(info);
	int i;

	/* audio only: video half-frames are copied by each channel's copy thread */
	for (i = 0; i < MAX_VID_CHANNELS; i++)
		hws_audio_dpc(pdx, i);
	return IRQ_HANDLED;
}

/* per-channel RT copy thread: up to 2 MB per half-frame */
static void hws_copy_work(struct kthread_work *work)
{
	struct hws_video *v = container_of(work, struct hws_video, copy_work);

	hws_video_dpc(v->dev, v->index);
}

static struct hws_pcie_dev *alloc_dev_instance(struct pci_dev *pdev)
{
	struct hws_pcie_dev *lro;

	BUG_ON(!pdev);

	/* allocate zeroed device book keeping structure */
	lro = kzalloc(sizeof(struct hws_pcie_dev), GFP_KERNEL);
	if (!lro)
		return NULL;

	lro->irq_line = -1;

	/* create a device to driver reference */
	dev_set_drvdata(&pdev->dev, lro);
	/* create a driver to device reference; held until the last file is closed
	 * (querycap and vb2 use pdev after an unbind) */
	lro->pdev = pci_dev_get(pdev);

	/* Set up data user IRQ data structures */

	return lro;
}


static void SetDMAAddress(struct hws_pcie_dev *pdx)
{

	u32 Addrmsk;
	u32 AddrLowmsk;
	u32 PhyAddr_A_Low;
	u32 PhyAddr_A_High;


	u32 PCI_Addr;
	u32 cnt;
	int i = 0;
	DWORD halfframeLength=0;
	PhyAddr_A_High = 0;
	PhyAddr_A_Low=0;
	PCI_Addr =0;


	//------------------------------------------ // re write dma register

	Addrmsk = PCI_E_BAR_ADD_MASK;
	AddrLowmsk = PCI_E_BAR_ADD_LOWMASK;


	cnt = 0x208;  // Table address
	for (i = 0; i< pdx->m_nMaxChl; i++)
	{
		if (pdx->m_pbyVideoBuffer[i])
		{
			PhyAddr_A_Low = pdx->m_dwVideoBuffer[i];
			PhyAddr_A_High = pdx->m_dwVideoHighBuffer[i];

			PCI_Addr = (PhyAddr_A_Low&AddrLowmsk);
			PhyAddr_A_Low = (PhyAddr_A_Low&Addrmsk);


			WRITE_REGISTER_ULONG(pdx,(PCI_ADDR_TABLE_BASE + cnt),PhyAddr_A_High);
			WRITE_REGISTER_ULONG(pdx,(PCI_ADDR_TABLE_BASE + cnt+PCIE_BARADDROFSIZE),PhyAddr_A_Low);  //Entry 0
			(void)READ_REGISTER_ULONG(pdx,(PCI_ADDR_TABLE_BASE + cnt));
			(void)READ_REGISTER_ULONG(pdx,(PCI_ADDR_TABLE_BASE + cnt+PCIE_BARADDROFSIZE));


			WRITE_REGISTER_ULONG(pdx,( CBVS_IN_BUF_BASE + (i*PCIE_BARADDROFSIZE)), ((i+1)*PCIEBAR_AXI_BASE)+PCI_Addr); //Buffer 1 address
			halfframeLength = pdx->m_format[i].HLAF_SIZE/16;
			WRITE_REGISTER_ULONG(pdx,( CBVS_IN_BUF_BASE2 + (i*PCIE_BARADDROFSIZE)),halfframeLength); //Buffer 1 address


			(void)READ_REGISTER_ULONG(pdx,(  CBVS_IN_BUF_BASE + (i*PCIE_BARADDROFSIZE)));
			(void)READ_REGISTER_ULONG(pdx,(  CBVS_IN_BUF_BASE2 + (i*PCIE_BARADDROFSIZE)));


		}
		cnt +=8;
		if(pdx->m_pbyAudioBuffer[i])
		{
			PhyAddr_A_Low = pdx->m_dwAudioBuffer[i];
			PhyAddr_A_High = pdx->m_dwAudioBufferHigh[i];
			PCI_Addr = (PhyAddr_A_Low&AddrLowmsk);
			PhyAddr_A_Low = (PhyAddr_A_Low&Addrmsk);
			WRITE_REGISTER_ULONG(pdx,(CBVS_IN_BUF_BASE + ((8+i)*PCIE_BARADDROFSIZE)), ((i+1)*PCIEBAR_AXI_BASE+PCI_Addr)); //Buffer 1 address
			(void)READ_REGISTER_ULONG(pdx,(CBVS_IN_BUF_BASE + ((8+i)*PCIE_BARADDROFSIZE)));
		}
	}
	hws_enable_card_irqs(pdx); //enable PCI Interruput

}

static void ChangeVideoSize(struct hws_pcie_dev *pdx,int ch,int w,int h,int interlace)
{
	int j;
	int halfframeLength;
	unsigned long flags;
	/* size, half sizes and generation change together; the copy path snapshots them under this lock */
	spin_lock_irqsave(&pdx->videoslock[ch], flags);
	for (j = 0; j<MAX_VIDEO_QUEUE; j++)
	{
		pdx->m_pVCAPStatus[ch][j].dwWidth = w ;
		pdx->m_pVCAPStatus[ch][j].dwHeight = h;
		pdx->m_pVCAPStatus[ch][j].dwinterlace = interlace;

	}
	SetVideoFormteSize(pdx,ch,w,h);
	pdx->m_fmt_gen[ch]++;
	/* first halves are in the new layout only once another second half has come */
	pdx->m_fmt_cnt0[ch] = READ_ONCE(pdx->m_half_cnt[ch][0]);
	pdx->m_fmt_armed[ch] = true;
	halfframeLength = pdx->m_format[ch].HLAF_SIZE/16;
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);
	/* SetDMAAddress (CheckCardStatus) writes the same register under reg_lock */
	spin_lock_irqsave(&pdx->reg_lock, flags);
	WRITE_REGISTER_ULONG(pdx,(DWORD)(CBVS_IN_BUF_BASE2 + (ch*PCIE_BARADDROFSIZE)), halfframeLength); //Buffer 1 address
	spin_unlock_irqrestore(&pdx->reg_lock, flags);
	/* again: a second half that came before the new length reached the card
	 * proves nothing, and a first half accepted in between must not pair */
	spin_lock_irqsave(&pdx->videoslock[ch], flags);
	pdx->m_fmt_gen[ch]++;
	pdx->m_fmt_cnt0[ch] = READ_ONCE(pdx->m_half_cnt[ch][0]);
	pdx->m_fmt_armed[ch] = true;
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);
}

/* tell subscribed apps (GStreamer, PipeWire, ...) that the input changed */
static void hws_notify_source_change(struct hws_video *videodev)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	if (video_is_registered(&videodev->vdev))
		v4l2_event_queue(&videodev->vdev, &ev);
}

/*
 * New input size: if no application holds buffers, make it the default
 * output size, so an app that keeps the default gets the input as it is
 * (direct path, no scaling). Only sizes the driver offers.
 */
static void hws_follow_input(struct hws_video *videodev, int w, int h)
{
	struct hws_pcie_dev *pdx = videodev->dev;
	v4l2_model_timing_t *t;
	unsigned long flags;
	int i;

	for (i = 0; (t = v4l2_model_get_support_videoformat(i)) != NULL; i++)
		if (t->frame_size.width == w && t->frame_size.height == h)
			break;
	if (!t)
		return;
	mutex_lock(&videodev->video_lock);
	/* not under an application that chose its format or has buffers */
	if (!videodev->fmt_by_app && !hws_video_busy(videodev))
	{
		spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
		videodev->current_out_size_index = i;
		videodev->current_out_width = w;
		videodev->curren_out_height = h;
		spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);
	}
	mutex_unlock(&videodev->video_lock);
}

/*
 * Every few seconds of capture: the input's exact rate from the frame
 * intervals, snapped to a standard rate (59.94 and 60 differ by 0.1%; the
 * mean over a few hundred frames is good to ~0.01%). Taken only when two
 * windows in a row agree: interrupts held off for milliseconds at a window's
 * edge shift one mean by about that 0.1%.
 */
static void hws_measure_rate(struct hws_pcie_dev *pdx, int ch)
{
	static const struct { u32 mhz, num, den; } std[] = {
		{ 60000, 1, 60 }, { 59940, 1001, 60000 }, { 50000, 1, 50 },
		{ 30000, 1, 30 }, { 29970, 1001, 30000 }, { 25000, 1, 25 },
		{ 24000, 1, 24 }, { 23976, 1001, 24000 },
	};
	struct hws_timing_stats *ts = &pdx->tstats[ch];
	struct hws_video *v = &pdx->video[ch];
	u64 acc, sum, mhz, tpf;
	u32 n;
	int i;

	if ((atomic64_read(&ts->rate_acc) >> RATE_N_SHIFT) < 200)
		return;
	acc = atomic64_xchg(&ts->rate_acc, 0);
	n = acc >> RATE_N_SHIFT;
	sum = acc & (BIT_ULL(RATE_N_SHIFT) - 1);
	if (!n || !sum)
		return;
	mhz = div64_u64((u64)n * 1000000000000ULL, sum);
	for (i = 0; i < ARRAY_SIZE(std); i++)
		if (mhz * 2500 > (u64)std[i].mhz * 2499 && mhz * 2500 < (u64)std[i].mhz * 2501)
		{
			/* within 0.04% */
			tpf = (u64)std[i].num << 32 | std[i].den;

			if (ts->rate_cand == tpf)
				atomic64_set(&v->tpf, tpf);
			ts->rate_cand = tpf;
			return;
		}
	ts->rate_cand = 0;
	/*
	 * Nothing standard. Close to what was measured before: a burst of jitter
	 * spoiled this window, keep it. Far off: the source changed its rate
	 * (75, 120 Hz) - back to the card's own figure (0).
	 */
	tpf = atomic64_read(&v->tpf);
	if (tpf && (u32)tpf)
	{
		u64 old_mhz = div64_u64((u64)(u32)tpf * 1000ULL, tpf >> 32);	/* den/num Hz */

		if (mhz * 200 < old_mhz * 199 || mhz * 200 > old_mhz * 201)
			atomic64_set(&v->tpf, 0);
	}
}

/*
 * The measured rate against the card's own whole-number figure (newer
 * hardware reads it from the receiver): more than 1.5 fps apart, the source
 * changed its rate. Interlaced, the card may count fields or frames.
 */
static bool hws_rate_matches_card(struct hws_pcie_dev *pdx, int ch, u64 tpf)
{
	u64 card = (u64)pdx->m_pVCAPStatus[ch][0].dwFrameRate * 1000, mhz;

	if (!tpf || !(u32)tpf || !card)
		return true;	/* nothing to compare */
	mhz = div64_u64((u64)(u32)tpf * 1000ULL, tpf >> 32);	/* den/num Hz, x1000 */
#define HWS_NEAR(a, b)	((a) + 1500 >= (b) && (a) <= (b) + 1500)
	if (HWS_NEAR(mhz, card))
		return true;
	if (pdx->m_pVCAPStatus[ch][0].dwinterlace && (HWS_NEAR(2 * mhz, card) || HWS_NEAR(mhz, 2 * card)))
		return true;
#undef HWS_NEAR
	return false;
}

/*
 * Signal lost or back: measure afresh, but keep the rate measured before -
 * usually the same source returns after a dropout, and the exact rate
 * (59.94) is known at once instead of 60 for the seconds a measurement takes.
 * Not on the oldest hardware (its figure is counted from frames, stale after a
 * dropout); a different rate is caught by hws_rate_check() on every poll, and
 * a new mode (ChangeVideoSize: hws_reset_rate) starts over anyway.
 */
static void hws_rate_signal_change(struct hws_pcie_dev *pdx, int ch, bool back)
{
	atomic64_set(&pdx->tstats[ch].rate_acc, 0);
	pdx->tstats[ch].rate_cand = 0;
	if (back && (pdx->m_DeviceHW_Version == 0 ||
		     !hws_rate_matches_card(pdx, ch, atomic64_read(&pdx->video[ch].tpf))))
		atomic64_set(&pdx->video[ch].tpf, 0);
}

/* with a signal, each poll: a kept or measured rate the card contradicts goes */
static void hws_rate_check(struct hws_pcie_dev *pdx, int ch)
{
	if (pdx->m_DeviceHW_Version > 0 &&
	    !hws_rate_matches_card(pdx, ch, atomic64_read(&pdx->video[ch].tpf)))
	{
		atomic64_set(&pdx->video[ch].tpf, 0);
		atomic64_set(&pdx->tstats[ch].rate_acc, 0);
		pdx->tstats[ch].rate_cand = 0;
	}
}

/* new mode: the old measurement doesn't apply */
static void hws_reset_rate(struct hws_pcie_dev *pdx, int ch)
{
	atomic64_set(&pdx->video[ch].tpf, 0);
	atomic64_set(&pdx->tstats[ch].rate_acc, 0);
	pdx->tstats[ch].rate_cand = 0;
}

static int Get_Video_Status(struct hws_pcie_dev *pdx,unsigned int  ch)
{
	int value;
	int out_value=0;
	int res_w=0;
	int res_h=0;
	int frame_rate=0;
	int active_video=1;
	int interlace=0;
	int offset;
	int no_video;
	int video_hdcp;
	unsigned char br;
	unsigned char co;
	unsigned char hu;
	unsigned char sa;
	int out_res_w=0;
	int out_res_h=0;
	int out_frame_rate=0;

	value =  READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (5*PCIE_BARADDROFSIZE)));
	if ((u32)value == 0xFFFFFFFF)
	{
		/* card gone (surprise removal, link down): after a few reads in a row stop
		 * touching it and show "no signal"; a single glitch doesn't count */
		if (pdx->m_lost_reads < 3 && ++pdx->m_lost_reads == 3)
		{
			WRITE_ONCE(pdx->m_PciDeviceLost, 1);
			dev_warn(&pdx->pdev->dev, "card stopped responding\n");
		}
		/* gone: "no signal"; a glitch so far: keep what we had */
		return READ_ONCE(pdx->m_PciDeviceLost) ? 1 : READ_ONCE(pdx->m_card_No_Video[ch]);
	}
	pdx->m_lost_reads = 0;
	if (READ_ONCE(pdx->m_PciDeviceLost) && !READ_ONCE(pdx->suspended))
	{
		/* it is back (link retrained); CheckVideFmt re-arms the streaming channels */
		WRITE_ONCE(pdx->m_PciDeviceLost, 0);
		pdx->m_recovered = 1;
		dev_info(&pdx->pdev->dev, "card responds again\n");
	}
	active_video = ((value&0xFF)>>ch)&0x01;
	interlace = value>>8;
	interlace = ((interlace&0xFF)>>ch)&0x01;
	if(active_video >0)
	{
		if(pdx->m_DeviceHW_Version>0)
		{
			offset = 110 + ch;
			frame_rate = READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)));
			if(frame_rate !=pdx->m_pVCAPStatus[ch][0].dwFrameRate )
			{
				pdx->m_pVCAPStatus[ch][0].dwFrameRate = frame_rate;
			}
			offset = 120 + ch;
			value =  READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)));
			out_res_w = value&0xFFFF;
			out_res_h = (value>>16)&0xFFFF;
			if((out_res_w!=pdx->m_pVCAPStatus[ch][0].dwOutWidth)||(out_res_h!= pdx->m_pVCAPStatus[ch][0].dwOutHeight))
			{
				out_value = pdx->m_pVCAPStatus[ch][0].dwOutHeight;
				out_value =(out_value<<16)|pdx->m_pVCAPStatus[ch][0].dwOutWidth;
				WRITE_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)), out_value);
			}
			offset = 130 + ch;
			out_frame_rate = READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)));
			if(out_frame_rate !=pdx->m_pVCAPStatus[ch][0].dwOutFrameRate )
			{
				WRITE_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)), pdx->m_pVCAPStatus[ch][0].dwOutFrameRate);

			}
			offset = 91 + ch*2;
			value =  READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)));
			br = value&0xFF;
			co= (value>>8)&0xFF;
			hu= (value>>16)&0xFF;
			sa= (value>>24)&0xFF;

			if((co != pdx->m_contrast[ch])||(br != pdx->m_brightness[ch])||(sa != pdx->m_saturation[ch])||(hu != pdx->m_hue[ch]))
			{
				out_value  =  pdx->m_saturation[ch];
				out_value  = (out_value<<8)|pdx->m_hue[ch];
				out_value  = (out_value<<8)|pdx->m_contrast[ch];
				out_value  = (out_value<<8)|pdx->m_brightness[ch];
				WRITE_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)), out_value);
			}
			value =  READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (8*PCIE_BARADDROFSIZE)));
			video_hdcp = ((value&0xFF)>>ch)&0x01;
			pdx->m_pVCAPStatus[ch][0].dwhdcp = video_hdcp;
		}
		else if (ktime_get_ns() - pdx->m_sw_rate_t0[ch] >= NSEC_PER_SEC)  //---------------
		{
			/* half-frames counted over (at least) one second, whatever the poll rate */
			u64 now = ktime_get_ns();
			u64 rate = div64_u64((u64)atomic_xchg(&pdx->m_dwSWFrameRate[ch], 0) * NSEC_PER_SEC,
					     max_t(u64, now - pdx->m_sw_rate_t0[ch], 1));

			pdx->m_sw_rate_t0[ch] = now;
			if(rate>10)
			{
				if(rate>(55*2))
				{
					pdx->m_pVCAPStatus[ch][0].dwFrameRate =60;

				}
				else if(rate>(45*2))
				{
					pdx->m_pVCAPStatus[ch][0].dwFrameRate =50;

				}
				else if(rate>(25*2))
				{
					pdx->m_pVCAPStatus[ch][0].dwFrameRate =30;

				}
				else if(rate>(20*2))
				{
					pdx->m_pVCAPStatus[ch][0].dwFrameRate =25;

				}
				else
				{
					pdx->m_pVCAPStatus[ch][0].dwFrameRate =60;
				}
			}
		}
		offset = 90 + ch*2;
		value =  READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)));
		res_w = value&0xFFFF;
		res_h = (value>>16)&0xFFFF;
		/*
		 * 0 x 0 (or one side 0) shows up while the source switches modes: keep
		 * what we had. A mode the buffers aren't made for - above 1920x1080, or
		 * tiny (a zero DMA half size) - counts as no signal, with the channel's
		 * DMA off until the mode fits: the card might write past the buffer.
		 */
		if ((res_w > 0) && (res_h > 0))
		{
			bool fits = res_w >= HWS_MIN_IN_W && res_w <= MAX_VIDEO_HW_W &&
				    res_h >= HWS_MIN_IN_H && (interlace ? 2 * res_h : res_h) <= MAX_VIDEO_HW_H;

			if (!fits)
			{
				if (!pdx->mode_unsupported[ch])
				{
					hws_park_channel(pdx, ch, true);
					dev_warn_ratelimited(&pdx->pdev->dev,
						 "channel %u: input %dx%d%s not supported (%dx%d to %dx%d), shown as no signal\n",
						 ch, res_w, interlace ? 2 * res_h : res_h, interlace ? "i" : "p",
						 HWS_MIN_IN_W, HWS_MIN_IN_H, MAX_VIDEO_HW_W, MAX_VIDEO_HW_H);
				}
				return 1;
			}
			if((res_w !=pdx->m_pVCAPStatus[ch][0].dwWidth )||(res_h!= pdx->m_pVCAPStatus[ch][0].dwHeight)||(pdx->m_pVCAPStatus[ch][0].dwinterlace!=interlace))
			{
				ChangeVideoSize(pdx,ch,res_w,res_h,interlace);
				hws_reset_rate(pdx, ch);
				hws_follow_input(&pdx->video[ch], res_w, interlace ? 2 * res_h : res_h);
				hws_notify_source_change(&pdx->video[ch]);

			}
			/* programmed for the mode first, then the DMA back on */
			if (pdx->mode_unsupported[ch])
				hws_park_channel(pdx, ch, false);
		}
		else if (pdx->mode_unsupported[ch])
		{
			return 1;	/* still switching away from the unsupported mode */
		}

		no_video = 0;
	}
	else
	{
		no_video = 1;
	}

	return no_video;

}

static void CheckVideFmt (struct hws_pcie_dev *pdx)
{
	int i;


	for(i =0; i<pdx->m_nCurreMaxVideoChl;i++)
	{
	{
		int was = READ_ONCE(pdx->m_curr_No_Video[i]);
		int card = Get_Video_Status(pdx,i);
		/* a stalled channel (hws_watchdog) shows the no-signal picture, not a frozen one */
		int now = card || READ_ONCE(pdx->wd_stalled[i]);

		WRITE_ONCE(pdx->m_card_No_Video[i], card);

		WRITE_ONCE(pdx->m_curr_No_Video[i], now);
		if (was != now)
		{
			hws_rate_signal_change(pdx, i, !now);
			hws_notify_source_change(&pdx->video[i]);	/* signal lost or back */
			/* lost: the silence fill starts now, not after its long re-arm
			 * (it would then have to catch up in a burst) */
			if (now && READ_ONCE(pdx->audio[i].running))
				mod_delayed_work(pdx->wq, &pdx->audio[i].silence_work, 0);
		}
		else if (!now)
		{
			hws_rate_check(pdx, i);
			hws_measure_rate(pdx, i);
		}
		if (was && !now)
		{
			/* signal back: count the frame rate from here, not across the gap */
			atomic_set(&pdx->m_dwSWFrameRate[i], 0);
			pdx->m_sw_rate_t0[i] = ktime_get_ns();
		}
	}
	if (pdx->m_recovered)
	{
		int ch;

		/* re-program the card and re-enable every channel that still streams */
		pdx->m_recovered = 0;
		/* no copy or conversion pass may run while the card and rings are reset */
		for (ch = 0; ch < pdx->m_nCurreMaxVideoChl; ch++)
		{
			kthread_flush_work(&pdx->video[ch].copy_work);
			kthread_flush_work(&pdx->video[ch].videowork);
		}
		mutex_lock(&pdx->pm_lock);
		if (pdx->need_bringup && !pdx->suspended && !READ_ONCE(pdx->removing))
			hws_hw_bringup(pdx);	/* it was gone at resume: never initialized */
		mutex_unlock(&pdx->pm_lock);
		for (ch = 0; ch < pdx->m_nCurreMaxVideoChl; ch++)
		{
			mutex_lock(&pdx->video[ch].engine_lock);
			mutex_lock(&pdx->pm_lock);
			if (pdx->video[ch].engine_users > 0 && !pdx->suspended && !READ_ONCE(pdx->removing))
			{
				kthread_flush_work(&pdx->video[ch].copy_work);
				kthread_flush_work(&pdx->video[ch].videowork);
				StartVideoCapture(pdx, ch);
			}
			mutex_unlock(&pdx->pm_lock);
			mutex_unlock(&pdx->video[ch].engine_lock);
		}
		/* audio streams still running: the re-init switched their capture off */
		for (ch = 0; ch < pdx->m_nCurreMaxVideoChl; ch++)
		{
			struct hws_audio *drv = &pdx->audio[ch];
			unsigned long aflags;

			/* under ring_lock: a trigger STOP can't come in between */
			spin_lock_irqsave(&drv->ring_lock, aflags);
			if (drv->running && !READ_ONCE(pdx->removing) && !READ_ONCE(pdx->m_PciDeviceLost))
			{
				pdx->m_bACapStarted[ch] = 0;	/* not on the card any more */
				StartAudioCapture(pdx, ch);
			}
			spin_unlock_irqrestore(&drv->ring_lock, aflags);
		}
	}
	if((pdx->m_curr_No_Video[i] ==0x1)&&(pdx->m_bVCapStarted[i]==TRUE))
	{
		kthread_queue_work(pdx->conv_worker[i], &pdx->video[i].videowork);
	}

	}

}

/*
 * A card marked lost: touch its registers again only once it is back on the
 * bus (some root complexes fault on reads of a gone device). Back after a
 * reset, its config space is empty (no memory decode, no MSI): restore it and
 * bring the card up again.
 */
static bool hws_card_reachable(struct hws_pcie_dev *pdx)
{
	u16 cmd;

	if (!READ_ONCE(pdx->m_PciDeviceLost))
		return true;
	if (!pci_device_is_present(pdx->pdev))
		return false;
	if (pci_read_config_word(pdx->pdev, PCI_COMMAND, &cmd) == 0 && !(cmd & PCI_COMMAND_MEMORY))
	{
		mutex_lock(&pdx->pm_lock);
		/* never save here: the card's own (empty) state would replace the good one */
		if (!pdx->suspended && !READ_ONCE(pdx->removing) && pdx->pci_state &&
		    pci_load_saved_state(pdx->pdev, pdx->pci_state) == 0)
		{
			/* the core marked it D3cold when it was unreadable: power state first */
			pci_set_power_state(pdx->pdev, PCI_D0);
			pci_restore_state(pdx->pdev);
			pdx->need_bringup = true;
		}
		mutex_unlock(&pdx->pm_lock);
	}
	return true;
}

/*
 * Frames stopped coming although capture runs and the card reports a picture
 * it can take: the interrupt line was switched off by the kernel ("nobody
 * cared", e.g. a misbehaving device sharing it), an interrupt got lost, or the
 * card's DMA engine hung. Nothing else would notice - apps just wait. After
 * 2 s: log it, show the no-signal picture instead of a frozen one, and bring
 * the card up again (the path a card back after a loss takes), at most 3
 * times 10 s apart. Frames coming back end it. Status thread only.
 */
static void hws_watchdog(struct hws_pcie_dev *pdx)
{
	u64 now = ktime_get_ns();
	/* first run, or the thread didn't run for a while (suspend, a stall of the
	 * system): what it saw then says nothing about now - start counting afresh */
	bool fresh = now - pdx->wd_last_ns > NSEC_PER_SEC / 2;
	bool stalled = false;
	int ch;

	pdx->wd_last_ns = now;
	for (ch = 0; ch < pdx->m_nCurreMaxVideoChl; ch++)
	{
		u32 cnt = READ_ONCE(pdx->m_half_cnt[ch][0]);
		/* frames due: capture on, a picture the card can take, nothing paused on purpose */
		bool due = READ_ONCE(pdx->m_bVCapStarted[ch]) && !READ_ONCE(pdx->m_card_No_Video[ch]) &&
			   !READ_ONCE(pdx->mode_unsupported[ch]) && !pdx->m_pVCAPStatus[ch][0].dwhdcp &&
			   !READ_ONCE(pdx->irq_storm_off) && !READ_ONCE(pdx->m_PciDeviceLost) &&
			   !READ_ONCE(pdx->need_bringup);

		if (cnt != pdx->wd_cnt[ch] || !due || fresh)
		{
			if (cnt != pdx->wd_cnt[ch] && pdx->wd_stalled[ch])
				dev_info(&pdx->pdev->dev, "channel %d: frames from the card again\n", ch);
			pdx->wd_cnt[ch] = cnt;
			pdx->wd_ns[ch] = now;
			WRITE_ONCE(pdx->wd_stalled[ch], false);
			continue;
		}
		if (now - pdx->wd_ns[ch] < 2 * NSEC_PER_SEC)
			continue;
		stalled = true;
		if (!pdx->wd_stalled[ch])
		{
			struct irq_data *d = pdx->irq_line >= 0 ? irq_get_irq_data(pdx->irq_line) : NULL;

			WRITE_ONCE(pdx->wd_stalled[ch], true);
			dev_warn(&pdx->pdev->dev, "channel %d: no frames from the card for 2 s although it reports a signal%s\n",
				 ch, d && irqd_irq_disabled(d) ?
				 " - the kernel switched its interrupt off (\"nobody cared\"?); reload the driver" : "");
		}
	}
	/* a card that keeps hanging gets 3 tries, until a minute passes without a stall */
	if (!stalled && pdx->wd_tries && now - pdx->wd_kick_ns > 60 * NSEC_PER_SEC)
		pdx->wd_tries = 0;
	if (stalled && pdx->wd_tries < 3 && now - pdx->wd_kick_ns > 10 * NSEC_PER_SEC)
	{
		pdx->wd_tries++;
		pdx->wd_kick_ns = now;
		dev_warn(&pdx->pdev->dev, "bringing the card up again (attempt %d of 3)\n", pdx->wd_tries);
		mutex_lock(&pdx->pm_lock);
		if (!pdx->suspended && !READ_ONCE(pdx->removing))
		{
			pdx->need_bringup = true;	/* CheckVideFmt: hws_hw_bringup, then the streams restart */
			pdx->m_recovered = 1;
		}
		mutex_unlock(&pdx->pm_lock);
	}
}

static int MainKsThreadHandle(void *arg)
{
	int need_check=0;
	int i=0;
	unsigned int tick = 0;
	struct hws_pcie_dev *pdx = (struct hws_pcie_dev *)(arg);

	/*
	 * Frozen before the card is suspended, so it never polls a powered-down
	 * card. kthread_freezable_should_stop() also leaves the freezer when
	 * kthread_stop() comes while frozen (remove during resume), where a plain
	 * try_to_freeze() would never return.
	 */
	set_freezable();
	while (!kthread_freezable_should_stop(NULL))
	{
		/* shutdown stopped the card: leave it alone */
		if (READ_ONCE(pdx->removing))
		{
			schedule_timeout_interruptible(HZ / 4);
			continue;
		}

		/* card interrupts paused by the storm guard: try again after a second */
		if (smp_load_acquire(&pdx->irq_storm_off) && !READ_ONCE(pdx->removing) &&
		    ktime_get_ns() - atomic64_read(&pdx->irq_storm_off_ns) > NSEC_PER_SEC)
		{
			unsigned long flags;

			mutex_lock(&pdx->pm_lock);
			/* suspended: resume re-enables them; card lost: retry once it's back;
			 * shut down meanwhile: the card stays off */
			if (!pdx->suspended && !READ_ONCE(pdx->m_PciDeviceLost) && !READ_ONCE(pdx->removing))
			{
				/* flag first: the guard must be able to act on the first interrupt */
				WRITE_ONCE(pdx->irq_storm_off, false);
				smp_mb();
				spin_lock_irqsave(&pdx->reg_lock, flags);
				WRITE_REGISTER_ULONG(pdx, INT_EN_REG_BASE, 0x3ffff);
				spin_unlock_irqrestore(&pdx->reg_lock, flags);
			}
			mutex_unlock(&pdx->pm_lock);
		}
		need_check=0;
		for(i=0; i<pdx->m_nMaxChl; i++)
		{
			if(pdx->m_bVCapStarted[i] ==1)
			{
				need_check = 1;
				break;
			}
		}
		tick++;
		/* every tick: a channel not streaming is refreshed (and its stall ends) */
		hws_watchdog(pdx);
		/* also when a card that was gone at resume is back (config-space check
		 * only until then): CheckVideFmt notices it and re-inits it. Idle: once
		 * a second, so the default format matches the input before an app opens
		 * the device (a few register reads) */
		if((need_check==1 && hws_card_reachable(pdx)) ||
		   (READ_ONCE(pdx->need_bringup) && pci_device_is_present(pdx->pdev) && hws_card_reachable(pdx)) ||
		   (tick % 4 == 0 && !READ_ONCE(pdx->need_bringup) &&
		    pci_device_is_present(pdx->pdev) && hws_card_reachable(pdx)))
		{
			CheckVideFmt(pdx);
			if(timing_debug && tick % 4 == 0)
				hws_tstat_report(pdx);	/* per-second summary */
		}
		if (READ_ONCE(dma_probe) && tick % 8 == 0)
			hws_dma_probe(pdx);
		/* without reports nothing else empties the 32-bit window counters:
		 * move them into the 64-bit totals before any can wrap */
		if (!timing_debug && tick % 240 == 0)
			for (i = 0; i < pdx->m_nCurreMaxVideoChl; i++)
			{
				hws_tstat_fold(&pdx->tstats[i], NULL);
				/* a report after timing_debug is switched on starts here */
				pdx->tstats[i].window_start_ns = ktime_get_ns();
				pdx->tstats[i].last_half0_cnt = READ_ONCE(pdx->m_half_cnt[i][0]);
			}
		/* 4 Hz: a mode change is noticed within 250 ms (cheap: a few register
		 * reads per channel). Returns early when kthread_stop() wakes us */
		schedule_timeout_interruptible(HZ / 4);
	}
	return 0;
}
static void StartKSThread(struct hws_pcie_dev *pdx)
{
	pdx->mMain_tsk = kthread_run(MainKsThreadHandle,(void*)pdx,"hws-status");
	if (IS_ERR(pdx->mMain_tsk))
	{
		dev_err(&pdx->pdev->dev, "cannot start status thread\n");
		pdx->mMain_tsk = NULL;
	}

}


#ifndef arch_msi_check_device
static int arch_msi_check_device(struct pci_dev *dev, int nvec, int type)
{
	return 0;
}
#endif

/* type = PCI_CAP_ID_MSI or PCI_CAP_ID_MSIX */
static int msi_msix_capable(struct pci_dev *dev, int type)
{
	struct pci_bus *bus;
	int ret;
	if (!dev || dev->no_msi)
	{
		dev_dbg(&dev->dev, "%s: MSI disabled\n", __func__);
		return 0;
	}

	for (bus = dev->bus; bus; bus = bus->parent)
	{
		if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI)
		{
			dev_dbg(&dev->dev, "%s: bus has no MSI\n", __func__);
			return 0;
		}
	}
	ret = arch_msi_check_device(dev, 1, type);
	if (ret)
	{
		return 0;
	}
	ret = pci_find_capability(dev, type);
	if (!ret)
	{
		dev_dbg(&dev->dev, "%s: pci_find_capability = %d\n", __func__, ret);
		return 0;
	}

	return 1;
}


static int probe_scan_for_msi(struct hws_pcie_dev *lro, struct pci_dev *pdev)
{
	int rc = 0;

	//		for (i = 0; i < req_nvec; i++)
	//		if (rc < 0)

	//else

	if (msi_msix_capable(pdev, PCI_CAP_ID_MSI)) {
		/* enable message signalled interrupts */
		rc = pci_enable_msi(pdev);
		if (rc < 0)
		{
			/* not fatal: fall back to the legacy interrupt line */
			dev_info(&pdev->dev, "couldn't enable MSI (%d), using INTx\n", rc);
			lro->msi_enabled = 0;
			rc = 0;
		}
		else
		{
			lro->msi_enabled = 1;
		}
		lro->msix_enabled = 0;
	} else {
		lro->msi_enabled = 0;
		lro->msix_enabled = 0;
	}

	return rc;
}


static int irq_setup(struct hws_pcie_dev *lro, struct pci_dev *pdev)
{
	int rc = 0;
	u32 irq_flag;
	u8 val;


	//else
	{
		if (!lro->msi_enabled){
			pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &val);
		}
		/* a legacy INTx line may be shared with other devices; an MSI vector is ours */
		irq_flag = lro->msi_enabled ? 0 : IRQF_SHARED;

		/*
		 * IRQF_ONESHOT has to match among all sharers of a line. Without it
		 * by default (most drivers' plain request_irq() don't set it); join a
		 * line already set up with it: IRQF_COND_ONESHOT where the kernel
		 * has it, else a second try. The IRQ thread only delivers audio
		 * (tens of us), so a ONESHOT line is hardly ever held back.
		 */
#ifdef IRQF_COND_ONESHOT
		if (irq_flag & IRQF_SHARED)
			irq_flag |= IRQF_COND_ONESHOT;
#endif
		rc = request_threaded_irq(pdev->irq, irqhandler, hws_irq_thread, irq_flag, pci_name(pdev), lro);
		if (rc == -EBUSY && (irq_flag & IRQF_SHARED))
			rc = request_threaded_irq(pdev->irq, irqhandler, hws_irq_thread,
						  irq_flag | IRQF_ONESHOT, pci_name(pdev), lro);
		if (rc)
		{
		}
		else
		{
			lro->irq_line = (int)pdev->irq;
		}
	}

	return rc;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
static void enable_pcie_relaxed_ordering(struct pci_dev *dev)
{
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,10,0)
	struct pci_dev *root = pcie_find_root_port(dev);

	/* the PCI core clears Relaxed Ordering below root ports where it corrupts
	 * or hangs DMA (PCI_DEV_FLAGS_NO_RELAXED_ORDERING); don't turn it back on */
	if (root && (root->dev_flags & PCI_DEV_FLAGS_NO_RELAXED_ORDERING))
		return;
	#endif
	pcie_capability_set_word(dev, PCI_EXP_DEVCTL, PCI_EXP_DEVCTL_RELAX_EN);
}
#else
static void enable_pcie_relaxed_ordering(struct pci_dev *dev)
{
	u16 v;
	int pos;

	pos = pci_pcie_cap(dev);
	if (pos > 0) {
		pci_read_config_word(dev, pos + PCI_EXP_DEVCTL, &v);
		v |= PCI_EXP_DEVCTL_RELAX_EN;
		pci_write_config_word(dev, pos + PCI_EXP_DEVCTL, v);
	}
}
#endif
static void InitVideoSys(struct hws_pcie_dev *pdx,int set)
{
	// init decoder
	int i,j;
	DWORD m_Valude;
	if(pdx->m_bStartRun&&(set==0)) return;
	WRITE_REGISTER_ULONG(pdx,( CVBS_IN_BASE + (0 * PCIE_BARADDROFSIZE)), 0X00);
	SetDMAAddress(pdx);
	if(set ==0)
	{
		for(i=0; i<pdx->m_nMaxChl;i++)
		{
			for (j = 0; j<MAX_VIDEO_QUEUE; j++)
			{
				pdx->m_pVCAPStatus[i][j].byLock = MEM_UNLOCK;
				pdx->m_pVCAPStatus[i][j].byPath = 2;
				pdx->m_pVCAPStatus[i][j].byField = 0;
				pdx->m_pVCAPStatus[i][j].dwinterlace =0;
			}
			pdx->m_nAudioBufferIndex[i] =0;
			EnableVideoCapture(pdx,i,0);
			EnableAudioCapture(pdx,i,0);

		}
	}

	hws_enable_card_irqs(pdx);
	//start Run
	WRITE_REGISTER_ULONG(pdx,CVBS_IN_BASE, 0x80000000);
	m_Valude= 0x00FFFFFF;
	m_Valude |= 0x80000000;
	WRITE_REGISTER_ULONG(pdx,CVBS_IN_BASE, m_Valude);
	WRITE_REGISTER_ULONG(pdx,(CVBS_IN_BASE + (0 * PCIE_BARADDROFSIZE)), 0X13);
	pdx->m_bStartRun = 1;

}

static void SetHardWareInfo(struct hws_pcie_dev *pdx)
{
	switch (pdx->dwDeviceID)
	{
		case 0x9534:
		case 0x6524:
		case 0x8524:
		{
			pdx->m_nCurreMaxVideoChl = 4;
			pdx->m_nCurreMaxLineInChl =1;
			pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
			break;
		}
		case 0x8532:
		{
			pdx->m_nCurreMaxVideoChl = 2;
			pdx->m_nCurreMaxLineInChl =1;
			pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
			break;
		}
		case 0x8512:
		case 0x6502:	/* 2-channel card, vendor 20260519 */
		{
			pdx->m_nCurreMaxVideoChl = 2;
			pdx->m_nCurreMaxLineInChl =0;
			pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
			break;
		}
		case 0x8501:
		{
			pdx->m_nCurreMaxVideoChl = 1;
			pdx->m_nCurreMaxLineInChl =0;
			pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
			break;
		}
		default:
		{
			pdx->m_nCurreMaxVideoChl = 4;
			pdx->m_nCurreMaxLineInChl =0;
			pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
			break;
		}
	}
	if(pdx->m_Device_Version>121)
	{
		if((pdx->dwDeviceID==0x8501)&&(pdx->m_Device_Version==122))
		{
			pdx->m_DeviceHW_Version =0;
		}
		else
		{
			pdx->m_DeviceHW_Version =1;
			//--- set DMA_MAX_SIZE
			WRITE_REGISTER_ULONG(pdx,(CVBS_IN_BASE + (9 * PCIE_BARADDROFSIZE)), (MAX_VIDEO_SCLAER_SIZE/16));
		}
	}
	else
	{
		pdx->m_DeviceHW_Version =0;
	}


}
static int ReadChipId(struct hws_pcie_dev *pdx)
{
	int ret=0;
	int i;
	//------read Dvice Version
	ULONG m_dev_ver;
	ULONG m_tmpVersion;
	ULONG m_tmpHWKey;
	m_dev_ver= READ_REGISTER_ULONG(pdx,CVBS_IN_BASE+(88*PCIE_BARADDROFSIZE));
	if (m_dev_ver == 0xFFFFFFFF)
	{
		dev_err(&pdx->pdev->dev, "card does not respond (reads all ones)\n");
		return -ENODEV;
	}

	m_tmpVersion = m_dev_ver>>8;
	pdx->m_Device_Version =  (m_tmpVersion&0xFF);
	m_tmpVersion = m_dev_ver>>16;
	pdx->m_Device_SubVersion = (m_tmpVersion&0xFF);
	pdx->m_Device_SupportYV12 = ((m_dev_ver>>28)&0x0F);
	if (pdx->m_Device_SupportYV12 > 2)
	{
		dev_warn(&pdx->pdev->dev, "unknown pixel layout %u, treating as YUYV\n", pdx->m_Device_SupportYV12);
		pdx->m_Device_SupportYV12 = 0;
	}
	m_tmpHWKey =  m_dev_ver>>24;
	m_tmpHWKey = m_tmpHWKey&0x0F;
	pdx->m_Device_PortID = m_tmpHWKey&0x03;
	pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
	pdx->m_nMaxChl  = 4;
	pdx->m_bBufferAllocate = FALSE;
	pdx->mMain_tsk = NULL;
	pdx->m_dwAudioPTKSize = MAX_DMA_AUDIO_PK_SIZE; //128*16*4;
	pdx->m_bStartRun = 0;
	pdx->m_PciDeviceLost =0;

	for( i=0; i<MAX_VID_CHANNELS; i++)
	{
		SetVideoFormteSize(pdx,i,1920,1080);
	}
	WRITE_REGISTER_ULONG(pdx,CVBS_IN_BASE,0x0);
	WRITE_REGISTER_ULONG(pdx,CVBS_IN_BASE,0x10);
	SetHardWareInfo(pdx);
	dev_info(&pdx->pdev->dev, "hardware version %d.%d, port id %d\n", pdx->m_Device_Version, pdx->m_Device_SubVersion, pdx->m_Device_PortID);
	return ret;

}

/*
 * FIFO 2 for the copy threads, FIFO 1 for conversion: a half-frame copy has a
 * deadline (the card rewrites that half), a conversion only latency, so a
 * copy must not wait for a conversion run on the same CPU. Both stay far
 * below the IRQ threads (FIFO 50) and ahead of normal tasks.
 */
static struct kthread_worker *hws_create_worker(const char *fmt, int ch, int prio)
{
	struct kthread_worker *w = kthread_create_worker(0, fmt, ch);

	if (!IS_ERR(w)) {
		struct sched_attr attr = {
			.size		= sizeof(attr),
			.sched_policy	= SCHED_FIFO,
			.sched_priority	= prio,
		};

		if (sched_setattr_nocheck(w->task, &attr))
			sched_set_fifo_low(w->task);
		wake_up_process(w->task);
	}
	return w;
}

static int hws_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct hws_pcie_dev *gdev;
	int err;
	int j, i;

	gdev = alloc_dev_instance(pdev);
	if (!gdev)
		return -ENOMEM;
	kref_init(&gdev->kref);
	spin_lock_init(&gdev->reg_lock);
	mutex_init(&gdev->pm_lock);
	mutex_init(&gdev->irq_lock);

	gdev->dwDeviceID = gdev->pdev->device;
	gdev->dwVendorID = gdev->pdev->vendor;
	dev_info(&pdev->dev, "AVMatrix capture card %04x:%04x\n", gdev->dwVendorID, gdev->dwDeviceID);
	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "%s: pci_enable_device failed: %d\n",
				__func__, err);
		goto err_free;
	}

	/* the card is programmed with 32-bit-window addresses: say so explicitly */
	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(&pdev->dev, "no usable 32-bit DMA\n");
		goto err_disable;
	}
	if (pci_request_regions(pdev, KBUILD_MODNAME) == 0)
		gdev->regions_requested = true;
	else
		dev_warn(&pdev->dev, "BAR regions busy, continuing\n");

	enable_pcie_relaxed_ordering(pdev);
	probe_scan_for_msi(gdev, pdev);	/* falls back to INTx, never fails */
	/* known root complex's max read request sizes */
	#ifdef CONFIG_ARCH_TI816X
	pcie_set_readrq(pdev, 128);
	#endif

	/* another board variant or a fallback FPGA image: don't map what we can't use */
	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM) ||
	    pci_resource_len(pdev, 0) < HWS_BAR0_MIN) {
		dev_err(&pdev->dev, "BAR0 is not a memory region of at least %#x bytes\n", HWS_BAR0_MIN);
		err = -ENODEV;
		goto err_msi;
	}
	gdev->info.mem[0].internal_addr = ioremap(pci_resource_start(pdev, 0),
											  pci_resource_len(pdev, 0));
	gdev->map_bar0_addr = gdev->info.mem[0].internal_addr;
	if (!gdev->info.mem[0].internal_addr) {
		err = -ENOMEM;
		goto err_msi;
	}
	gdev->info.mem[0].size = pci_resource_len(pdev, 0);
	gdev->info.mem[0].memtype = UIO_MEM_PHYS;

	err = ReadChipId(gdev);
	if (err)
		goto err_unmap;
	/* only now that the card is reset may it master the bus */
	pci_set_master(pdev);
	for (i = 0; i<MAX_VID_CHANNELS; i++)
	{
		gdev->m_nRDVideoIndex[i] =0;
		gdev->m_bVCapIntDone[i] =0;
		gdev->m_nVideoBusy[i] = 0;
		gdev->m_bChangeVideoSize[i] =0;
		gdev->m_nVideoBufferIndex[i] = 0;
		gdev->m_nVideoHalfDone[i] =0;
		gdev->m_pVideoEvent[i] = 0;
		SetVideoFormteSize(gdev,i,1920,1080);
		gdev->m_bVCapStarted[i] = 0;
		gdev->m_bVideoStop[i]=0;
		gdev->video_data[i] =0;
		gdev->m_contrast[i] =0x80 ;
		gdev->m_brightness[i] = 0x80;
		gdev->m_saturation[i] = 0x80;
		gdev->m_hue[i] = 0x80;
		atomic_set(&gdev->m_dwSWFrameRate[i], 0);
		gdev->m_pbyVideoBuffer[i] = NULL;
		gdev->m_VideoInfo[i].dwisRuning= 0;
		gdev->m_VideoInfo[i].m_nVideoIndex= 0;
		gdev->m_VideoInfo[i].m_pVideoScalerBuf = NULL;
		gdev->m_VideoInfo[i].m_pScaler = NULL;
		gdev->m_VideoInfo[i].m_pVideoYUV2Buf = NULL;
		gdev->m_VideoInfo[i].m_pRotateVideoBuf = NULL;
		for (j = 0; j<MAX_VIDEO_QUEUE; j++)
		{
			gdev->m_pVCAPStatus[i][j].byLock = MEM_UNLOCK;
			gdev->m_pVCAPStatus[i][j].byField = 0;
			gdev->m_pVCAPStatus[i][j].byPath = 2;
			gdev->m_pVCAPStatus[i][j].dwWidth = 1920 ;
			gdev->m_pVCAPStatus[i][j].dwHeight = 1080;
			gdev->m_pVCAPStatus[i][j].dwinterlace =0;
			gdev->m_pVCAPStatus[i][j].dwFrameRate = 60;
			gdev->m_pVCAPStatus[i][j].dwOutWidth = 1920 ;
			gdev->m_pVCAPStatus[i][j].dwOutHeight = 1080;
			gdev->m_VideoInfo[i].m_pVideoBufData[j] = NULL;
			gdev->m_VideoInfo[i].m_pVideoBufData1[j] = NULL;
			gdev->m_VideoInfo[i].m_pVideoBufData2[j] = NULL;
			gdev->m_VideoInfo[i].m_pVideoBufData3[j] = NULL;
			gdev->m_VideoInfo[i].pStatusInfo[j].byLock= MEM_UNLOCK;
		}
		//--------audio
		gdev->m_pAudioEvent[i] = 0;
		gdev->m_bACapStarted[i]=0;
		gdev->m_bAudioRun[i] = 0;
		gdev->m_bAudioStop[i] = 0;
		gdev->m_nAudioBusy[i] = 0;
		gdev->m_nRDAudioIndex[i] =0;
		spin_lock_init(&gdev->videoslock[i]);
		spin_lock_init(&gdev->audiolock[i]);
		gdev->m_AudioInfo[i].dwisRuning =0;
		gdev->m_AudioInfo[i].m_nAudioIndex =0;
		gdev->audio[i].resampled_buf =NULL;
		for(j=0; j<MAX_AUDIO_QUEUE;j++)
		{
			gdev->m_AudioInfo[i].m_pAudioBufData[j] =NULL;
			gdev->m_AudioInfo[i].pStatusInfo[j].byLock = MEM_UNLOCK;
		}
		/* everything the IRQ, its thread and the workers can reach is set up before the IRQ */
		gdev->video[i].index = i;
		gdev->video[i].dev = gdev;
		mutex_init(&gdev->video[i].video_lock);
		mutex_init(&gdev->video[i].engine_lock);
		spin_lock_init(&gdev->video[i].consumers_lock);
		INIT_LIST_HEAD(&gdev->video[i].consumers);
		gdev->video[i].engine_users = 0;
		kthread_init_work(&gdev->video[i].videowork, video_data_process);
		kthread_init_delayed_work(&gdev->video[i].nosig_work, hws_nosig_work);
		rt_mutex_init(&gdev->direct_lock[i]);
		INIT_LIST_HEAD(&gdev->direct_bufs[i]);
		kthread_init_work(&gdev->video[i].fanout_work, hws_fanout_work);
		kthread_init_work(&gdev->video[i].copy_work, hws_copy_work);
		spin_lock_init(&gdev->video[i].fanout_lock);
		INIT_LIST_HEAD(&gdev->video[i].fanout_q);
		INIT_LIST_HEAD(&gdev->video[i].fanout_free);
		gdev->video[i].fanout_inflight = 0;
		for (j = 0; j < HWS_FANOUT_POOL; j++) {
			INIT_LIST_HEAD(&gdev->video[i].fanout_pool[j].bufs);
			list_add_tail(&gdev->video[i].fanout_pool[j].node, &gdev->video[i].fanout_free);
		}
		gdev->audio[i].index = i;
		gdev->audio[i].dev = gdev;
		spin_lock_init(&gdev->audio[i].ring_lock);
		spin_lock_init(&gdev->tstats[i].fold_lock);
		INIT_WORK(&gdev->audio[i].audiowork,audio_data_process);
		INIT_DELAYED_WORK(&gdev->audio[i].silence_work, hws_audio_silence_work);
	}
	/*
	 * High priority so frame delivery isn't delayed behind ordinary kernel
	 * work under load; unbound so the channels don't wait for each other.
	 * Each work item still never runs concurrently with itself.
	 */
	gdev->wq = alloc_workqueue("hws", WQ_HIGHPRI | WQ_UNBOUND, 0);
	if (!gdev->wq) {
		err = -ENOMEM;
		goto err_mem;	/* bus mastering is on: clear it (no pool yet: free is a no-op) */
	}
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		/* low RT priorities: ahead of normal tasks (an encoder can't delay
		 * frames), behind every IRQ thread. Not started by
		 * kthread_create_worker() since 6.14, hence wake_up_process(). */
		gdev->copy_worker[i] = hws_create_worker("hws-copy%d", i, 2);
		gdev->conv_worker[i] = hws_create_worker("hws-conv%d", i, 1);
		if (IS_ERR(gdev->copy_worker[i]) || IS_ERR(gdev->conv_worker[i])) {
			err = IS_ERR(gdev->copy_worker[i]) ? PTR_ERR(gdev->copy_worker[i]) :
							     PTR_ERR(gdev->conv_worker[i]);
			if (IS_ERR(gdev->copy_worker[i]))
				gdev->copy_worker[i] = NULL;
			if (IS_ERR(gdev->conv_worker[i]))
				gdev->conv_worker[i] = NULL;
			goto err_mem;	/* hws_dev_release() destroys the ones made */
		}
	}

	err = DmaMemAllocPool(gdev);
	if (err)
		goto err_mem;	/* frees what was allocated, clears bus mastering */

	err = irq_setup(gdev, pdev);
	if (err) {
		dev_err(&pdev->dev, "cannot get IRQ %d: %d\n", pdev->irq, err);
		goto err_mem;
	}
	/* restored if the card comes back after a reset (link down, power loss);
	 * our own copy: the PCI core's saved state is used up by its own restores */
	pci_save_state(pdev);
	gdev->pci_state = pci_store_saved_state(pdev);
	InitVideoSys(gdev,0);
	StartKSThread(gdev);

	hws_adapters_init(gdev);
	err = hws_video_register(gdev);
	if (err)
		goto err_video;	/* earlier channels may already be open and streaming */
	err = hws_audio_register(gdev);
	if (err)
		goto err_video;
	return 0;

err_video:
	/* the nodes were visible: someone may already be streaming */
	hws_block_streams(gdev);
	hws_quiesce(gdev);
err_mem:
	pci_clear_master(pdev);
	DmaMemFreePool(gdev);
err_unmap:
	iounmap(gdev->info.mem[0].internal_addr);
	gdev->map_bar0_addr = NULL;
err_msi:
	if (gdev->msi_enabled)
	{
		pci_disable_msi(pdev);
		gdev->msi_enabled = 0;
	}
	if (gdev->regions_requested)
		pci_release_regions(pdev);
err_disable:
	pci_disable_device(pdev);
err_free:
	pci_set_drvdata(pdev, NULL);
	hws_video_release_v4l2(gdev);
	kref_put(&gdev->kref, hws_dev_release);
	return err;
}

MODULE_DEVICE_TABLE(pci, hws_pci_table);

/*
 * System suspend/resume. The card loses its DMA addresses, enables and
 * interrupt mask while powered down, so suspend stops it cleanly and resume
 * re-runs the hardware init and restarts every channel that still has
 * streamers. Apps keep their streams; ALSA streams are suspended by the core
 * and restarted by the apps.
 */
/*
 * Same bring-up as probe (ReadChipId + InitVideoSys), keeping our state: after
 * resume, or when a card that was gone at resume time is back. pm_lock held.
 */
static void hws_hw_bringup(struct hws_pcie_dev *pdx)
{
	DWORD il[MAX_VID_CHANNELS];
	unsigned long flags;
	int i, j;

	pci_set_master(pdx->pdev);
	pdx->m_PciDeviceLost = 0;
	pdx->need_bringup = false;
	/* InitVideoSys re-enables the card's interrupts */
	pdx->irq_stuck_passes = 0;
	WRITE_ONCE(pdx->irq_storm_off, false);
	WRITE_REGISTER_ULONG(pdx, CVBS_IN_BASE, 0x0);
	WRITE_REGISTER_ULONG(pdx, CVBS_IN_BASE, 0x10);
	SetHardWareInfo(pdx);
	/* InitVideoSys() resets the per-channel interlace flag; keep the
	 * detected one (the status thread would only restore it later) */
	for (i = 0; i < MAX_VID_CHANNELS; i++)
		il[i] = pdx->m_pVCAPStatus[i][0].dwinterlace;
	pdx->m_bStartRun = 0;
	InitVideoSys(pdx, 0);
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		spin_lock_irqsave(&pdx->videoslock[i], flags);
		for (j = 0; j < MAX_VIDEO_QUEUE; j++)
			pdx->m_pVCAPStatus[i][j].dwinterlace = il[i];
		pdx->m_fmt_gen[i]++;	/* nothing captured before pairs up */
		spin_unlock_irqrestore(&pdx->videoslock[i], flags);
	}
}

static int __maybe_unused hws_suspend(struct device *dev)
{
	struct hws_pcie_dev *pdx = dev_get_drvdata(dev);
	int i;

	if (!pdx || !pdx->map_bar0_addr || pdx->removing)
		return 0;

	mutex_lock(&pdx->pm_lock);
	pdx->suspended = true;
	hws_hw_stop(pdx);
	if (pdx->irq_line != -1)
		synchronize_irq(pdx->irq_line);	/* hard handler and IRQ thread */
	for (i = 0; i < MAX_VID_CHANNELS; i++)
		kthread_flush_work(&pdx->video[i].copy_work);	/* and the copies it queued */
	for (i = 0; i < MAX_VID_CHANNELS; i++) {
		hws_direct_reclaim(pdx, i, NULL);
		kthread_flush_work(&pdx->video[i].fanout_work);
		kthread_cancel_delayed_work_sync(&pdx->video[i].nosig_work);
		kthread_flush_work(&pdx->video[i].videowork);
		kthread_cancel_delayed_work_sync(&pdx->video[i].nosig_work);
		flush_work(&pdx->audio[i].audiowork);
		cancel_delayed_work_sync(&pdx->audio[i].silence_work);
	}
	mutex_unlock(&pdx->pm_lock);
	return 0;
}

static int __maybe_unused hws_resume(struct device *dev)
{
	struct hws_pcie_dev *pdx = dev_get_drvdata(dev);
	int i;

	if (!pdx || !pdx->map_bar0_addr || pdx->removing)
		return 0;

	mutex_lock(&pdx->pm_lock);
	/* the card may be gone after the sleep (unplugged dock, dead link) */
	if (!pci_device_is_present(pdx->pdev) ||
	    READ_REGISTER_ULONG(pdx, CVBS_IN_BASE) == 0xFFFFFFFF)
	{
		WRITE_ONCE(pdx->m_PciDeviceLost, 1);
		pdx->need_bringup = true;	/* the status thread re-inits it if it comes back */
		/* streams that ran before the sleep get "no signal" frames instead
		 * of nothing (only the flags are set: the registers are skipped) */
		for (i = 0; i < pdx->m_nCurreMaxVideoChl; i++)
			if (pdx->video[i].engine_users > 0)
				StartVideoCapture(pdx, i);
		pdx->suspended = false;
		mutex_unlock(&pdx->pm_lock);
		dev_warn(dev, "card not responding after resume\n");
		return 0;
	}
	hws_hw_bringup(pdx);
	if (pci_save_state(pdx->pdev) == 0)	/* a working card: refresh our copy */
	{
		struct pci_saved_state *st = pci_store_saved_state(pdx->pdev);

		if (st)
		{
			kfree(pdx->pci_state);
			pdx->pci_state = st;
		}
	}
	for (i = 0; i < pdx->m_nCurreMaxVideoChl; i++)
		if (pdx->video[i].engine_users > 0)
			StartVideoCapture(pdx, i);
	pdx->suspended = false;
	mutex_unlock(&pdx->pm_lock);
	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0))
static DEFINE_SIMPLE_DEV_PM_OPS(hws_pm_ops, hws_suspend, hws_resume);
#define HWS_PM_OPS	pm_sleep_ptr(&hws_pm_ops)
#else
static SIMPLE_DEV_PM_OPS(hws_pm_ops, hws_suspend, hws_resume);
#define HWS_PM_OPS	(&hws_pm_ops)
#endif

/*
 * Reboot/kexec: stop the card so it neither DMAs into memory the next kernel
 * owns nor keeps a (shared) interrupt line asserted. Nothing is freed.
 */
static void hws_shutdown(struct pci_dev *pdev)
{
	struct hws_pcie_dev *dev = pci_get_drvdata(pdev);
	int i;

	if (!dev || !dev->map_bar0_addr)
		return;
	WRITE_ONCE(dev->removing, true);	/* no new streams, no re-arming */
	/*
	 * The status thread isn't stopped: when hibernation powers off it is
	 * frozen and would never exit. It sees "removing" and stays away.
	 * A stream start that is already past its "removing" check finishes
	 * before we stop the card (engine_lock), so it can't switch it back on.
	 */
	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++) {
		mutex_lock(&dev->video[i].engine_lock);
		mutex_unlock(&dev->video[i].engine_lock);
	}
	mutex_lock(&dev->pm_lock);
	if (!pci_device_is_present(pdev))
		WRITE_ONCE(dev->m_PciDeviceLost, 1);
	hws_hw_stop(dev);
	mutex_unlock(&dev->pm_lock);
	pci_clear_master(pdev);
}

static struct pci_driver hws_pci_driver = {
	.name        = KBUILD_MODNAME,
	.id_table    = hws_pci_table,
	.probe       = hws_probe,
	.remove      = hws_remove,
	.shutdown    = hws_shutdown,
	.driver.pm   = HWS_PM_OPS,
};

static __init int pcie_hws_init(void)
{

	return pci_register_driver(&hws_pci_driver);
}

static __exit void pcie_hws_exit(void)
{
	pci_unregister_driver(&hws_pci_driver);
}

module_init(pcie_hws_init);
module_exit(pcie_hws_exit);

MODULE_DESCRIPTION("HWS driver");
MODULE_AUTHOR("Sales <sales@avmatrix.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
