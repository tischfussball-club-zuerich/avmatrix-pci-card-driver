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
//static void StartDma(int index);
//static void StopDma(int index);
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
void video_data_process(struct kthread_work *p_work);
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
MODULE_PARM_DESC(nt_copy, "Direct path: write frames with non-temporal stores, bypassing the CPU cache (default 0)");

/* direct path into an app buffer */
static inline void hws_dcopy(void *dst, const void *src, size_t n, bool nt)
{
	if (nt)
		memcpy_flushcache(dst, src, n);
	else
		memcpy(dst, src, n);
}

static int timing_debug;
module_param(timing_debug, int, 0644);
MODULE_PARM_DESC(timing_debug, "Frame timing diagnostics: 0=off, 1=per-second summary, 2=also warn on late frames");

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

	atomic_inc(&ts->frames_in);
	hws_tstat_max(&ts->copy_lat_max, ktime_get_ns() - irq_ns);

	if (ts->last_frame_ns) {
		interval = irq_ns - ts->last_frame_ns;
		if (interval < ts->interval_min)
			ts->interval_min = interval;
		hws_tstat_max(&ts->interval_max, interval);
		if (timing_debug >= 2 && interval > hws_frame_period_ns(pdx, ch) * 3 / 2)
			pr_warn_ratelimited("HwsCapture ch%d: late frame from hardware, interval %llu us\n",
					    ch, div_u64(interval, NSEC_PER_USEC));
	}
	ts->last_frame_ns = irq_ns;
}

static void hws_tstat_report(struct hws_pcie_dev *pdx)
{
	u64 now = ktime_get_ns();
	int ch;

	for (ch = 0; ch < pdx->m_nCurreMaxVideoChl; ch++) {
		struct hws_timing_stats *ts = &pdx->tstats[ch];
		u64 elapsed_ms = div_u64(now - ts->window_start_ns, NSEC_PER_MSEC);
		unsigned int in = atomic_xchg(&ts->frames_in, 0);
		unsigned int out = atomic_xchg(&ts->frames_out, 0);

		if (pdx->m_bVCapStarted[ch] && elapsed_ms > 0)
			pr_info("HwsCapture ch%d: in %llu fps (hw %u) out %llu fps | irq_busy %d stale_half %d dma_overrun %d direct %d overwrite %d skip %d wq_coalesced %d no_app_buf %d empty %d | interval %llu-%llu us, copy_lat %llu us, wq_lat %llu us, lock_hold %llu us, backlog %d\n",
				ch, div64_u64((u64)in * 1000, elapsed_ms),
				pdx->m_pVCAPStatus[ch][0].dwFrameRate,
				div64_u64((u64)out * 1000, elapsed_ms),
				atomic_read(&ts->irq_busy_skip), atomic_read(&ts->stale_half),
				atomic_read(&ts->dma_overrun), atomic_read(&ts->direct),
				atomic_read(&ts->ring_overwrite), atomic_read(&ts->ring_skip),
				atomic_read(&ts->wq_coalesced),
				atomic_read(&ts->no_app_buf), atomic_read(&ts->worker_empty),
				ts->interval_min == U64_MAX ? 0 : div_u64(ts->interval_min, NSEC_PER_USEC),
				div_u64(ts->interval_max, NSEC_PER_USEC),
				div_u64(ts->copy_lat_max, NSEC_PER_USEC),
				div_u64(ts->wq_lat_max, NSEC_PER_USEC),
				div_u64(ts->lock_hold_max, NSEC_PER_USEC),
				ts->backlog_max);

		atomic_set(&ts->irq_busy_skip, 0);
		atomic_set(&ts->stale_half, 0);
		atomic_set(&ts->dma_overrun, 0);
		atomic_set(&ts->direct, 0);
		atomic_set(&ts->ring_overwrite, 0);
		atomic_set(&ts->ring_skip, 0);
		atomic_set(&ts->wq_coalesced, 0);
		atomic_set(&ts->no_app_buf, 0);
		atomic_set(&ts->worker_empty, 0);
		ts->interval_min = U64_MAX;
		ts->interval_max = 0;
		ts->copy_lat_max = 0;
		ts->wq_lat_max = 0;
		ts->lock_hold_max = 0;
		ts->backlog_max = 0;
		ts->window_start_ns = now;
	}
}

//------------------------

static	void VideoScaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
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
static	void All_VideoScaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h);
//---------------------------
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
//------------------
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
	#if 0
	[V4L2_MODEL_VIDEOFORMAT_1920X1200P60]	= V4L2_MODEL_TIMING(1920,1200,60,0),
	[V4L2_MODEL_VIDEOFORMAT_2560X1080P60]	= V4L2_MODEL_TIMING(2560,1080,60,0),
	[V4L2_MODEL_VIDEOFORMAT_2560X1440P60]	= V4L2_MODEL_TIMING(2560,1440,60,0),
	[V4L2_MODEL_VIDEOFORMAT_3840X2160P60]	= V4L2_MODEL_TIMING(3840,2160,60,0),
	[V4L2_MODEL_VIDEOFORMAT_4096X2160P60]	= V4L2_MODEL_TIMING(4096,2160,60,0),
	#endif
};
static const framegrabber_pixfmt_t support_pixfmts[] = {

	[FRAMEGRABBER_PIXFMT_YUYV]={ //YUYV index=0
		.name     = "4:2:2, packed, YUYV",
		.fourcc   = V4L2_PIX_FMT_YUYV,
		.depth    = 16,
		.is_yuv   = true,
		.pixfmt_out = YUYV,
	},
	#if 0
	[FRAMEGRABBER_PIXFMT_UYVY]={ //UYVY
	.name     = "4:2:2, packed, UYVY",
	.fourcc   = V4L2_PIX_FMT_UYVY,
	.depth    = 16,
	.is_yuv   = true,
	.pixfmt_out = UYVY,
},
[FRAMEGRABBER_PIXFMT_YVYU]={ //YVYU
.name     = "4:2:2, packed, YVYU",
.fourcc   = V4L2_PIX_FMT_YVYU,
.depth    = 16,
.is_yuv   = true,
.pixfmt_out = YVYU,
},

[FRAMEGRABBER_PIXFMT_VYUY]={ //VYUY
.name     = "4:2:2, packed, VYUY",
.fourcc   = V4L2_PIX_FMT_VYUY,
.depth    = 16,
.is_yuv   = true,
.pixfmt_out = VYUY,
},

[FRAMEGRABBER_PIXFMT_RGB565]={ //RGBP
.name     = "RGB565 (LE)",
.fourcc   = V4L2_PIX_FMT_RGB565, /* gggbbbbb rrrrrggg */
.depth    = 16,
.is_yuv   = false,
.pixfmt_out = RGBP,
},
[FRAMEGRABBER_PIXFMT_RGB565X]={ //RGBR
.name     = "RGB565 (BE)",
.fourcc   = V4L2_PIX_FMT_RGB565X, /* rrrrrggg gggbbbbb */
.depth    = 16,
.is_yuv   = false,
.pixfmt_out = RGBR,
},
[FRAMEGRABBER_PIXFMT_RGB555]={ //RGBO
.name     = "RGB555 (LE)",
.fourcc   = V4L2_PIX_FMT_RGB555, /* gggbbbbb arrrrrgg */
.depth    = 16,
.is_yuv   = false,
.pixfmt_out = RGBO,
},
[FRAMEGRABBER_PIXFMT_RGB555X]={ //RGBQ
.name     = "RGB555 (BE)",
.fourcc   = V4L2_PIX_FMT_RGB555X, /* arrrrrgg gggbbbbb */
.depth    = 16,
.is_yuv   = false,
.pixfmt_out = RGBQ,
},
[FRAMEGRABBER_PIXFMT_RGB24]={ //RGB3 index=8
.name     = "RGB24 (LE)",
.fourcc   = V4L2_PIX_FMT_RGB24, /* rgb */
.depth    = 24,
.is_yuv   = false,
.pixfmt_out = RGB3,
},

[FRAMEGRABBER_PIXFMT_BGR24]={ //BGR3
.name     = "RGB24 (BE)",
.fourcc   = V4L2_PIX_FMT_BGR24, /* bgr */
.depth    = 24,
.is_yuv   = false,
.pixfmt_out = BGR3,
},
[FRAMEGRABBER_PIXFMT_RGB32]={ //RGB4
.name     = "RGB32 (LE)",
.fourcc   = V4L2_PIX_FMT_RGB32, /* argb */
.depth    = 32,
.is_yuv   = false,
.pixfmt_out = RGB4,
},
[FRAMEGRABBER_PIXFMT_BGR32]={ //BGR4
.name     = "RGB32 (BE)",
.fourcc   = V4L2_PIX_FMT_BGR32, /* bgra */
.depth    = 32,
.is_yuv   = false,
.pixfmt_out = BGR4,
},
#endif
};


//-------------------------------------------

static int v4l2_get_suport_VideoFormatIndex(struct v4l2_format *fmt)
{
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	int index;
	int videoIndex=-1;
	for(index=0; index<V4L2_MODEL_VIDEOFORMAT_NUM;index++)
	{
		if((pix->width==support_videofmt[index].frame_size.width)&&(pix->height==support_videofmt[index].frame_size.height))
		{
			videoIndex = index;
			break;
		}
	}
	return videoIndex;
}
v4l2_model_timing_t *v4l2_model_get_support_videoformat(int index)
{
	if(index <0 ||index >=V4L2_MODEL_VIDEOFORMAT_NUM)
		return NULL;

	return (v4l2_model_timing_t *)&support_videofmt[index];
}


framegrabber_pixfmt_t *v4l2_model_get_support_pixformat(int index)
{
	if(index <0 ||index >=ARRAY_SIZE(support_pixfmts))
		return NULL;

	return (framegrabber_pixfmt_t *)&support_pixfmts[index];
}
const framegrabber_pixfmt_t *framegrabber_g_support_pixelfmt_by_fourcc(u32 fourcc)
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

static int hws_vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct hws_video *videodev = video_drvdata(file);
	struct hws_pcie_dev *dev = videodev->dev;
	int vi_index;
	vi_index = videodev->index+1+dev->m_Device_PortID*dev->m_nCurreMaxVideoChl;
	//printk( "%s\n", __func__);
	strcpy(cap->driver, KBUILD_MODNAME);
	sprintf(cap->card, "%s %d",HWS_VIDEO_NAME,vi_index);
	/* unique per card, so apps and v4l2-compliance can tell boards apart */
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", pci_name(dev->pdev));
	cap->device_caps =	V4L2_CAP_VIDEO_CAPTURE |V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	//printk( "%s(IN END  )\n", __func__);
	return 0;
}
static int hws_vidioc_enum_fmt_vid_cap(struct file *file, void *priv_fh,struct v4l2_fmtdesc *f)
{
	struct hws_video *videodev = video_drvdata(file);
	int index = f->index;
	//printk( "%s(%d)\n", __func__,videodev->index);
	//printk( "%s(f->index = %d)\n", __func__,f->index);

	if(f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
	{
		pr_debug("%s: bad type\n",__func__);
		return -EINVAL;
	}
	if(videodev)
	{
		const framegrabber_pixfmt_t *pixfmt;
		if(f->index <0)
		{
			return -EINVAL;
		}
		if(f->index >= FRAMEGRABBER_PIXFMT_MAX)
		{
			return -EINVAL;
		}
		else
		{
			pixfmt=v4l2_model_get_support_pixformat(f->index);
			if(pixfmt ==NULL) return -EINVAL;
			//printk("%s..pixfmt=%d.\n",__func__,f->index);
			f->index = index;
			f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			strscpy(f->description, pixfmt->name, sizeof(f->description));
			f->pixelformat=pixfmt->fourcc;
		}
	}
	return 0;
}
void framegrabber_g_Curr_input_framesize(struct hws_video *dev,int *width,int *height)
{
	struct hws_pcie_dev *pdx = dev->dev;
	int index = dev->index;
	*width= pdx->m_pVCAPStatus[index][0].dwWidth;
	*height=pdx->m_pVCAPStatus[index][0].dwHeight;
}
const framegrabber_pixfmt_t *framegrabber_g_out_pixelfmt(struct hws_video *dev)
{
	return &support_pixfmts[dev->current_out_pixfmt];
}
static int hws_vidioc_g_fmt_vid_cap(struct file *file, void *fh, struct v4l2_format *fmt)
{
	struct hws_video *videodev = video_drvdata(file);
	//struct v4l2_pix_format *pix = &fmt->fmt.pix;
	const framegrabber_pixfmt_t *pixfmt;
	v4l2_model_timing_t *p_SupportmodeTiming;
	//printk( "%s(%d)\n", __func__,videodev->index);
	//printk( "w=%d,h=%d\n",fmt->fmt.pix.width,fmt->fmt.pix.height);
	pixfmt=framegrabber_g_out_pixelfmt(videodev);
	if(pixfmt)
	{
		//framegrabber_g_Curr_input_framesize(videodev,&width,&height);
		p_SupportmodeTiming = v4l2_model_get_support_videoformat(videodev->current_out_size_index);
		if(p_SupportmodeTiming ==NULL) return -EINVAL;
		fmt->fmt.pix.width=p_SupportmodeTiming->frame_size.width;
		fmt->fmt.pix.height=p_SupportmodeTiming->frame_size.height;
		fmt->fmt.pix.field=V4L2_FIELD_NONE; //Field
		fmt->fmt.pix.pixelformat  = pixfmt->fourcc;
		fmt->fmt.pix.bytesperline = (fmt->fmt.pix.width * pixfmt->depth) >> 3;
		fmt->fmt.pix.sizeimage =	fmt->fmt.pix.height * fmt->fmt.pix.bytesperline;
		fmt->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;
		//printk("%s....f->fmt.pix.width=%d.f->fmt.pix.height=%d.\n",__func__,fmt->fmt.pix.width,fmt->fmt.pix.height);
		return 0;
	}

	return -EINVAL;
}
static v4l2_model_timing_t *Get_input_framesizeIndex(int width,int height)
{
	int i;
	for(i =0;i<V4L2_MODEL_VIDEOFORMAT_NUM;i++)
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
	//int TimeingIndex = f->index;
	//printk( "%s(%d)\n", __func__,videodev->index);
	//printk( "pix->height =%d  pix->width =%d \n", pix->height,pix->width);

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
            printk("%s.. format not support (even fallback)\n", __func__);
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
		pix->colorspace = V4L2_COLORSPACE_REC709;//V4L2_COLORSPACE_SMPTE170M;
		pix->priv = 0;
		//return -EINVAL;
		return 0;
	}
	pix->field = V4L2_FIELD_NONE;
	pix->width=pModeTiming->frame_size.width;
	pix->height=pModeTiming->frame_size.height;
	pix->bytesperline = (pix->width * fmt->depth) >> 3;
	pix->sizeimage =pix->height * pix->bytesperline;
	pix->colorspace = V4L2_COLORSPACE_REC709;//V4L2_COLORSPACE_SMPTE170M;
	pix->priv = 0;

	//printk("%s<<pix->width=%d.pix->height=%d.\n",__func__,pix->width,pix->height);
	return 0;


	//----------------------------------
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
	//printk( "%s()\n", __func__);
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
	mutex_unlock(&videodev->video_lock);
	return 0;
}
/* frame rate of the input as measured by the card (60 until it reports one) */
static int hws_input_fps(struct hws_video *videodev)
{
	u32 r = READ_ONCE(videodev->dev->m_pVCAPStatus[videodev->index][0].dwFrameRate);

	return (r >= 1 && r <= 240) ? r : 60;
}

/* frames come at the input's rate; there is no frame rate to choose */
int hws_vidioc_g_parm(struct file *file,void *fh, struct v4l2_streamparm *setfps)
{
	struct hws_video *videodev = video_drvdata(file);

	if (setfps->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	memset(&setfps->parm.capture, 0, sizeof(setfps->parm.capture));
	setfps->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	setfps->parm.capture.timeperframe.numerator = 1;
	setfps->parm.capture.timeperframe.denominator = hws_input_fps(videodev);
	return 0;
}
static int hws_vidioc_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize)
{
	//struct hws_video *videodev = video_drvdata(file);
	const framegrabber_pixfmt_t *pixfmt;
	v4l2_model_timing_t *p_SupportmodeTiming;
	int width=0,height=0;
	int frameRate;
	//printk( "%s(%d)-FrameIndex=[%d]\n", __func__,videodev->index,fsize->index);
	//----------------------------
	pixfmt=framegrabber_g_support_pixelfmt_by_fourcc(fsize->pixel_format);
	if(pixfmt==NULL)
	{
		//printk("%s..\n",__func__);
		return -EINVAL;
	}
	p_SupportmodeTiming = v4l2_model_get_support_videoformat(fsize->index);
	if(p_SupportmodeTiming == NULL)
	{
		//printk("%s. invalid framesize[%d]\n",__func__,fsize->index);
		return -EINVAL;
	}
	width = p_SupportmodeTiming->frame_size.width;
	height = p_SupportmodeTiming->frame_size.height;
	frameRate = p_SupportmodeTiming->refresh_rate;

	//printk("%s...supportframesize[%d] width=%d height=%d Framerate=%d..\n",__func__,fsize->index,width,height,frameRate); //12
	if((width ==0) || (height ==0))
	{
		//printk("%s. invalid framesize 2\n",__func__);
		return -EINVAL;
	}
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->pixel_format=pixfmt->fourcc;
	fsize->discrete.width=width;
	fsize->discrete.height=height;
	//fsize->discrete.denominator = frameRate;
	//fsize->discrete..numerator =1 ;
	//-------------------------------
	//width = videodev->current_out_width;
	//height = videodev->curren_out_height;
	//fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	//fsize->pixel_format=videodev->pixfmt;
	//fsize->discrete.width=width;
	//fsize->discrete.height=height;
	return 0;


}

static int hws_vidioc_enum_input(struct file *file, void *priv,struct v4l2_input *i)
{
	struct hws_video *videodev = video_drvdata(file);
	if (i->index != 0)
		return -EINVAL;
	i->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(i->name, KBUILD_MODNAME, sizeof(i->name));
	i->std = 0;	/* HDMI: no analog TV standard */
	i->capabilities = 0;
	i->status = READ_ONCE(videodev->dev->m_curr_No_Video[videodev->index]) ? V4L2_IN_ST_NO_SIGNAL : 0;

	return 0;
}

static int hws_vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	//struct hws_video *videodev = video_drvdata(file);
	int Index;
	Index = *i;
	//printk( "%s(%d)-index =%d\n", __func__,videodev->index,Index);

	#if 0
	if(Index <0 ||Index >=V4L2_MODEL_VIDEOFORMAT_NUM)
	{
	return   -EINVAL;
}
else
{
*i = Index;
}
#else
if(Index >0)
{
	return   -EINVAL;
}
else
{
	*i = 0;
}
#endif
return 0;
}

static int hws_vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	#if 0
	struct hws_video *videodev = video_drvdata(file);

	int Index;
	v4l2_model_timing_t *p_SupportmodeTiming;
	Index = i;
	if(Index <0 ||Index >=V4L2_MODEL_VIDEOFORMAT_NUM)
	{
	return   -EINVAL;
}
p_SupportmodeTiming = v4l2_model_get_support_videoformat(Index);
videodev->current_out_size_index = Index;
videodev->current_out_width = p_SupportmodeTiming->frame_size.width;
videodev->curren_out_height = p_SupportmodeTiming->frame_size.height;
printk( "%s(%d)- %dx%d \n", __func__,i,videodev->current_out_width,videodev->curren_out_height);
#endif
//printk( "%s(%d)\n", __func__,i);
return i ? -EINVAL : 0;
}
static ssize_t hws_read(struct file *file,char *buf,size_t count, loff_t *ppos)
{
	//printk( "%s()\n", __func__);
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

    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
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
    spin_lock_irqsave(&pdx->videoslock[videodev->index], flags);
    if (videodev->fileindex > 0)
        videodev->fileindex--;
    spin_unlock_irqrestore(&pdx->videoslock[videodev->index], flags);

    /* release vb2 queue resources; stops streaming and waits for the worker */
    mutex_lock(&ctx->qmutex);
    vb2_queue_release(&ctx->vbq);
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

//-------------------
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
#if 0
static int hws_vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
//struct hws_video *videodev = video_drvdata(file);
//printk( "%s(ch-%d)\n", __func__,videodev->index);
#if 0
StartVideoCapture(videodev->dev,videodev->index);
#endif
return(vb2_ioctl_streamon(file,priv,i));

}
static int hws_vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
//struct hws_video *videodev = video_drvdata(file);
//printk( "%s(ch-%d)\n", __func__,videodev->index);
#if 0
StopVideoCapture(videodev->dev,videodev->index);
#endif
return(vb2_ioctl_streamoff(file,priv,i));

}
#endif
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
	fival->discrete.numerator = 1;
	fival->discrete.denominator = hws_input_fps(videodev);
	return 0;
}
int hws_vidioc_s_parm(struct file *file, void *fh,struct v4l2_streamparm *a)
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

static int hws_prepare_buf(struct file *file, void *priv, struct v4l2_buffer *b)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	ret = vb2_prepare_buf(&ctx->vbq, HWS_VB2_MDEV(file) b);
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

static int hws_qbuf(struct file *file, void *priv, struct v4l2_buffer *b)
{
	struct hws_vfh_ctx *ctx = hws_ctx_from_file(file);
	int ret;

	if (mutex_lock_interruptible(&ctx->qmutex))
		return -ERESTARTSYS;
	ret = vb2_qbuf(&ctx->vbq, HWS_VB2_MDEV(file) b);
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
	 * Uninterruptibly, so a signal can't turn into EPOLLERR; it is held only
	 * briefly (a blocking DQBUF drops it while waiting).
	 */
	mutex_lock(&ctx->qmutex);
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
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_enum_input = hws_vidioc_enum_input,
	.vidioc_g_input = hws_vidioc_g_input,
	.vidioc_s_input = hws_vidioc_s_input,
	//.vidioc_log_status = vidioc_log_status,
	//.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	//.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_g_parm = hws_vidioc_g_parm,
	.vidioc_s_parm = hws_vidioc_s_parm,
};

static void hws_buffer_finish(struct vb2_buffer *vb)
{
	return;
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
        /* larger than the biggest frame twice over is only a way to eat memory */
        if (sizes[0] < size || sizes[0] > PAGE_ALIGN(MAX_VIDEO_SCLAER_SIZE))
            return -EINVAL;
        return 0;
    }

    *num_planes = 1;
    sizes[0] = size;
    return 0;
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
//-----------------------------------------
const unsigned char  g_YUVColors [MAX_COLOR][3] = {
	{128, 16, 128},     // BLACK
	{128, 235 , 128},    // WHITE
	{16, 211, 146},     // YELLOW
	{166, 170, 16},     // CYAN
	{54, 145, 34},      // GREEN
	{202, 106, 222},    // MAGENTA
	{90, 81, 240},      // RED
	{240, 41, 109},     // BLUE
	{128, 125, 128},    // GREY
};


static void SetNoVideoMem(uint8_t * pDest, int  w,int h)
{
	int x,y;
	uint8_t *pST;
	uint8_t *pNS;
	pST = (uint8_t *)pDest;
	//printk("SetNoVideoMem[%d-%d]\n",w,h);

	for(x=0;x<w/2;x++)
	{
		pST[0] = 41;
		pST[1] = 240;
		pST[2] = 41;
		pST[3] = 109;
		pST +=4;
	}

	pNS = pDest+w*2;
	for(y=1;y<h;y++)
	{
		memcpy(pNS,pDest,w*2);
		pNS = pNS+w*2;
	}
}
//---------------------------------------------------
/*
 * Append one captured packet to the ALSA ring. Everything that touches the
 * runtime happens under ring_lock and only while the stream runs, so a
 * concurrent STOP/close can't free dma_area underneath us.
 */
static void _deliver_samples(struct hws_audio *drv, const u8 *aud_data, u32 aud_len)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	u32 frame_bytes, frames, chunk;
	unsigned long flags;
	int elapsed = 0;

	spin_lock_irqsave(&drv->ring_lock, flags);
	substream = drv->substream;
	runtime = substream ? substream->runtime : NULL;
	if (!drv->running || !runtime || !runtime->dma_area ||
	    drv->ring_size_byframes == 0 || drv->period_size_byframes == 0 ||
	    drv->channels == 0) {
		spin_unlock_irqrestore(&drv->ring_lock, flags);
		return;
	}

	frame_bytes = 2 * drv->channels;
	frames = aud_len / frame_bytes;
	/* the packet may be larger than the whole ring: wrap as often as needed */
	while (frames) {
		chunk = min(frames, drv->ring_size_byframes - drv->ring_wpos_byframes);
		memcpy(runtime->dma_area + drv->ring_wpos_byframes * frame_bytes,
		       aud_data, chunk * frame_bytes);
		aud_data += chunk * frame_bytes;
		frames -= chunk;
		drv->ring_wpos_byframes = (drv->ring_wpos_byframes + chunk) % drv->ring_size_byframes;
		drv->period_used_byframes += chunk;
	}
	/* check if a period available */
	if (drv->period_used_byframes >= drv->period_size_byframes) {
		elapsed = 1;
		drv->period_used_byframes %= drv->period_size_byframes;
	}
	spin_unlock_irqrestore(&drv->ring_lock, flags);

	/* outside ring_lock: this may call back into trigger(STOP) and pointer() */
	if (elapsed)
		snd_pcm_period_elapsed(substream);
}

void audio_data_process(struct work_struct *p_work)
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
			_deliver_samples(drv, bBuf, aud_len);

		spin_lock_irqsave(&pdx->audiolock[dwAudioCh], flags);
		if (ai->pStatusInfo[nIndex].byLock == MEM_READING)
			ai->pStatusInfo[nIndex].byLock = MEM_UNLOCK;
		spin_unlock_irqrestore(&pdx->audiolock[dwAudioCh], flags);
	}
}
//-------------------------------
static void FHD_To_800X600_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int x,y;
	BYTE*pSrcBuf;
	//   BYTE*pDestBuf;
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
	//int *pSrcData;
	//   int *pDestData;
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
	//int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	//int *pSrcData;
	//int *pDestData;
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
	// int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	//   int *pSrcData;
	//int *pDestData;
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
	//---
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
		//------------------
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
	//---
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
		//------------------
	}


}

static void SD_PAL_To_HD_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int x,y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	// int dwJ;
	//   BYTE*pDstYUV;
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
		//------------------
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
	//   int dwJ;
	//BYTE*pDstYUV;
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
		//------------------
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
	//BYTE*pDestBuf;
	DWORD *pSrcData;
	DWORD *pDestData;
	int    out_w_size;
	//BYTE*pSrcTmp;
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
	//   BYTE *pDestData;
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
	//   BYTE *pDestData;
	int dwJ;
	pSrcBuf = pSrc;
	pDestBuf = pOut;

	//-- 1920X1080 0->720X576
	//-------------------
	pDstYUV = pDestBuf;
	for(dwJ=0; dwJ<18*out_w; dwJ++)
	{
		pDstYUV[0] = 0x10;
		pDstYUV[1] = 0x80;
		pDstYUV +=2;
	}
	//------------------
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
	//------------
	pDstYUV = pDestBuf;
	for(dwJ=0; dwJ<18*out_w; dwJ++)
	{
		pDstYUV[0] = 0x10;
		pDstYUV[1] = 0x80;
		pDstYUV +=2;
	}
	//------------
}

void HD_To_SD_NTSC_Scaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	int y;
	BYTE*pSrcBuf;
	BYTE*pDestBuf;
	//int *pSrcData;
	// int *pDestData;
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
	//int dwJ;
	pSrcBuf = pSrc;
	pDestBuf = pOut;
	//BYTE*pDstYUV;
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
	//   BYTE*pDstYUV;
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
	//   BYTE*pDstYUV;
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
	//DbgPrint("[MV]VideoRotate90deg %dX%d->%dX%d\n",in_w,in_h,out_w,out_h);
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
static void All_VideoScaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
{
	BYTE *pSrcBuf = pSrc;
	BYTE *pDestBuf = pOut;
	int missX = 0, missY = 0, dumyX = 0, dumyY = 0;
	int copyW, copyH, padR, padB;
	int y;

	if (in_w > out_w)
		missX = ((in_w - out_w) / 2) & ~1;
	else
		dumyX = ((out_w - in_w) / 2) & ~1;
	if (in_h > out_h)
		missY = (in_h - out_h) / 2;
	else
		dumyY = (out_h - in_h) / 2;

	copyW = min(out_w - 2 * dumyX, in_w - missX);
	copyH = min(out_h - 2 * dumyY, in_h - missY);
	padR = out_w - dumyX - copyW;
	padB = out_h - dumyY - copyH;

	FillBlackYUYV(pDestBuf, dumyY * out_w);
	pDestBuf += dumyY * out_w * 2;
	pSrcBuf += missY * in_w * 2;

	for (y = 0; y < copyH; y++)
	{
		FillBlackYUYV(pDestBuf, dumyX);
		pDestBuf += dumyX * 2;
		memcpy(pDestBuf, pSrcBuf + missX * 2, copyW * 2);
		pDestBuf += copyW * 2;
		FillBlackYUYV(pDestBuf, padR);
		pDestBuf += padR * 2;
		pSrcBuf += in_w * 2;
	}
	FillBlackYUYV(pDestBuf, padB * out_w);
}

static void VideoScaler(BYTE *pSrc,BYTE *pOut,int in_w,int in_h,int out_w,int out_h)
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
		All_VideoScaler(pSrc,pOut,in_w,in_h,out_w,out_h);
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
	//DWORD dwDstPitch = nw;
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
		//DbgPrint("nh");
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
//-----------
static void SetDeInterlace(BYTE *pSrc,BYTE *pOut, int nw,int nh,int interlace)
{
	int dwSrcPitch = nw*2;
	int h;
	//DbgPrint("[MV]SetDeInterlace W=%d H=%d interlace=%d \n",nw,nh,interlace);
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
//----------------------------------------------------------------
static void FillYUU2(BYTE *pSrc,BYTE *pOut, int nw,int nh,int interlace)
{
	BYTE *pVideoMem = pOut ;// g_pD1TempBuffer;
	//	DWORD dwDstPitch = nw;
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
		//---------------------------------
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
			//------------------------------
			memcpy(pDstYUV,(pDstYUV-nw*2),nw*2);
			pDstYUV = pDstYUV+ nw*2;
			//-----------------------------------
		}
		/* the last field line (shown twice) has no next line for its chroma: black */
		FillBlackYUYV(pDstYUV - 2*nw*2, 2*nw);
		//---------------------------------

	}

}

//--------------------------------

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
 * How many frames the consumers can take right now: the smallest number of
 * queued buffers over the streaming consumers that have any. A consumer with
 * none queued (paused, or holding all its buffers) is ignored, so it can't
 * cost the others frames. Returns 0 if no consumer has a buffer queued and
 * INT_MAX if nobody streams.
 */
static int hws_min_queued(struct hws_video *videodev)
{
    struct hws_vfh_ctx *ctx;
    struct list_head *pos;
    unsigned long flags;
    int min = INT_MAX;
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
        if (n > 0 && n < min)
            min = n;
    }
    spin_unlock_irqrestore(&videodev->consumers_lock, flags);
    if (streaming && min == INT_MAX)
        return 0;
    return min;
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
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, dst, in_width, in_height, nWidth, nHeight);
            } else {
                if (!pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf) {
                    return -1;
                }
                FillYUU2(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                /* scale to the unrotated size, then rotate (as for YUYV) */
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, in_width, in_height, nHeight, nWidth);
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
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, dst, in_width, in_height, nWidth, nHeight);
            } else {
                if (!pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf) {
                    return -1;
                }
                FillNV12ToYUY2(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                /* scale to the unrotated size, then rotate (as for YUYV) */
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, in_width, in_height, nHeight, nWidth);
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
                    VideoScaler(bBuf, dst, in_width, in_height, nWidth, nHeight);
                } else {
                    SetDeInterlace(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                    VideoScaler(pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, dst, in_width, in_height, nWidth, nHeight);
                }
            } else {
                if (!pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf) {
                    return -1;
                }
                SetDeInterlace(bBuf, pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, in_width, in_height, interlace);
                VideoScaler(pdx->m_VideoInfo[nDecoder].m_pVideoScalerBuf, pdx->m_VideoInfo[nDecoder].m_pRotateVideoBuf, in_width, in_height, nHeight, nWidth);
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

void video_data_process(struct kthread_work *p_work)
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
        avail = hws_min_queued(videodev);

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
                    SetNoVideoMem(first->mem, nWidth, nHeight);
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
         * an app with enough buffers still gets every frame.
         */
        while (backlog > 1 && backlog > avail) {
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

	//printk( "%s(): %x \n", __func__, index);
	int width,height;
	width= dev->m_pVCAPStatus[index][0].dwWidth;
	height=dev->m_pVCAPStatus[index][0].dwHeight;
	dev->video[index].current_out_pixfmt =0;
	dev->video[index].current_out_size_index = 0;
	dev->video[index].current_out_width = width;
	dev->video[index].curren_out_height = height;
	dev->video[index].current_out_framerate = 60;
	dev->video[index].Interlaced = 0;
	//printk( "%s(%dx%d):  \n", __func__, width,height);

}

static void hws_adapters_init(struct hws_pcie_dev *dev)
{
	int i;
	for (i = 0; i <MAX_VID_CHANNELS; i++) {
		hws_get_video_param(dev,i);
	}
}
static int hws_video_register(struct hws_pcie_dev *dev)
{
	int i;
	int err=-1;

	for(i=0;i<dev->m_nCurreMaxVideoChl;i++){
		struct hws_video *video = &dev->video[i];
		struct video_device *vdev = &video->vdev;

		video->v4l2_dev.release = hws_v4l2_dev_release;
		err = v4l2_device_register(&dev->pdev->dev, &video->v4l2_dev);
		if(err<0){
			printk(KERN_ERR " v4l2_device_register %d error! \n", i);
			goto fail;
		}
		/* dropped by hws_v4l2_dev_release() */
		kref_get(&dev->kref);
		dev->nr_v4l2_registered++;

		video->fileindex =0;
		video->pixfmt = V4L2_PIX_FMT_YUYV;
		err = hws_init_controls(video);
		if (err) {
			printk(KERN_ERR " control init %d failed: %d\n", i, err);
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
			printk(KERN_ERR " video_register_device %d failed !!!!! \n", i);
			goto fail;
		}
	}
	return 0;
fail:
	hws_video_unregister_nodes(dev);
	hws_video_release_v4l2(dev);
	return err;
}

/* HDMI 0x39[3:0] - CS_DATA[27:24] 0 for reserved values*/
static const int cs_data_fs[] = {
	44100,
	0,
	48000,
	32000,
	0,
	0,
	0,
	0,
	88200,
	768000,
	96000,
	0,
	176000,
	0,
	192000,
	0,
};
#if 1
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
	.buffer_bytes_max 	=	64*1024,
	/* the card delivers 4096-byte packets; smaller periods can't be honoured */
	.period_bytes_min 	=	4096,
	.period_bytes_max 	=	16*1024,
	.periods_min 		=	2,
	.periods_max 		=	255,
};
#else
static struct snd_pcm_hardware audio_pcm_hardware ={
	.info =  (SNDRV_PCM_INFO_INTERLEAVED |SNDRV_PCM_INFO_BLOCK_TRANSFER ),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = SNDRV_PCM_RATE_KNOT | SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max =2,
	.period_bytes_min = HWS_AUDIO_CELL_SIZE,
	.period_bytes_max = HWS_AUDIO_CELL_SIZE,
	.periods_min      = 4,
	.periods_max      = 4,
	.buffer_bytes_max = HWS_AUDIO_CELL_SIZE*4,
};
#endif
int hws_pcie_audio_open(struct snd_pcm_substream *substream)
{
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;


	unsigned long flags;

	//printk(KERN_INFO "%s() index:%x\n",__func__,drv->index);
	runtime->hw = audio_pcm_hardware;
	spin_lock_irqsave(&drv->ring_lock, flags);
	drv->sample_rate_out        = 48000;
	drv->channels               = 2;
	drv->running                = false;
	drv->substream = substream;
	spin_unlock_irqrestore(&drv->ring_lock, flags);
	//snd_pcm_hw_constraint_minmax(runtime,SNDRV_PCM_HW_PARAM_RATE,setrate,setrate);
	return 0;
}

/* deliveries run in the IRQ thread: wait for one in progress */
static void hws_audio_sync(struct hws_audio *drv)
{
	struct hws_pcie_dev *pdx = drv->dev;

	/* irq_lock: remove can't free (and reuse) the IRQ in between */
	mutex_lock(&pdx->irq_lock);
	if (pdx->irq_line != -1)
		synchronize_irq(pdx->irq_line);
	mutex_unlock(&pdx->irq_lock);
}

int hws_pcie_audio_close(struct snd_pcm_substream *substream)
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
int hws_pcie_audio_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	//printk(KERN_INFO "%s() \n",__func__);
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

int hws_pcie_audio_hw_free(struct snd_pcm_substream *substream)
{
	//printk(KERN_INFO "%s() \n",__func__);
	return snd_pcm_lib_free_pages(substream);
}

int hws_pcie_audio_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	//struct hws_pcie_dev *dev= drv->dev;
	//int i;
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
int hws_pcie_audio_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct hws_audio *chip = snd_pcm_substream_chip(substream);
	struct hws_pcie_dev *dev= chip->dev;
	unsigned long flags;
	switch(cmd){
		case SNDRV_PCM_TRIGGER_START:
			/* shut down / being removed: the card stays off */
			if (READ_ONCE(dev->removing))
				return -ENODEV;
			//HWS_PCIE_READ(HWS_DMA_BASE(chip->index), HWS_DMA_STATUS);
			//start dma
			//HWS_PCIE_WRITE(HWS_INT_BASE, HWS_DMA_MASK(chip->index), 0x00000001);
			//HWS_PCIE_WRITE(HWS_DMA_BASE(chip->index), HWS_DMA_START, 0x00000001);
			//printk(KERN_INFO "SNDRV_PCM_TRIGGER_START index:%x\n",chip->index);
			/* reset the capture queue before deliveries are accepted again */
			StartAudioCapture(dev,chip->index);
			spin_lock_irqsave(&chip->ring_lock, flags);
			chip->ring_wpos_byframes = 0;
			chip->period_used_byframes = 0;
			chip->running = true;
			spin_unlock_irqrestore(&chip->ring_lock, flags);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
			//stop dma
			//HWS_PCIE_WRITE(HWS_INT_BASE, HWS_DMA_MASK(chip->index), 0x000000000);
			//HWS_PCIE_WRITE(HWS_DMA_BASE(chip->index), HWS_DMA_START, 0x00000000);
			//printk(KERN_INFO "SNDRV_PCM_TRIGGER_STOP index:%x\n",chip->index);
			StopAudioCapture(dev,chip->index);
			spin_lock_irqsave(&chip->ring_lock, flags);
			chip->running = false;
			spin_unlock_irqrestore(&chip->ring_lock, flags);
			break;
		default:
			return -EINVAL;
			break;
	}
	return 0;
}
//-------------------------------------------------


//-------------------------------------------------
static snd_pcm_uframes_t hws_pcie_audio_pointer(struct snd_pcm_substream *substream)
{
	struct hws_audio *drv = snd_pcm_substream_chip(substream);
	//struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t pos;
	int dwAudioCh;
	unsigned long flags;
	dwAudioCh = drv->index;
	//printk(KERN_INFO "%s() index:%x\n",__func__,dwAudioCh);
	spin_lock_irqsave(&drv->ring_lock,flags);  //spin_lock
	pos = drv->ring_wpos_byframes;
	spin_unlock_irqrestore(&drv->ring_lock,flags); //spin_unlock
	return pos;
}

struct snd_pcm_ops hws_pcie_pcm_ops ={
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
			printk(KERN_ERR "%s() ERROR: snd_card_new failed <%d>\n",__func__, ret);
			goto fail;
		}
		/* from here on freeing the card drops this reference */
		kref_get(&dev->kref);
		card->private_data = &dev->audio[i];
		card->private_free = hws_snd_card_free;
		strscpy(card->driver, KBUILD_MODNAME, sizeof(card->driver));
		strscpy(card->shortname, audioname, sizeof(card->shortname));
		strscpy(card->longname, card->shortname, sizeof(card->longname));

		ret = snd_pcm_new(card,pcm_name,0,0,1,&pcm);
		if (ret < 0){
			printk(KERN_ERR "%s() ERROR: snd_pcm_new failed <%d>\n",__func__, ret);
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
		//----------------------------
		dev->audio[i].sample_rate_out        = 48000;
		dev->audio[i].channels               = 2;
		dev->audio[i].resampled_buf_size = dev->audio[i].sample_rate_out * 2/* sample bytes */ * dev->audio[i].channels /* channels */;
		ret = snd_card_register(card);
		if ( ret < 0) {
			printk(KERN_ERR "%s() ERROR: snd_card_register failed\n",__func__);
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
//-------------------
//static unsigned long video_data[MAX_VID_CHANNELS];
//static unsigned long audio_data[MAX_VID_CHANNELS];



static void  WRITE_REGISTER_ULONG (struct hws_pcie_dev *pdx,u32 RegisterOffset,u32 Value)
{
	//map_bar0_addr[RegisterOffset/4] = Value;
	char *bar0;
	bar0 = (char*)pdx->map_bar0_addr;
	iowrite32(Value,bar0+RegisterOffset);
	//map_bar0_addr[RegisterOffset/4] = Value;

}

static u32 READ_REGISTER_ULONG (struct hws_pcie_dev *pdx,u32 RegisterOffset)
{
	char *bar0;
	bar0 = (char*)pdx->map_bar0_addr;
	//return(map_bar0_addr[RegisterOffset/4]);
	return(ioread32(bar0+RegisterOffset));
}
//----------------------------------------------
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
		msleep(10);
	}
	printk(KERN_WARNING "HwsCapture: DSP did not go idle, status %X\n", statusreg);
	return -ETIMEDOUT;
}

static void StopDsp(struct hws_pcie_dev *pdx)
{
	//int j, i;
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
	pdx->m_bVCapStarted[index] = en;
	if(pdx->m_PciDeviceLost) return;
	/* one register holds the enable bits of all channels: serialize the RMW */
	spin_lock_irqsave(&pdx->reg_lock, flags);
	status = READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE+2*PCIE_BARADDROFSIZE));
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
	WRITE_REGISTER_ULONG(pdx,( CVBS_IN_BASE + (2 * PCIE_BARADDROFSIZE)), status);
	status = READ_REGISTER_ULONG(pdx,(CVBS_IN_BASE+2*PCIE_BARADDROFSIZE));
	spin_unlock_irqrestore(&pdx->reg_lock, flags);
	//printk("EnableVideoCapture[%d]=%X %d \n",index,status,pdx->m_bVCapStarted[index]);
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
	//printk("EnableAudioCapture =%X",status);
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
	//int frame_size;
	//int buf_cnt;

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
	//DbgPrint("[MV-X1]-CH0 SetVideoFormteSize = %d %d \n",m_format[ch].HLAF_SIZE,m_format[ch].DWON_SIZE);

	return 1;

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
	//------------
	for(i=0; i<pdx->m_nMaxChl; i++)
	{
		pdx->m_pbyVideoBuffer[i] = dma_alloc_coherent(&pdx->pdev->dev, pdx->m_MaxHWVideoBufferSize, &pdx->m_pbyVideo_phys[i], GFP_KERNEL);
		if(pdx->m_pbyVideoBuffer[i]== NULL)
		{
			printk(KERN_ERR "HwsCapture: DMA buffer %d allocation failed\n", i);
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
	//int i;

	//BUG_ON(!lro);

	//if (lro->msix_enabled) {
	//	for (i = 0; i < lro->irq_user_count; i++) {
	//		printk("Releasing IRQ#%d\n", lro->entry[i].vector);
	//		free_irq(lro->entry[i].vector, &lro->user_irq[i]);
	//	}
	//}
	//else

	if (lro->irq_line != -1) {
		//printk("Releasing IRQ#%d\n", lro->irq_line);
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
	if (pdx->wq)
		destroy_workqueue(pdx->wq);
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

	for (i = 0; i < dev->m_nCurreMaxVideoChl; i++)
		video_unregister_device(&dev->video[i].vdev);	/* no-op if never registered */
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

//----------------------------
static void hws_remove(struct pci_dev *pdev)
{
	int i;
	struct hws_pcie_dev *dev =
	(struct hws_pcie_dev*) pci_get_drvdata(pdev);
	//----------------------------
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
//---------------------------------------
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
		//DbgPrint("Re StartAudioCapture =%d",index);
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
	//--------------------
	CheckCardStatus(pdx);
	//--------------------
	spin_lock_irqsave(&pdx->videoslock[index], flags);
	for (j = 0; j<MAX_VIDEO_QUEUE; j++)
	{
		pdx->m_pVCAPStatus[index][j].byLock = MEM_UNLOCK;
		pdx->m_pVCAPStatus[index][j].byPath = 2;
		pdx->m_VideoInfo[index].pStatusInfo[j].byLock = MEM_UNLOCK;
	}
	pdx->m_nRDVideoIndex[index]=0;
	pdx->tstats[index].last_frame_ns = 0;
	pdx->tstats[index].interval_min = U64_MAX;
	pdx->tstats[index].window_start_ns = ktime_get_ns();
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
	//int inc=0;

	if(pdx->m_bVCapStarted[index] ==0) return;
	//pdx->m_nVideoIndex[index] =0;
	pdx->m_VideoInfo[index].dwisRuning = 0;
	pdx->m_bVideoStop[index] = 1;
	pdx->m_pVideoEvent[index] = 0;
	pdx->m_bChangeVideoSize[index] = 0;
	#if 0
	while(1)
	{
	if(pdx->m_bVideoStop[index] ==0)
	{
	break;
}
inc++;
if(inc >2000)
{
break;
}
msleep(10);
}
#endif
EnableVideoCapture(pdx,index,0);
pdx->m_bVCapIntDone[index] = 0;
}
static void StopAudioCapture(struct hws_pcie_dev *pdx,int index)
{
	//int inc=0;
	if(pdx->m_bAudioRun[index] ==0) return;
	pdx->m_bAudioRun[index] = 0;
	pdx->m_bAudioStop[index] = 1;
	pdx->m_nAudioBufferIndex[index] =0;
	pdx->m_AudioInfo[index].dwisRuning =0;
	#if 0
	while(1)
	{
	if(pdx->m_bAudioStop[index] ==0)
	{
	break;
}
inc++;
if(inc >2000)
{
break;
}
msleep(10);
}
#endif
EnableAudioCapture(pdx,index,0);
}
//-----------------------------


//-----------------------------
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
	if ((queued_ahead || hws_list_count(bufs) > HWS_INLINE_FANOUT) && !list_empty(&v->fanout_free)) {
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
	u64 frame_ts;
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
		pdx->direct_nt[ch] = READ_ONCE(nt_copy) != 0;	/* the same for both halves */
		pdx->direct_frame[ch] = true;
		if (interlace)
			hws_copy_doubled(first->mem, src, 0, halfsize, nw * 2, pdx->direct_nt[ch]);
		else
			hws_dcopy(first->mem, src, halfsize, pdx->direct_nt[ch]);

		/* see the overrun check in MemCopyVideoToSteam() */
		smp_rmb();
		if (READ_ONCE(pdx->m_half_cnt[ch][0]) != peer_at_irq || hws_copy_too_late(pdx, ch, t_irq))
		{
			atomic_inc(&pdx->tstats[ch].dma_overrun);
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
	if (interlace)
		hws_copy_doubled(first->mem, src, halfsize, copysize, nw * 2, pdx->direct_nt[ch]);
	else
		hws_dcopy(first->mem + halfsize, src, copysize, pdx->direct_nt[ch]);
	if (pdx->direct_nt[ch])
		wmb();	/* non-temporal stores are weakly ordered: done before the buffer is */
	smp_rmb();
	if (READ_ONCE(pdx->m_half_cnt[ch][1]) != peer_at_irq || hws_copy_too_late(pdx, ch, t_irq))
	{
		atomic_inc(&pdx->tstats[ch].dma_overrun);
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
static int MemCopyVideoToSteam(struct hws_pcie_dev *pdx,int nDecoder,int mVideoBufIndex)
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
	u32 cnt_self, peer_at_irq;
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
		atomic_inc(&pdx->tstats[nDecoder].stale_half);
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
				pdx->m_nVideoHalfDone[nDecoder] =0;
				hws_put_mark(pMask);
				return -1;
			}
			else
			{
				pdx->m_nVideoHalfDone[nDecoder] =0;
			}
		}
		else if ((s32)(peer_at_irq - fmt_cnt0) <= 0)
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

			pdx->m_nVideoHalfDone[nDecoder] = 1;
			pdx->m_half1_gen[nDecoder] = gen;
			pdx->m_half1_ts[nDecoder] = t_irq;
		}
	}
	//-------------------------------
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
			if(mVideoBufIndex== 0)
			{
				bBuf += halfsize;
			}
			memcpy(bBuf,pSrcBuf,copysize);

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
				atomic_inc(&pdx->tstats[nDecoder].dma_overrun);
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
//------------------------------------
int MemCopyAudioToSteam( struct hws_pcie_dev  *pdx,int dwAudioCh,int half)
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

	//-----------------------------------------------------
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

int SetAudioQuene( struct hws_pcie_dev *pdx,int dwAudioCh,int half)
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
	//-------------------
	if(pdx->m_DeviceHW_Version ==0)
	{
		atomic_inc(&pdx->m_dwSWFrameRate[nDecoder]);
	}
	//-------------------
	pdx->m_nVideoBusy[nDecoder] = 1;
	//-------------------------------
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
//-----------------------------
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

	/* being removed after the card vanished: a shared line's neighbour fired */
	if (READ_ONCE(pdx->m_PciDeviceLost) && READ_ONCE(pdx->removing))
		return IRQ_NONE;
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
		if (++pdx->irq_stuck_passes >= 1000 && !READ_ONCE(pdx->irq_storm_off))
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
	//int i;
	struct hws_pcie_dev *lro;

	BUG_ON(!pdev);

	/* allocate zeroed device book keeping structure */
	lro = kzalloc(sizeof(struct hws_pcie_dev), GFP_KERNEL);
	if (!lro) {
		printk("Could not kzalloc(hws_pcie_dev).\n");
		return NULL;
	}

	//lro->magic = MAGIC_DEVICE;
	//lro->config_bar_idx = -1;
	//lro->user_bar_idx = -1;
	//lro->bypass_bar_idx = -1;
	lro->irq_line = -1;

	/* create a device to driver reference */
	dev_set_drvdata(&pdev->dev, lro);
	/* create a driver to device reference; held until the last file is closed
	 * (querycap and vb2 use pdev after an unbind) */
	lro->pdev = pci_dev_get(pdev);
	//printk("probe() lro = 0x%p\n", lro);

	/* Set up data user IRQ data structures */
	//for (i = 0; i < MAX_USER_IRQ; i++) {
	//	lro->user_irq[i].lro = lro;
	//	spin_lock_init(&lro->user_irq[i].events_lock);
	//	init_waitqueue_head(&lro->user_irq[i].events_wq);
	//}

	return lro;
}


static void SetDMAAddress(struct hws_pcie_dev *pdx)
{
	//-------------------------------------

	u32 Addrmsk;
	u32 AddrLowmsk;
	//u32 AddrPageSize;
	//u32 Addr2PageSize;
	u32 PhyAddr_A_Low;
	u32 PhyAddr_A_High;

	//u32 PhyAddr_A_Low2;
	//u32 PhyAddr_A_High2;
	//u32 PCI_Addr2;

	u32 PCI_Addr;
	//u32 AVALON_Addr;
	u32 cnt;
	//u64 m_tmp64cnt = 0;
	//u32 RDAvalon = 0;
	//u32 m_AddreeSpace = 0;
	int i = 0;
	u32 m_ReadTmp;
	u32 m_ReadTmp2;
	//u32 m_ReadTmp3;
	//u32 m_ReadTmp4;
	DWORD halfframeLength=0;
	//DWORD m_Valude;
	PhyAddr_A_High = 0;
	PhyAddr_A_Low=0;
	PCI_Addr =0;


	//------------------------------------------ // re write dma register

	Addrmsk = PCI_E_BAR_ADD_MASK;
	AddrLowmsk = PCI_E_BAR_ADD_LOWMASK;

	//printk("[MV]1DispatchCreate :Addrmsk = %X  AddrPageSize =%X Addr2PageSize =%X \n", Addrmsk, AddrPageSize, Addr2PageSize);

	cnt = 0x208;  // Table address
	for (i = 0; i< pdx->m_nMaxChl; i++)
	{
		//printk("[MV] pdx->m_pbyVideoBuffer[%d]=%x\n", i, pdx->m_pbyVideoBuffer[i]);
		if (pdx->m_pbyVideoBuffer[i])
		{
			PhyAddr_A_Low = pdx->m_dwVideoBuffer[i];
			PhyAddr_A_High = pdx->m_dwVideoHighBuffer[i];

			PCI_Addr = (PhyAddr_A_Low&AddrLowmsk);
			PhyAddr_A_Low = (PhyAddr_A_Low&Addrmsk);


			//printk("[MV]1-pdx->m_dwVideoBuffer[%d]-%X\n",i,pdx->m_dwVideoBuffer[i]);
			//-------------------------------------------------------------------------------
			WRITE_REGISTER_ULONG(pdx,(PCI_ADDR_TABLE_BASE + cnt),PhyAddr_A_High);
			WRITE_REGISTER_ULONG(pdx,(PCI_ADDR_TABLE_BASE + cnt+PCIE_BARADDROFSIZE),PhyAddr_A_Low);  //Entry 0
			//----------------------------------------
			m_ReadTmp =  READ_REGISTER_ULONG(pdx,(PCI_ADDR_TABLE_BASE + cnt));
			m_ReadTmp2 = READ_REGISTER_ULONG(pdx,(PCI_ADDR_TABLE_BASE + cnt+PCIE_BARADDROFSIZE));
			//printk("[MV]1-PCI_Addr[%d] :PhyAddr_A_Low  %X=%X  PhyAddr_A_High %X=%X\n", i, PhyAddr_A_Low, m_ReadTmp2, PhyAddr_A_High, m_ReadTmp);


			//--------------------------
			WRITE_REGISTER_ULONG(pdx,( CBVS_IN_BUF_BASE + (i*PCIE_BARADDROFSIZE)), ((i+1)*PCIEBAR_AXI_BASE)+PCI_Addr); //Buffer 1 address
			halfframeLength = pdx->m_format[i].HLAF_SIZE/16;
			WRITE_REGISTER_ULONG(pdx,( CBVS_IN_BUF_BASE2 + (i*PCIE_BARADDROFSIZE)),halfframeLength); //Buffer 1 address


			m_ReadTmp =  READ_REGISTER_ULONG(pdx,(  CBVS_IN_BUF_BASE + (i*PCIE_BARADDROFSIZE)));
			m_ReadTmp2 = READ_REGISTER_ULONG(pdx,(  CBVS_IN_BUF_BASE2 + (i*PCIE_BARADDROFSIZE)));
			//printk("[MV]1-Avalone [X64]BUF[%d]:BUF1=%X  BUF2=%X\n", i,  m_ReadTmp,  m_ReadTmp2);

			//---------------------------

		}
		cnt +=8;
		#if 1
		if(pdx->m_pbyAudioBuffer[i])
		{
			PhyAddr_A_Low = pdx->m_dwAudioBuffer[i];
			PhyAddr_A_High = pdx->m_dwAudioBufferHigh[i];
			PCI_Addr = (PhyAddr_A_Low&AddrLowmsk);
			PhyAddr_A_Low = (PhyAddr_A_Low&Addrmsk);
			//printk("[X1]Audio:PCI_Addr =%X\n",PCI_Addr);
			//printk("[X1]Audio:-------- - LOW=%X  HIGH =%X\n",pdx->m_dwAudioBuffer[i],pdx->m_dwAudioBufferHigh[i]);
			WRITE_REGISTER_ULONG(pdx,(CBVS_IN_BUF_BASE + ((8+i)*PCIE_BARADDROFSIZE)), ((i+1)*PCIEBAR_AXI_BASE+PCI_Addr)); //Buffer 1 address
			m_ReadTmp = READ_REGISTER_ULONG(pdx,(CBVS_IN_BUF_BASE + ((8+i)*PCIE_BARADDROFSIZE)));
			//printk("[X1]Audio:[%d] :--------BUF1: %X=%X\n",i,(PCIEBAR_AXI_BASE+PCI_Addr),m_ReadTmp);
		}
		#endif
	}
	hws_enable_card_irqs(pdx); //enable PCI Interruput
	//WRITE_REGISTER_ULONG(PCIEBR_EN_REG_BASE, 0xFFFFFFFF);

}

//-----------------------------------
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
	spin_unlock_irqrestore(&pdx->videoslock[ch], flags);
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
	//int check_res=0;
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
			printk(KERN_WARNING "HwsCapture: card stopped responding\n");
		}
		/* gone: "no signal"; a glitch so far: keep what we had */
		return READ_ONCE(pdx->m_PciDeviceLost) ? 1 : READ_ONCE(pdx->m_curr_No_Video[ch]);
	}
	pdx->m_lost_reads = 0;
	if (READ_ONCE(pdx->m_PciDeviceLost) && !READ_ONCE(pdx->suspended))
	{
		/* it is back (link retrained); CheckVideFmt re-arms the streaming channels */
		WRITE_ONCE(pdx->m_PciDeviceLost, 0);
		pdx->m_recovered = 1;
		printk(KERN_INFO "HwsCapture: card responds again\n");
	}
	active_video = ((value&0xFF)>>ch)&0x01;
	interlace = value>>8;
	interlace = ((interlace&0xFF)>>ch)&0x01;
	//printk("[MV][%d] active_video %d\n",ch,active_video);
	if(active_video >0)
	{
		if(pdx->m_DeviceHW_Version>0)
		{
			offset = 110 + ch;
			frame_rate = READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)));
			//DbgPrint("[MV][%d] Frame= %d\n",ch,frame_rate);
			if(frame_rate !=pdx->m_pVCAPStatus[ch][0].dwFrameRate )
			{
				pdx->m_pVCAPStatus[ch][0].dwFrameRate = frame_rate;
			}
			//-------------------
			offset = 120 + ch;
			//DbgPrint("[MV][%d] active_video %d\n",ch,interlace);
			value =  READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)));
			out_res_w = value&0xFFFF;
			out_res_h = (value>>16)&0xFFFF;
			if((out_res_w!=pdx->m_pVCAPStatus[ch][0].dwOutWidth)||(out_res_h!= pdx->m_pVCAPStatus[ch][0].dwOutHeight))
			{
				out_value = pdx->m_pVCAPStatus[ch][0].dwOutHeight;
				out_value =(out_value<<16)|pdx->m_pVCAPStatus[ch][0].dwOutWidth;
				WRITE_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)), out_value);
				//DbgPrint("[MV][%d] out_res_w=%d out_res_h=%d\n",ch,out_res_w,out_res_h);
				//DbgPrint("[MV][%d] out_w=%d out_h=%d \n",ch,m_pVCAPStatus[ch][0].dwOutWidth,m_pVCAPStatus[ch][0].dwOutHeight );
			}
			offset = 130 + ch;
			out_frame_rate = READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)));
			if(out_frame_rate !=pdx->m_pVCAPStatus[ch][0].dwOutFrameRate )
			{
				WRITE_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)), pdx->m_pVCAPStatus[ch][0].dwOutFrameRate);
				//DbgPrint("[MV][%d]  out_rate=%d ?=%d \n",ch,out_frame_rate,m_pVCAPStatus[ch][0].dwOutFrameRate );

			}
			//---------------------------------
			//-----------------------------------------
			offset = 91 + ch*2;
			//DbgPrint("[MV][%d] active_video %d\n",ch,interlace);
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
				//DbgPrint("[MV][%d] value=%X ?=%X\n",ch,value,out_value);
			}
			value =  READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (8*PCIE_BARADDROFSIZE)));
			video_hdcp = ((value&0xFF)>>ch)&0x01;
			pdx->m_pVCAPStatus[ch][0].dwhdcp = video_hdcp;
			//DbgPrint("[MV][%d]support hdcp %d\n",ch,video_hdcp);
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
		//------------------------------------------------
		offset = 90 + ch*2;
		//DbgPrint("[MV][%d] active_video %d\n",ch,interlace);
		value =  READ_REGISTER_ULONG(pdx,(DWORD)(CVBS_IN_BASE + (offset*PCIE_BARADDROFSIZE)));
		res_w = value&0xFFFF;
		res_h = (value>>16)&0xFFFF;
		/* 0 x 0 shows up while the source switches modes; it would divide by zero later */
		if ((res_w > 0) && (res_h > 0) &&
		    (((res_w <=MAX_VIDEO_HW_W) &&(res_h<=MAX_VIDEO_HW_H)&&(interlace==0))||((res_w <=MAX_VIDEO_HW_W) &&(res_h*2<=MAX_VIDEO_HW_H)&&(interlace==1))))
		{
			if((res_w !=pdx->m_pVCAPStatus[ch][0].dwWidth )||(res_h!= pdx->m_pVCAPStatus[ch][0].dwHeight)||(pdx->m_pVCAPStatus[ch][0].dwinterlace!=interlace))
			{
				ChangeVideoSize(pdx,ch,res_w,res_h,interlace);

			}
		}

		no_video = 0;
		//printk("[MV-X1]-[ch-%d]W=%d H=%d interlace =%d %dx%d \n",ch,res_w,res_h,interlace,pdx->m_pVCAPStatus[ch][0].dwWidth, pdx->m_pVCAPStatus[ch][0].dwHeight);
	}
	else
	{
		no_video = 1;
	}

	return no_video;

}

static void CheckVideFmt (struct hws_pcie_dev *pdx)
{
	//PAGED_CODE();
	int i;
	//DWORD value;
	//DWORD SetData;
	//	int ret=0;
	//int nNeed_ReInit =0;
	//	unsigned char mark=1;


	for(i =0; i<pdx->m_nCurreMaxVideoChl;i++)
	{
		#if 0
		value =  ReadDevReg((DWORD)(CVBS_IN_BASE + ((91+i*2)*PCIE_BARADDROFSIZE)));
		m_brightness[i] = value&0xFF;
		m_contrast[i] =  (value>>8)&0xFF;
		m_hue[i] = (value>>16)&0xFF;
		m_saturation[i] = (value>>24)&0xFF;
		//DbgPrint("[MV]value[%d]= %X\n",i,value);
		if((g_contrast[i] != m_contrast[i])||(g_brightness[i] != m_brightness[i])||(g_saturation[i] != m_saturation[i])||(g_hue[i] != m_hue[i]))
		{

		//DbgPrint("[MV]m_brightness[%d]= %d %d \n",i,m_brightness[i],g_brightness[i]);
		//DbgPrint("[MV]m_contrast[%d]= %d %d\n",i,m_contrast[i],g_contrast[i]);
		//DbgPrint("[MV]m_hue[%d]= %d %d \n",i,m_hue[i],g_hue[i]);
		//DbgPrint("[MV]m_saturation[%d]= %d %d \n",i,m_saturation[i],g_saturation[i] );
		SetData = g_saturation[i]<<24;
		SetData  |=g_hue[i]<<16;
		SetData  |=g_contrast[i]<<8;
		SetData  |=g_brightness[i];
		//DbgPrint("[MV]value[%d]= %X %X\n",i,value,SetData);
		WriteDevReg((DWORD)(CVBS_IN_BASE + ((91+i*2)*PCIE_BARADDROFSIZE)), SetData);

	}
	#endif
	{
		int was = READ_ONCE(pdx->m_curr_No_Video[i]);
		int now = Get_Video_Status(pdx,i);

		WRITE_ONCE(pdx->m_curr_No_Video[i], now);
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
	}
	//---------------
	if((pdx->m_curr_No_Video[i] ==0x1)&&(pdx->m_bVCapStarted[i]==TRUE))
	{
		//printk("[MV]check NoVideo End: [%d]\n",i);
		kthread_queue_work(pdx->conv_worker[i], &pdx->video[i].videowork);
	}
	//-----------------

	}

}

int MainKsThreadHandle(void *arg)
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
		/* also when a card that was gone at resume is back (config-space check
		 * only until then): CheckVideFmt notices it and re-inits it */
		if(need_check==1 ||
		   (READ_ONCE(pdx->need_bringup) && pci_device_is_present(pdx->pdev)))
		{
			CheckVideFmt(pdx);
			if(timing_debug && tick % 4 == 0)
				hws_tstat_report(pdx);	/* per-second summary */
		}
		/* 4 Hz: a mode change is noticed within 250 ms (cheap: a few register
		 * reads per channel). Returns early when kthread_stop() wakes us */
		schedule_timeout_interruptible(HZ / 4);
	}
	//printk("MainKsThreadHandle Exit");
	return 0;
}
static void StartKSThread(struct hws_pcie_dev *pdx)
{
	pdx->mMain_tsk = kthread_run(MainKsThreadHandle,(void*)pdx,"hws-status");
	if (IS_ERR(pdx->mMain_tsk))
	{
		printk(KERN_ERR "HwsCapture: cannot start status thread\n");
		pdx->mMain_tsk = NULL;
	}

}



//------------------------------


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
	//printk("msi_msix_capable in \n");
	if (!dev || dev->no_msi)
	{
		printk("msi_msix_capable no_msi exit \n");
		return 0;
	}

	for (bus = dev->bus; bus; bus = bus->parent)
	{
		if (bus->bus_flags & PCI_BUS_FLAGS_NO_MSI)
		{
			printk("msi_msix_capable PCI_BUS_FLAGS_NO_MSI \n");
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
		printk("msi_msix_capable pci_find_capability =%d\n",ret);
		return 0;
	}

	return 1;
}


static int probe_scan_for_msi(struct hws_pcie_dev *lro, struct pci_dev *pdev)
{
	//int i;
	int rc = 0;
	//int req_nvec = MAX_NUM_ENGINES + MAX_USER_IRQ;

	//BUG_ON(!lro);
	//BUG_ON(!pdev);
	//if (msi_msix_capable(pdev, PCI_CAP_ID_MSIX)) {
	//		printk("Enabling MSI-X\n");
	//		for (i = 0; i < req_nvec; i++)
	//			lro->entry[i].entry = i;
	//
	//		rc = pci_enable_msix(pdev, lro->entry, req_nvec);
	//		if (rc < 0)
	//			printk("Couldn't enable MSI-X mode: rc = %d\n", rc);

	//		lro->msix_enabled = 1;
	//		lro->msi_enabled = 0;
	//	}
	//else

	if (msi_msix_capable(pdev, PCI_CAP_ID_MSI)) {
		/* enable message signalled interrupts */
		//printk("pci_enable_msi()\n");
		rc = pci_enable_msi(pdev);
		if (rc < 0)
		{
			/* not fatal: fall back to the legacy interrupt line */
			printk("Couldn't enable MSI mode: rc = %d, using INTx\n", rc);
			lro->msi_enabled = 0;
			rc = 0;
		}
		else
		{
			lro->msi_enabled = 1;
		}
		lro->msix_enabled = 0;
	} else {
		//printk("MSI/MSI-X not detected - using legacy interrupts\n");
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
	//void *reg;
	//u32 w;

	//BUG_ON(!lro);

	//if (lro->msix_enabled) {
	//	rc = msix_irq_setup(lro);
	//}
	//else
	{
		if (!lro->msi_enabled){
			pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &val);
			//printk("Legacy Interrupt register value = %d\n", val);
		}
		//irq_flag = lro->msi_enabled ? 0 : IRQF_SHARED;
		/* a legacy INTx line may be shared with other devices; an MSI vector is ours */
		irq_flag = lro->msi_enabled ? 0 : IRQF_SHARED;
		//irq_flag = IRQF_SHARED;

		/*
		 * No IRQF_ONESHOT unless a sharer of the line insists: a masked line
		 * would hold back the next half-frame interrupt, and with it the
		 * overrun detection, until the IRQ thread is done. The hard handler
		 * acknowledges the card, and the storm guard covers a stuck line.
		 */
		rc = request_threaded_irq(pdev->irq, irqhandler, hws_irq_thread, irq_flag, pci_name(pdev), lro);
		if (rc == -EBUSY && (irq_flag & IRQF_SHARED))
			rc = request_threaded_irq(pdev->irq, irqhandler, hws_irq_thread,
						  irq_flag | IRQF_ONESHOT, pci_name(pdev), lro);
		if (rc)
		{
			//printk("Couldn't use IRQ#%d, rc=%d\n", pdev->irq, rc);
		}
		else
		{
			lro->irq_line = (int)pdev->irq;
			//printk("Using IRQ#%d with  MSI_EN=%d \n", pdev->irq,lro->msi_enabled);
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
static void __devinit enable_pcie_relaxed_ordering(struct pci_dev *dev)
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
//--------------------------------------
static void InitVideoSys(struct hws_pcie_dev *pdx,int set)
{
	// init decoder
	int i,j;
	//	DWORD dwRest=0;
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
			//pdx->m_nVideoIndex[i] =0;
			pdx->m_nAudioBufferIndex[i] =0;
			EnableVideoCapture(pdx,i,0);
			EnableAudioCapture(pdx,i,0);

		}
	}

	hws_enable_card_irqs(pdx);
	//start Run
	//---------------------------------------------
	WRITE_REGISTER_ULONG(pdx,CVBS_IN_BASE, 0x80000000);
	//DelayUs(500);
	m_Valude= 0x00FFFFFF;
	m_Valude |= 0x80000000;
	WRITE_REGISTER_ULONG(pdx,CVBS_IN_BASE, m_Valude);
	WRITE_REGISTER_ULONG(pdx,(CVBS_IN_BASE + (0 * PCIE_BARADDROFSIZE)), 0X13);
	pdx->m_bStartRun = 1;
	//--------------------------------------------------

}

//-------------------------------------
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
	//-----------------------
	if(pdx->m_Device_Version>121)
	{
		if((pdx->dwDeviceID==0x8501)&&(pdx->m_Device_Version==122))
		{
			pdx->m_DeviceHW_Version =0;
		}
		else
		{
			//DWORD m_ReadTmp;
			pdx->m_DeviceHW_Version =1;
			//--- set DMA_MAX_SIZE
			WRITE_REGISTER_ULONG(pdx,(CVBS_IN_BASE + (9 * PCIE_BARADDROFSIZE)), (MAX_VIDEO_SCLAER_SIZE/16));
			//m_ReadTmp = ReadDevReg((DWORD)(CVBS_IN_BASE + (9 * PCIE_BARADDROFSIZE)));
			//DbgPrint("[MV]DMA_MAX_SIZE =%d\n",m_ReadTmp);
		}
	}
	else
	{
		pdx->m_DeviceHW_Version =0;
	}
	//------------------


}
static int ReadChipId(struct hws_pcie_dev *pdx)
{
	//  CCIR_PACKET      reg;
	//int Chip_id1 = 0;
	int ret=0;
	//int reg_vaule = 0;
	//int nResult;
	int i;
	//------read Dvice Version
	ULONG m_dev_ver;
	ULONG m_tmpVersion;
	ULONG m_tmpHWKey;
	//ULONG m_OEM_code_data;
	m_dev_ver= READ_REGISTER_ULONG(pdx,CVBS_IN_BASE+(88*PCIE_BARADDROFSIZE));
	if (m_dev_ver == 0xFFFFFFFF)
	{
		printk(KERN_ERR "HwsCapture: card does not respond (reads all ones)\n");
		return -ENODEV;
	}

	m_tmpVersion = m_dev_ver>>8;
	pdx->m_Device_Version =  (m_tmpVersion&0xFF);
	m_tmpVersion = m_dev_ver>>16;
	pdx->m_Device_SubVersion = (m_tmpVersion&0xFF);
	pdx->m_Device_SupportYV12 = ((m_dev_ver>>28)&0x0F);
	if (pdx->m_Device_SupportYV12 > 2)
	{
		printk(KERN_WARNING "HwsCapture: unknown pixel layout %u, treating as YUYV\n", pdx->m_Device_SupportYV12);
		pdx->m_Device_SupportYV12 = 0;
	}
	m_tmpHWKey =  m_dev_ver>>24;
	m_tmpHWKey = m_tmpHWKey&0x0F;
	pdx->m_Device_PortID = m_tmpHWKey&0x03;
	//n_VideoModle =	READ_REGISTER_ULONG(pdx,0x4000+(4*PCIE_BARADDROFSIZE));
	//n_VideoModle = (n_VideoModle>>8)&0xFF;
	//pdx->m_IsHDModel = 1;
	pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
	pdx->m_nMaxChl  = 4;
	pdx->m_bBufferAllocate = FALSE;
	pdx->mMain_tsk = NULL;
	pdx->m_dwAudioPTKSize = MAX_DMA_AUDIO_PK_SIZE; //128*16*4;
	pdx->m_bStartRun = 0;
	pdx->m_PciDeviceLost =0;

	//--------
	for( i=0; i<MAX_VID_CHANNELS; i++)
	{
		SetVideoFormteSize(pdx,i,1920,1080);
	}
	//-------
	WRITE_REGISTER_ULONG(pdx,CVBS_IN_BASE,0x0);
	//ssleep(100);
	WRITE_REGISTER_ULONG(pdx,CVBS_IN_BASE,0x10);
	//ssleep(500);
	//-------
	SetHardWareInfo(pdx);
	printk("************[HW]-[VIDV]=[%d]-[%d]-[%d] ************\n",  pdx->m_Device_Version, pdx->m_Device_SubVersion,pdx->m_Device_PortID );
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
	printk("MV360: Device =%X VID =%X\n",gdev->dwDeviceID,gdev->dwVendorID);
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

	gdev->info.mem[0].internal_addr = ioremap(pci_resource_start(pdev, 0),
											  pci_resource_len(pdev, 0));
	gdev->map_bar0_addr = (u32 *)gdev->info.mem[0].internal_addr;
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
	//---------------
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
		//----------------------
		gdev->m_contrast[i] =0x80 ;
		gdev->m_brightness[i] = 0x80;
		gdev->m_saturation[i] = 0x80;
		gdev->m_hue[i] = 0x80;
		atomic_set(&gdev->m_dwSWFrameRate[i], 0);
		gdev->m_pbyVideoBuffer[i] = NULL;
		gdev->m_VideoInfo[i].dwisRuning= 0;
		gdev->m_VideoInfo[i].m_nVideoIndex= 0;
		gdev->m_VideoInfo[i].m_pVideoScalerBuf = NULL;
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
			//------------------
			gdev->m_VideoInfo[i].m_pVideoBufData[j] = NULL;
			gdev->m_VideoInfo[i].m_pVideoBufData1[j] = NULL;
			gdev->m_VideoInfo[i].m_pVideoBufData2[j] = NULL;
			gdev->m_VideoInfo[i].m_pVideoBufData3[j] = NULL;
			gdev->m_VideoInfo[i].pStatusInfo[j].byLock= MEM_UNLOCK;
			//----------------
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
		INIT_WORK(&gdev->audio[i].audiowork,audio_data_process);
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
		pdx->suspended = false;
		mutex_unlock(&pdx->pm_lock);
		dev_warn(dev, "card not responding after resume\n");
		return 0;
	}
	hws_hw_bringup(pdx);
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
