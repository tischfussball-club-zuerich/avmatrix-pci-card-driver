/*
 * Userspace test of the driver's frame converters/scalers (code extracted
 * verbatim from src/hws_video.c into driver_conv.inc).
 *
 * Every buffer the converters touch is placed directly against a PROT_NONE
 * guard page - flush with the end in one pass (catches overruns) and with
 * the start in the other (catches underruns) - so any out-of-bounds access
 * faults immediately. Inputs follow what the driver accepts: Get_Video_Status
 * bounds plus the worker's checks in video_data_process().
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <setjmp.h>
#include <time.h>
#include <sys/mman.h>
#include <unistd.h>

typedef unsigned long long u64; typedef unsigned long long __le64;
typedef unsigned int u32; typedef unsigned int __le32;
typedef unsigned short u16; typedef unsigned char u8;
#define le32_to_cpu(x) ((u32)(x))
#define cpu_to_le64(x) ((__le64)(x))
#define le64_to_cpu(x) ((u64)(x))
typedef unsigned int DWORD;
typedef unsigned long long u64_t;
typedef unsigned char BYTE;
typedef unsigned char BOOL;
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
typedef long long s64;
#define div_s64(a, b) ((s64)(a) / (b))

#define MAX_VIDEO_HW_W 1920
#define MAX_VIDEO_HW_H 1080
#define MAX_VIDEO_QUEUE 4
#define MAX_MM_VIDEO_SIZE 0x400000

struct hws_scaler;
typedef struct {
	BYTE *m_pVideoScalerBuf;
	struct hws_scaler *m_pScaler;
	BYTE *m_pVideoYUV2Buf;
	BYTE *m_pRotateVideoBuf;
} ACAP_VIDEO_INFO;

struct hws_pcie_dev {
	int m_Device_SupportYV12;
	int m_MaxHWVideoBufferSize;
	ACAP_VIDEO_INFO m_VideoInfo[4];
};

#include "driver_conv.inc"

/* ---- guarded allocations ------------------------------------------------ */
static long pg;
struct gbuf { BYTE *base; size_t maplen; BYTE *p; size_t len; };

static void galloc(struct gbuf *g, size_t len, int at_end)
{
	size_t data = (len + pg - 1) / pg * pg;
	g->maplen = data + 2 * pg;
	g->base = mmap(NULL, g->maplen, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (g->base == MAP_FAILED) { perror("mmap"); exit(2); }
	mprotect(g->base, pg, PROT_NONE);
	mprotect(g->base + pg + data, pg, PROT_NONE);
	g->len = len;
	g->p = at_end ? g->base + pg + data - len : g->base + pg;
}
static void gfree(struct gbuf *g) { munmap(g->base, g->maplen); }

static sigjmp_buf jb;
static void onsegv(int sig) { (void)sig; siglongjmp(jb, 1); }

/* ---- same accept rules as the driver ------------------------------------ */
/* Get_Video_Status: res_w/res_h (field height when interlaced) */
static int hw_accepts(int w, int h, int il)
{
	if (w <= 0 || h <= 0 || w > MAX_VIDEO_HW_W)
		return 0;
	return il ? h * 2 <= MAX_VIDEO_HW_H : h <= MAX_VIDEO_HW_H;
}

static const int outs[][2] = {	/* support_videofmt[] */
	{1920,1080},{1280,720},{720,480},{720,576},{800,600},{1024,768},
	{1280,768},{1280,800},{1280,1024},{1360,768},{1440,900},{1680,1050},
	{640,480},{1080,1920},	/* added with the vendor 20260519 modes; 1080x1920 = rotated */
};
static const int ws[] = {1920,1918,1680,1600,1440,1366,1360,1280,1152,1080,1024,960,800,720,640,17,16,2};
static const int hs[] = {1080,1079,1050,1024,900,800,768,720,600,576,540,480,360,288,240,3,2,1};

static long n_run, n_skip, n_fail;
static int hash_mode;
static u64_t fnv(const BYTE *p, size_t n) { u64_t h = 1469598103934665603ULL; while (n--) { h ^= *p++; h *= 1099511628211ULL; } return h; }

/* one call exactly as video_data_process() makes it; returns 1 on a fault */
static int run_one(int yv12, int res_w, int res_h, int il, int ow, int oh, int at_end)
{
	struct hws_pcie_dev pdx = { .m_Device_SupportYV12 = yv12, .m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE };
	struct gbuf ring, sc, tab, y2, rot, dst;
	int in_width = res_w, in_height = res_h, in_vsize, nCopySize, out_size = ow * oh * 2;
	int needRotateVideo = 0, ok, fault;

	if (yv12 == 1)      nCopySize = (in_width * 12 * in_height) / 8;
	else if (yv12 == 2) nCopySize = (in_width * 5 * in_height) / 4;
	else                nCopySize = in_width * in_height * 2;
	if (il) in_height *= 2;
	in_vsize = in_width * in_height * 2;
	if ((in_width == 1280 && in_height == 720 && ow == 1080 && oh == 1920) ||
	    (in_width == 960 && in_height == 540 && ow == 1080 && oh == 1920))
		needRotateVideo = 1;

	/* the worker's ok check */
	ok = in_width > 0 && in_height > 0 &&
	     (yv12 == 0 || (in_width % 16) == 0) &&
	     in_width <= MAX_VIDEO_HW_W && in_height <= MAX_VIDEO_HW_H &&
	     nCopySize <= pdx.m_MaxHWVideoBufferSize;
	if (!ok) { n_skip++; return 0; }

	galloc(&ring, MAX_MM_VIDEO_SIZE, at_end);
	galloc(&sc, MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2, at_end);
	galloc(&tab, sizeof(struct hws_scaler), at_end);
	galloc(&y2, MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2, at_end);
	galloc(&rot, MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2, at_end);
	galloc(&dst, out_size, at_end);	/* exact size: stricter than vb2's page rounding */
	if (hash_mode) {
		unsigned x = 2463534242u; size_t k;
		for (k = 0; k < ring.len; k++) { x ^= x << 13; x ^= x >> 17; x ^= x << 5; ring.p[k] = x >> 24; }
		if (getenv("ZERO_TAIL")) memset(ring.p + nCopySize, 0, ring.len - nCopySize);
	} else
		memset(ring.p, 0x5a, ring.len);
	pdx.m_VideoInfo[0].m_pVideoScalerBuf = sc.p;
	pdx.m_VideoInfo[0].m_pScaler = getenv("NO_SCALER_TABLES") ? NULL : (struct hws_scaler *)tab.p;
	pdx.m_VideoInfo[0].m_pVideoYUV2Buf = y2.p;
	pdx.m_VideoInfo[0].m_pRotateVideoBuf = rot.p;

	fault = 0;
	/* same tail zeroing the worker does before converting */
	if (yv12 != 0 && nCopySize + 4 * in_width * 2 <= MAX_MM_VIDEO_SIZE)
		memset(ring.p + nCopySize, 0, 4 * in_width * 2);
	if (sigsetjmp(jb, 1) == 0)
		hws_convert_frame(&pdx, 0, ring.p, dst.p, in_width, in_height, in_vsize, nCopySize,
				  il, ow, oh, out_size, needRotateVideo);
	else
		fault = 1;
	if (!fault && hash_mode)
		printf("H yv12=%d in=%dx%d%s out=%dx%d %016llx\n", yv12, res_w, res_h, il ? "i" : "p", ow, oh, fnv(dst.p, out_size));
	if (!fault && sigsetjmp(jb, 1) == 0)
		SetNoVideoMem(dst.p, ow, oh);
	else if (!fault)
		fault = 2;
	if (fault) {
		n_fail++;
		printf("FAULT(%s, %s): yv12=%d in=%dx%d%s out=%dx%d\n",
		       fault == 1 ? "convert" : "novideo", at_end ? "overrun" : "underrun",
		       yv12, res_w, res_h, il ? "i" : "p", ow, oh);
	}
	gfree(&ring); gfree(&sc); gfree(&tab); gfree(&y2); gfree(&rot); gfree(&dst);
	n_run++;
	return fault != 0;
}

/* ---- timing of the common cases ----------------------------------------- */
static double time_case(int yv12, int w, int h, int il, int ow, int oh)
{
	struct hws_pcie_dev pdx = { .m_Device_SupportYV12 = yv12, .m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE };
	BYTE *ring = malloc(MAX_MM_VIDEO_SIZE), *dst = malloc(ow * oh * 2);
	int ih = il ? h * 2 : h, nCopy = yv12 == 1 ? w * 12 * h / 8 : yv12 == 2 ? w * 5 * h / 4 : w * h * 2;
	struct timespec a, b;
	int i, n = 60;

	pdx.m_VideoInfo[0].m_pVideoScalerBuf = malloc(1920 * 1080 * 2);
	pdx.m_VideoInfo[0].m_pScaler = getenv("NO_SCALER_TABLES") ? NULL : malloc(sizeof(struct hws_scaler));
	pdx.m_VideoInfo[0].m_pVideoYUV2Buf = malloc(1920 * 1080 * 2);
	pdx.m_VideoInfo[0].m_pRotateVideoBuf = malloc(1920 * 1080 * 2);
	memset(ring, 0x5a, MAX_MM_VIDEO_SIZE);
	memset(dst, 0, ow * oh * 2);
	hws_convert_frame(&pdx, 0, ring, dst, w, ih, w * ih * 2, nCopy, il, ow, oh, ow * oh * 2, 0);
	clock_gettime(CLOCK_MONOTONIC, &a);
	for (i = 0; i < n; i++)
		hws_convert_frame(&pdx, 0, ring, dst, w, ih, w * ih * 2, nCopy, il, ow, oh, ow * oh * 2, 0);
	clock_gettime(CLOCK_MONOTONIC, &b);
	free(ring); free(dst);
	free(pdx.m_VideoInfo[0].m_pVideoScalerBuf); free(pdx.m_VideoInfo[0].m_pVideoYUV2Buf);
	free(pdx.m_VideoInfo[0].m_pRotateVideoBuf); free(pdx.m_VideoInfo[0].m_pScaler);
	return ((b.tv_sec - a.tv_sec) * 1e3 + (b.tv_nsec - a.tv_nsec) / 1e6) / n;
}

int main(int argc, char **argv)
{
	struct sigaction sa = { .sa_handler = onsegv, .sa_flags = SA_NODEFER };
	int yv12, wi, hi, il, o, at_end;

	pg = sysconf(_SC_PAGESIZE);
	hash_mode = argc > 1 && strcmp(argv[1], "hash") == 0;
	sigaction(SIGSEGV, &sa, NULL);

	if (argc > 1 && strcmp(argv[1], "time") == 0) {
		printf("ms per frame (one consumer):\n");
		printf("  YUYV 1080p -> 1080p      %.2f\n", time_case(0, 1920, 1080, 0, 1920, 1080));
		printf("  YUYV 1080i -> 1080p      %.2f\n", time_case(0, 1920, 540, 1, 1920, 1080));
		printf("  YUYV 720p  -> 1080p      %.2f\n", time_case(0, 1280, 720, 0, 1920, 1080));
		printf("  YUYV 1080p -> 720p       %.2f\n", time_case(0, 1920, 1080, 0, 1280, 720));
		printf("  YV12 1080p -> 1080p      %.2f\n", time_case(1, 1920, 1080, 0, 1920, 1080));
		printf("  NV12 1080p -> 1080p      %.2f\n", time_case(2, 1920, 1080, 0, 1920, 1080));
		printf("  YUYV 1366x768 -> 1080p   %.2f\n", time_case(0, 1366, 768, 0, 1920, 1080));
		return 0;
	}

	for (at_end = 1; at_end >= (hash_mode ? 1 : 0); at_end--)
	for (yv12 = 0; yv12 <= 2; yv12++)
	for (il = 0; il <= 1; il++)
	for (wi = 0; wi < (int)(sizeof(ws) / sizeof(ws[0])); wi++)
	for (hi = 0; hi < (int)(sizeof(hs) / sizeof(hs[0])); hi++) {
		if (!hw_accepts(ws[wi], hs[hi], il))
			continue;
		for (o = 0; o < (int)(sizeof(outs) / sizeof(outs[0])); o++)
			run_one(yv12, ws[wi], hs[hi], il, outs[o][0], outs[o][1], at_end);
	}
	printf("combinations run: %ld, rejected by driver checks: %ld, faults: %ld\n", n_run, n_skip, n_fail);
	return n_fail ? 1 : 0;
}
