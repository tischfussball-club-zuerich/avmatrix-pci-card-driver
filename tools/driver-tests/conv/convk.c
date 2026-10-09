/* the driver's converters, compiled with x86 kernel code-generation flags */
#include <string.h>
#include <stdint.h>
typedef unsigned long long u64; typedef unsigned long long __le64;
#define cpu_to_le64(x) ((__le64)(x))
#define le64_to_cpu(x) ((u64)(x))
typedef unsigned int DWORD; typedef unsigned char BYTE; typedef unsigned char BOOL;
#define min(a, b) ((a) < (b) ? (a) : (b))
typedef struct { BYTE *m_pVideoScalerBuf, *m_pVideoYUV2Buf, *m_pRotateVideoBuf; } ACAP_VIDEO_INFO;
struct hws_pcie_dev { int m_Device_SupportYV12; int m_MaxHWVideoBufferSize; ACAP_VIDEO_INFO m_VideoInfo[4]; };
#include "driver_conv.inc"
int kconvert(int yv12, BYTE *ring, BYTE *dst, BYTE *sc, BYTE *rot, int w, int ih, int nCopy, int il, int ow, int oh)
{
	struct hws_pcie_dev pdx = { .m_Device_SupportYV12 = yv12, .m_MaxHWVideoBufferSize = 0x400000 };
	pdx.m_VideoInfo[0].m_pVideoScalerBuf = sc; pdx.m_VideoInfo[0].m_pRotateVideoBuf = rot;
	return hws_convert_frame(&pdx, 0, ring, dst, w, ih, w * ih * 2, nCopy, il, ow, oh, ow * oh * 2, 0);
}
