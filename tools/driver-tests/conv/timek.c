#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>
typedef unsigned char BYTE;
int kconvert(int yv12, BYTE *ring, BYTE *dst, BYTE *sc, BYTE *rot, int w, int ih, int nCopy, int il, int ow, int oh);
static double t(int yv12, int w, int h, int il, int ow, int oh)
{
	BYTE *ring = malloc(0x400000), *dst = malloc(ow * oh * 2), *sc = malloc(1920*1080*2), *rot = malloc(1920*1080*2);
	int ih = il ? h * 2 : h, nCopy = yv12 == 1 ? w*12*h/8 : yv12 == 2 ? w*5*h/4 : w*h*2, i, n = 100;
	struct timespec a, b;
	memset(ring, 0x5a, 0x400000); memset(dst, 0, ow*oh*2); memset(sc, 0, 1920*1080*2);
	kconvert(yv12, ring, dst, sc, rot, w, ih, nCopy, il, ow, oh);
	double best = 1e9;
	for (int r = 0; r < 40; r++) {
		clock_gettime(CLOCK_MONOTONIC, &a);
		for (i = 0; i < 5; i++) kconvert(yv12, ring, dst, sc, rot, w, ih, nCopy, il, ow, oh);
		clock_gettime(CLOCK_MONOTONIC, &b);
		double ms = ((b.tv_sec - a.tv_sec) * 1e3 + (b.tv_nsec - a.tv_nsec) / 1e6) / 5;
		if (ms < best) best = ms;
	}
	(void)n;
	free(ring); free(dst); free(sc); free(rot);
	return best;
}
int main(void)
{
	printf("%-28s", getenv("LABEL") ? getenv("LABEL") : "");
	printf("min ms/frame: YUYV1080p=%.2f YUYV1080i=%.2f 720p->1080p=%.2f 1080p->720p=%.2f YV12-1080p=%.2f NV12-1080p=%.2f\n",
	       t(0,1920,1080,0,1920,1080), t(0,1920,540,1,1920,1080), t(0,1280,720,0,1920,1080), t(0,1920,1080,0,1280,720),
	       t(1,1920,1080,0,1920,1080), t(2,1920,1080,0,1920,1080));
	return 0;
}
