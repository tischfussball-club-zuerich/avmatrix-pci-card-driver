/* The direct path's line doubling (hws_copy_doubled, two halves split anywhere)
 * must give exactly what the worker's SetDeInterlace() gives. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef unsigned char BYTE;
#define min(a,b) ((a)<(b)?(a):(b))
#include "doubling.inc"
int main(void)
{
	static const int ws[] = {1920, 1280, 720, 640, 2}, hs[] = {540, 360, 288, 240, 1};
	long n = 0, bad = 0;
	for (int wi = 0; wi < 5; wi++) for (int hi = 0; hi < 5; hi++) {
		int w = ws[wi], h = hs[hi], field = w * h * 2, pitch = w * 2;
		BYTE *src = malloc(field), *a = malloc(field * 2), *b = malloc(field * 2);
		for (int i = 0; i < field; i++) src[i] = rand();
		SetDeInterlace(src, a, w, h * 2, 1);
		/* every split point the driver can produce (multiples of 2048) plus odd ones */
		for (int half = 1; half < field; half += (half < 5000 ? 1 : 2048)) {
			memset(b, 0xEE, field * 2);
			hws_copy_doubled(b, src, 0, half, pitch);
			hws_copy_doubled(b, src + half, half, field - half, pitch);
			n++;
			if (memcmp(a, b, field * 2)) { bad++; if (bad < 5) printf("MISMATCH %dx%d split %d\n", w, h, half); }
		}
		free(src); free(a); free(b);
	}
	printf("line-doubling splits tested: %ld, mismatches: %ld\n", n, bad);
	return bad != 0;
}
