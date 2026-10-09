/*
 * Concurrency test of the driver's capture core, compiled from the driver's
 * own functions (extracted verbatim into core.inc) on top of shim.h.
 *
 *  hw thread      - simulated card: writes stamped half-frames into the DMA
 *                   buffer and raises the interrupt (runs the real irqhandler)
 *  irq thread     - runs the real hws_irq_thread when woken
 *  work threads   - one per work item (videowork, nosig_work), shim.c-style
 *  status thread  - like MainKsThreadHandle: signal loss, resolution changes
 *  consumers      - per channel, several apps with different buffer counts and
 *                   processing speeds that randomly stop/restart streaming
 *
 * Every delivered buffer is checked: one uniform frame (no tearing / mixing),
 * strictly increasing sequence and timestamp, no duplicate frame, and after
 * stop_streaming returns every buffer is back and none completes later.
 */
#include "shim.h"
#include "hws.h"
#include "hws_reg.h"

atomic_long sim_ratelimited_warnings;
static int timing_debug = 2;	/* exercise the diagnostic paths too */
static int direct_io = 1;

/* ------------------------------------------------------------ fake MMIO */
#define NREG (0x6000 / 4)
static _Atomic u32 regs[NREG];
static void WRITE_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 off, u32 v)
{
	(void)pdx;
	if (off == CVBS_IN_BASE + 1 * PCIE_BARADDROFSIZE)	/* IRQ status: write 1 to clear */
		atomic_fetch_and(&regs[off / 4], ~v);
	else
		atomic_store(&regs[off / 4], v);
}
static u32 READ_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 off)
{
	(void)pdx;
	if (off == CVBS_IN_BASE)
		return 0x04 | 0x01;	/* "interrupt pending" + "running" */
	return atomic_load(&regs[off / 4]);
}
static void InitVideoSys(struct hws_pcie_dev *pdx, int set)
{ (void)pdx; (void)set; fprintf(stderr, "unexpected InitVideoSys\n"); abort(); }

/* Diagnostic mode (SIM_DMA_LOCK=1): the card never overwrites DMA memory the
 * driver is copying out of, which removes the physical DMA race and isolates
 * driver logic. memcpy from the DMA buffer takes the channel's read lock. */
/* per-channel event trace, dumped when a torn frame is found (SIM_TORN_DEBUG) */
#define EV_N 32768
struct ev { u64 t; char type; int half; int v1; int v2; };
static struct ev evlog[MAX_VID_CHANNELS][EV_N];
static _Atomic unsigned evpos[MAX_VID_CHANNELS];
static void ev(int ch, char type, int half, int v1, int v2)
{
	unsigned i = atomic_fetch_add(&evpos[ch], 1) % EV_N;
	evlog[ch][i] = (struct ev){ ktime_get_ns(), type, half, v1, v2 };
}
static void ev_dump(int ch, u64 around)
{
	unsigned end = atomic_load(&evpos[ch]);
	u64 slot = NSEC_PER_SEC / 480 / 2;
	for (unsigned i = end > EV_N ? end - EV_N : 0; i < end; i++) {
		struct ev *e = &evlog[ch][i % EV_N];
		if (e->t + 8 * slot < around || e->t > around + 3 * slot)
			continue;
		fprintf(stderr, "   %+8.3fms %c half%d %d %d\n", ((s64)e->t - (s64)around) / 1e6, e->type, e->half, e->v1, e->v2);
	}
}
static pthread_rwlock_t dma_lock[MAX_VID_CHANNELS];
void hws_trace(int ch, char what, int half, long a, long b) { ev(ch, what, half, (int)a, (int)b); }
static int dma_lock_mode;
static struct hws_pcie_dev *pdx;
static void *sim_memcpy(void *d, const void *s, size_t n)
{
	int ch;
	for (ch = 0; ch < MAX_VID_CHANNELS; ch++) {
		const u8 *b = pdx->m_pbyVideoBuffer[ch];
		if (b && (const u8 *)s >= b && (const u8 *)s < b + MAX_MM_VIDEO_SIZE && n > 64) {
			int h = (const u8 *)s == b ? 1 : 0;
			ev(ch, 'c', h, ((const u8 *)s)[0], (int)READ_ONCE(pdx->m_half_cnt[ch][h]));
			if (dma_lock_mode) pthread_rwlock_rdlock(&dma_lock[ch]);
			memcpy(d, s, n);
			if (dma_lock_mode) pthread_rwlock_unlock(&dma_lock[ch]);
			ev(ch, 'C', h, ((const u8 *)d)[0], ((const u8 *)d)[n - 1]);
			return d;
		}
	}
	return memcpy(d, s, n);
}
#define memcpy sim_memcpy
#include "core.inc"
#undef memcpy

/* ------------------------------------------------------------ shim impl */
static void *work_thread(void *arg)
{
	struct work_struct *w = arg;
	pthread_mutex_lock(&w->lock);
	for (;;) {
		while (!w->pending && !w->stop)
			pthread_cond_wait(&w->cond, &w->lock);
		if (w->stop)
			break;
		w->pending = false;
		w->running = true;
		pthread_mutex_unlock(&w->lock);
		w->func(w);
		pthread_mutex_lock(&w->lock);
		w->running = false;
		pthread_cond_broadcast(&w->cond);
	}
	pthread_mutex_unlock(&w->lock);
	return NULL;
}
void INIT_WORK(struct work_struct *w, void (*f)(struct work_struct *))
{
	memset(w, 0, sizeof(*w));
	w->func = f;
	pthread_mutex_init(&w->lock, NULL);
	pthread_cond_init(&w->cond, NULL);
	pthread_create(&w->thr, NULL, work_thread, w);
}
bool queue_work(struct workqueue_struct *wq, struct work_struct *w)
{
	bool ret;
	(void)wq;
	pthread_mutex_lock(&w->lock);
	ret = !w->pending;
	w->pending = true;
	pthread_cond_broadcast(&w->cond);
	pthread_mutex_unlock(&w->lock);
	return ret;
}
bool flush_work(struct work_struct *w)
{
	pthread_mutex_lock(&w->lock);
	while (w->pending || w->running)
		pthread_cond_wait(&w->cond, &w->lock);
	pthread_mutex_unlock(&w->lock);
	return true;
}
bool cancel_work_sync(struct work_struct *w)
{
	pthread_mutex_lock(&w->lock);
	w->pending = false;
	while (w->running)
		pthread_cond_wait(&w->cond, &w->lock);
	pthread_mutex_unlock(&w->lock);
	return true;
}
static void *dwork_thread(void *arg)
{
	struct delayed_work *d = arg;
	pthread_mutex_lock(&d->lock);
	for (;;) {
		while (!d->expires)
			pthread_cond_wait(&d->cond, &d->lock);
		u64 now = ktime_get_ns();
		if (now >= d->expires) {
			d->expires = 0;
			pthread_mutex_unlock(&d->lock);
			queue_work(NULL, &d->work);
			pthread_mutex_lock(&d->lock);
			continue;
		}
		struct timespec ts = { (time_t)(d->expires / NSEC_PER_SEC), (long)(d->expires % NSEC_PER_SEC) };
		pthread_cond_timedwait(&d->cond, &d->lock, &ts);	/* CLOCK_MONOTONIC cond below */
	}
	return NULL;
}
void INIT_DELAYED_WORK(struct delayed_work *d, void (*f)(struct work_struct *))
{
	pthread_condattr_t a;
	INIT_WORK(&d->work, f);
	pthread_mutex_init(&d->lock, NULL);
	pthread_condattr_init(&a);
	pthread_condattr_setclock(&a, CLOCK_MONOTONIC);
	pthread_cond_init(&d->cond, &a);
	d->expires = 0;
	pthread_create(&d->thr, NULL, dwork_thread, d);
}
bool mod_delayed_work(struct workqueue_struct *wq, struct delayed_work *d, unsigned long ms)
{
	(void)wq;
	pthread_mutex_lock(&d->lock);
	d->expires = ktime_get_ns() + (u64)ms * NSEC_PER_MSEC;
	pthread_cond_broadcast(&d->cond);
	pthread_mutex_unlock(&d->lock);
	return true;
}
bool cancel_delayed_work_sync(struct delayed_work *d)
{
	pthread_mutex_lock(&d->lock);
	d->expires = 0;
	pthread_mutex_unlock(&d->lock);
	return cancel_work_sync(&d->work);
}
/* driver delivery latency: frame-complete IRQ -> buffer done, in microseconds */
#define DLAT_MAX 20000
static atomic_long dlat_us[DLAT_MAX + 1];
void vb2_buffer_done(struct vb2_buffer *vb, enum vb2_buffer_state state)
{
	struct vb2_queue *q = vb->vb2_queue;
	if (state == VB2_BUF_STATE_DONE && vb->timestamp) {
		u64 us = (ktime_get_ns() - vb->timestamp) / 1000;
		atomic_fetch_add(&dlat_us[us > DLAT_MAX ? DLAT_MAX : us], 1);
	}
	pthread_mutex_lock(&q->lock);
	if (q->stopped)
		q->done_after_stop++;
	vb->state = state;
	q->owned_by_driver--;
	list_add_tail(&vb->done_entry, &q->done);
	pthread_cond_broadcast(&q->cond);
	pthread_mutex_unlock(&q->lock);
}

/* ------------------------------------------------------------ test setup */
#define NCH 2		/* channels captured in parallel */
#define OUT_W 1280
#define OUT_H 720
static struct workqueue_struct dummy_wq;
static atomic_bool run = true;
static int fps = 60;
static int seconds = 30;

/* source state per channel, driven by the status thread */
static _Atomic int src_w[NCH], src_h[NCH], src_present[NCH];
static _Atomic u64 last_src_change[NCH];

/* results */
static atomic_long frames_hw, delivered, torn_transition, torn_steady, dups, seq_err, ts_err,
	err_bufs, stop_leaks, done_after_stop, nosig_frames, stalls, restarts;
static atomic_long lat_hist[8];	/* <2,<4,<8,<16,<33,<66,<133,>=133 ms */
static _Atomic u64 lat_max_ns;

static void sleep_ns(u64 ns)
{
	struct timespec t = { (time_t)(ns / NSEC_PER_SEC), (long)(ns % NSEC_PER_SEC) };
	nanosleep(&t, NULL);
}

/* ---- IRQ thread: kernel semantics, a wake while running runs it once more ---- */
static pthread_mutex_t irq_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t irq_cond = PTHREAD_COND_INITIALIZER;
static bool irq_runthread;
static void *irq_thread(void *a)
{
	(void)a;
	while (atomic_load(&run)) {
		pthread_mutex_lock(&irq_lock);
		while (!irq_runthread && atomic_load(&run))
			pthread_cond_wait(&irq_cond, &irq_lock);
		irq_runthread = false;
		pthread_mutex_unlock(&irq_lock);
		hws_irq_thread(0, pdx);
	}
	return NULL;
}
static void raise_irq(int ch, int half)
{
	static pthread_mutex_t hard = PTHREAD_MUTEX_INITIALIZER;	/* hard IRQs don't nest */
	pthread_mutex_lock(&hard);
	atomic_store(&regs[(CVBS_IN_BASE + (32 + ch) * PCIE_BARADDROFSIZE) / 4], half);
	atomic_fetch_or(&regs[(CVBS_IN_BASE + 1 * PCIE_BARADDROFSIZE) / 4], 1u << ch);
	if (irqhandler(0, pdx) == IRQ_WAKE_THREAD) {
		pthread_mutex_lock(&irq_lock);
		irq_runthread = true;
		pthread_cond_signal(&irq_cond);
		pthread_mutex_unlock(&irq_lock);
	}
	pthread_mutex_unlock(&hard);
}

/* exact frame identity: (IRQ timestamp of the completing half) -> hw frame number */
#define FID_N 8192
static struct { _Atomic u64 ts; _Atomic long f; } fid[NCH][FID_N];
static _Atomic long fid_pos[NCH];
static long frame_of_ts(int ch, u64 ts)
{
	for (int i = 0; i < FID_N; i++)
		if (atomic_load(&fid[ch][i].ts) == ts)
			return atomic_load(&fid[ch][i].f);
	return -1;	/* clamped timestamp or no-signal frame */
}
static atomic_long true_dups, order_err;

/* ---- simulated card ---- */
static int lose_irq, dup_irq;
static atomic_long dup_irqs;
static unsigned lose_seed = 99;
static atomic_long lost_irqs;
static void sleep_until(u64 t)
{
	u64 now = ktime_get_ns();
	if (t > now)
		sleep_ns(t - now);
}

/*
 * Fixed-rate card, like real hardware: a frame period is two half slots on an
 * absolute schedule. A half's data lands in its slot and its interrupt fires
 * at the end of the slot, so a half is rewritten no earlier than half a
 * period after its interrupt. A late card skips whole frames (keeping the
 * half order) instead of catching up in a burst.
 */
static void *hw_thread(void *a)
{
	int ch = (int)(intptr_t)a;
	u64 slot = NSEC_PER_SEC / fps / 2, t0 = ktime_get_ns(), k = 0;
	u32 f = 0;

	while (atomic_load(&run)) {
		u32 en = atomic_load(&regs[(CVBS_IN_BASE + 2 * PCIE_BARADDROFSIZE) / 4]);
		u64 s0 = t0 + k * slot;	/* first-half slot of this frame */
		u64 now = ktime_get_ns();

		if (now > s0 + slot / 4) {
			/* late: skip to the next frame boundary, never burst */
			k = ((now - t0) / slot + 2) & ~1ULL;
			continue;
		}
		k += 2;
		sleep_until(s0);
		if ((en & (1u << ch)) && atomic_load(&src_present[ch])) {
			int w = atomic_load(&src_w[ch]), h = atomic_load(&src_h[ch]);
			int total = w * h * 2;
			/* split point as programmed by the driver (ChangeVideoSize/SetDMAAddress) */
			int half = atomic_load(&regs[(CBVS_IN_BUF_BASE2 + ch * PCIE_BARADDROFSIZE) / 4]) * 16;
			u8 v = 120 + f % 100;
			int lose;

			if (half <= 0 || half >= total)
				half = total / 2;
			f++;
			atomic_fetch_add(&frames_hw, 1);
			/* first half: data during its slot, interrupt at the end */
			if (dma_lock_mode) pthread_rwlock_wrlock(&dma_lock[ch]);
			memset(pdx->m_pbyVideoBuffer[ch], v, half);
			if (dma_lock_mode) pthread_rwlock_unlock(&dma_lock[ch]);
			ev(ch, 'W', 1, v, 0);
			sleep_until(s0 + slot);
			raise_irq(ch, 1);
			ev(ch, 'I', 1, v, (int)pdx->m_half_cnt[ch][1]);
			/* second half */
			if (dma_lock_mode) pthread_rwlock_wrlock(&dma_lock[ch]);
			memset(pdx->m_pbyVideoBuffer[ch] + half, v, total - half);
			if (dma_lock_mode) pthread_rwlock_unlock(&dma_lock[ch]);
			ev(ch, 'W', 0, v, 0);
			/* SIM_DUP_IRQ=N: one in N first halves is reported again shortly
			 * after, without new data (spurious duplicate interrupt) */
			if (dup_irq && (rand_r(&lose_seed) % dup_irq) == 0) {
				sleep_until(s0 + slot + slot / 4);
				atomic_fetch_add(&dup_irqs, 1);
				raise_irq(ch, 1);
				ev(ch, 'D', 1, v, (int)pdx->m_half_cnt[ch][1]);
			}
			sleep_until(s0 + 2 * slot);
			/* SIM_LOSE_IRQ=N: lose one second-half interrupt in N (the card's next
			 * interrupt then reports the same half twice in a row) */
			lose = lose_irq && (rand_r(&lose_seed) % lose_irq) == 0;
			if (lose) {
				atomic_fetch_add(&lost_irqs, 1);
				ev(ch, 'L', 0, v, 0);
			} else {
				raise_irq(ch, 0);
				ev(ch, 'I', 0, v, (int)pdx->m_half_cnt[ch][0]);
				long kk = atomic_fetch_add(&fid_pos[ch], 1) % FID_N;
				atomic_store(&fid[ch][kk].f, (long)f);
				atomic_store(&fid[ch][kk].ts, (u64)atomic64_read(&pdx->m_half_ts[ch][0]));
			}
		}
	}
	return NULL;
}

/* ---- status thread: like MainKsThreadHandle/CheckVideFmt ---- */
static void *status_thread(void *a)
{
	(void)a;
	unsigned seed = 7;
	int tick = 0;
	while (atomic_load(&run)) {
		sleep_ns(NSEC_PER_SEC / 4);
		tick++;
		for (int ch = 0; ch < NCH; ch++) {
			/* source events: loss of signal or a mode change every few seconds */
			if (tick % 12 == 3 + ch * 4) {
				int r = rand_r(&seed) % 3;
				if (r == 0) {
					atomic_store(&src_present[ch], !atomic_load(&src_present[ch]));
				} else {
					int big = atomic_load(&src_w[ch]) == 1280;
					atomic_store(&src_w[ch], big ? 1920 : 1280);
					atomic_store(&src_h[ch], big ? 1080 : 720);
				}
				atomic_store(&last_src_change[ch], ktime_get_ns());
			}
			/* the driver notices once per second */
			if (tick % 4 == 0) {
				int present = atomic_load(&src_present[ch]);
				int w = atomic_load(&src_w[ch]), h = atomic_load(&src_h[ch]);
				if (present && (pdx->m_pVCAPStatus[ch][0].dwWidth != (DWORD)w ||
						pdx->m_pVCAPStatus[ch][0].dwHeight != (DWORD)h))
					ChangeVideoSize(pdx, ch, w, h, 0);
				WRITE_ONCE(pdx->m_curr_No_Video[ch], !present);
				if (!present && pdx->m_bVCapStarted[ch] == TRUE)
					queue_work(pdx->wq, &pdx->video[ch].videowork);
			}
		}
	}
	return NULL;
}

/* ---- consumers ---- */
struct consumer {
	int ch, nbufs, proc_ms, stop_every;
	struct hws_vfh_ctx ctx;
	struct hwsvideo_buffer *bufs;
};

static int is_nosig(const u8 *p) { return p[0] == 41 && p[1] == 240 && p[2] == 41 && p[3] == 109; }

static void qbuf(struct consumer *c, struct hwsvideo_buffer *b)
{
	pthread_mutex_lock(&c->ctx.vbq.lock);
	c->ctx.vbq.owned_by_driver++;
	pthread_mutex_unlock(&c->ctx.vbq.lock);
	hws_buffer_queue_multi(&b->vb.vb2_buf);
}

static void *consumer_thread(void *a)
{
	struct consumer *c = a;
	struct vb2_queue *q = &c->ctx.vbq;
	struct hws_video *vd = &pdx->video[c->ch];
	unsigned seed = 1000 + c->nbufs * 13 + c->proc_ms;
	size_t out_size = OUT_W * OUT_H * 2;
	unsigned long flags;
	int i, n = 0;

	/* like hws_open */
	INIT_LIST_HEAD(&c->ctx.buf_queue);
	spin_lock_init(&c->ctx.qlock);
	spin_lock_init(&c->ctx.dlock);
	c->ctx.video = vd;
	q->drv_priv = &c->ctx;
	pthread_mutex_init(&q->lock, NULL);
	pthread_cond_init(&q->cond, NULL);
	INIT_LIST_HEAD(&q->done);
	spin_lock_irqsave(&vd->consumers_lock, flags);
	list_add_tail(&c->ctx.node, &vd->consumers);
	spin_unlock_irqrestore(&vd->consumers_lock, flags);
	c->bufs = calloc(c->nbufs, sizeof(*c->bufs));
	for (i = 0; i < c->nbufs; i++) {
		c->bufs[i].vb.vb2_buf.vb2_queue = q;
		c->bufs[i].vb.vb2_buf.plane_size = out_size;
		c->bufs[i].mem = malloc(out_size);
	}

	while (atomic_load(&run)) {
		u32 last_seq = 0; u64 last_ts = 0; int last_v = -1, have_last = 0; long last_f = -1;

		/* STREAMON */
		q->stopped = false;
		for (i = 0; i < c->nbufs; i++)
			qbuf(c, &c->bufs[i]);
		hws_start_streaming_multi(q, c->nbufs);

		for (int k = 0; atomic_load(&run) && (c->stop_every == 0 || k < c->stop_every); k++) {
			struct vb2_buffer *vb;
			struct hwsvideo_buffer *b;
			struct timespec dl;

			pthread_mutex_lock(&q->lock);
			clock_gettime(CLOCK_REALTIME, &dl);
			dl.tv_sec += 2;
			while (list_empty(&q->done) && atomic_load(&run))
				if (pthread_cond_timedwait(&q->cond, &q->lock, &dl)) {
					if (!atomic_load(&run))
						break;		/* test is shutting down, the card stopped */
					atomic_fetch_add(&stalls, 1);	/* no frame for 2 s */
					fprintf(stderr, "STALL consumer ch%d nbufs=%d stop_every=%d k=%d: src_present=%d no_video=%d vcap_started=%d engine_users=%d streaming=%d owned=%d since_src_change=%.2fs\n",
						c->ch, c->nbufs, c->stop_every, k, src_present[c->ch], pdx->m_curr_No_Video[c->ch],
						pdx->m_bVCapStarted[c->ch], vd->engine_users, c->ctx.streaming, q->owned_by_driver,
						(ktime_get_ns() - last_src_change[c->ch]) / 1e9);
					break;
				}
			if (list_empty(&q->done)) { pthread_mutex_unlock(&q->lock); continue; }
			vb = list_entry(q->done.next, struct vb2_buffer, done_entry);
			list_del(&vb->done_entry);
			pthread_mutex_unlock(&q->lock);
			b = container_of(to_vb2_v4l2_buffer(vb), struct hwsvideo_buffer, vb);

			if (vb->state != VB2_BUF_STATE_DONE) {
				atomic_fetch_add(&err_bufs, 1);
			} else {
				const u8 *p = b->mem;
				u64 now = ktime_get_ns(), lat = now - vb->timestamp;
				int bucket = 0;
				u64 lm;

				atomic_fetch_add(&delivered, 1);
				if (have_last && (s32)(b->vb.sequence - last_seq) <= 0)
					atomic_fetch_add(&seq_err, 1);
				if (have_last && vb->timestamp <= last_ts)
					atomic_fetch_add(&ts_err, 1);
				if (is_nosig(p)) {
					atomic_fetch_add(&nosig_frames, 1);
					last_v = -1;
				} else {
					int v = p[0], torn = 0;
					for (size_t o = 0; o < out_size; o += 509)
						if (p[o] != v) { torn = 1; break; }
					if (p[out_size - 1] != v)
						torn = 1;
					if (torn) {
						/* expected only while the driver catches up with a source change */
						if (now - atomic_load(&last_src_change[c->ch]) < 1500 * NSEC_PER_MSEC)
							atomic_fetch_add(&torn_transition, 1);
						else {
							atomic_fetch_add(&torn_steady, 1);
							if (getenv("SIM_TORN_DEBUG")) {
								size_t o2, first_diff = 0, last_diff = 0; int v2 = -1;
								for (o2 = 0; o2 < out_size; o2++)
									if (p[o2] != v) { if (!first_diff) first_diff = o2; last_diff = o2; if (v2 < 0) v2 = p[o2]; }
								fprintf(stderr, "TORN ch%d diff %zu..%zu of %zu | since src change %.2fs (frame ts %.2fs after it) src %dx%d driver %ux%u | v=%d other=%d frame=%ld\n",
									c->ch, first_diff, last_diff, out_size,
									(now - atomic_load(&last_src_change[c->ch])) / 1e9,
									((s64)vb->timestamp - (s64)atomic_load(&last_src_change[c->ch])) / 1e9,
									src_w[c->ch], src_h[c->ch],
									pdx->m_pVCAPStatus[c->ch][0].dwWidth, pdx->m_pVCAPStatus[c->ch][0].dwHeight,
									v, v2, frame_of_ts(c->ch, vb->timestamp));
								ev_dump(c->ch, vb->timestamp);
							}
						}
					} else if (v == last_v && (b->vb.sequence - last_seq) < 100) {
						atomic_fetch_add(&dups, 1);	/* heuristic (content mod 100) */
					}
					{
						long fr = frame_of_ts(c->ch, vb->timestamp);
						if (fr >= 0 && last_f >= 0) {
							if (fr == last_f)
								atomic_fetch_add(&true_dups, 1);
							else if (fr < last_f)
								atomic_fetch_add(&order_err, 1);
						}
						if (fr >= 0)
							last_f = fr;
					}
					last_v = torn ? -1 : v;
					while (bucket < 7 && lat >= (2ULL << bucket) * NSEC_PER_MSEC)
						bucket++;
					atomic_fetch_add(&lat_hist[bucket], 1);
					lm = atomic_load(&lat_max_ns);
					while (lat > lm && !atomic_compare_exchange_weak(&lat_max_ns, &lm, lat))
						;
				}
				last_seq = b->vb.sequence; last_ts = vb->timestamp; have_last = 1;
			}
			if (c->proc_ms)
				sleep_ns((u64)(rand_r(&seed) % (2 * c->proc_ms + 1)) * NSEC_PER_MSEC);
			qbuf(c, b);
			n++;
		}

		/* STREAMOFF: every buffer must be back when stop_streaming returns */
		hws_stop_streaming_multi(q);
		pthread_mutex_lock(&q->lock);
		if (q->owned_by_driver != 0)
			atomic_fetch_add(&stop_leaks, q->owned_by_driver);
		q->stopped = true;
		INIT_LIST_HEAD(&q->done);
		q->owned_by_driver = 0;
		pthread_mutex_unlock(&q->lock);
		atomic_fetch_add(&restarts, 1);
		sleep_ns((u64)(rand_r(&seed) % 200) * NSEC_PER_MSEC);	/* app restarts later */
		pthread_mutex_lock(&q->lock);
		atomic_fetch_add(&done_after_stop, q->done_after_stop);
		q->done_after_stop = 0;
		pthread_mutex_unlock(&q->lock);
	}
	return NULL;
}

int main(int argc, char **argv)
{
	pthread_t hw[NCH], irq, st;
	/* per channel: a smooth app (6 bufs), a slow app (2 bufs, 25 ms), a restarting app */
	struct consumer cons[] = {
		{ .ch = 0, .nbufs = 6, .proc_ms = 2,  .stop_every = 0 },
		{ .ch = 0, .nbufs = 2, .proc_ms = 25, .stop_every = 0 },
		{ .ch = 0, .nbufs = 4, .proc_ms = 5,  .stop_every = 40 },
		{ .ch = 1, .nbufs = 4, .proc_ms = 3,  .stop_every = 0 },
		{ .ch = 1, .nbufs = 3, .proc_ms = 8,  .stop_every = 25 },
	};
	int ncons = sizeof(cons) / sizeof(cons[0]);
	pthread_t ct[16];
	int ch, i, j;

	if (argc > 1) fps = atoi(argv[1]);
	if (argc > 2) seconds = atoi(argv[2]);
	dma_lock_mode = getenv("SIM_DMA_LOCK") != NULL;
	if (getenv("SIM_LOSE_IRQ")) lose_irq = atoi(getenv("SIM_LOSE_IRQ"));
	if (getenv("SIM_DUP_IRQ")) dup_irq = atoi(getenv("SIM_DUP_IRQ"));
	for (ch = 0; ch < MAX_VID_CHANNELS; ch++)
		pthread_rwlock_init(&dma_lock[ch], NULL);
	printf("dma race %s\n", dma_lock_mode ? "REMOVED (logic-only mode)" : "present (like real hardware)");

	/* like hws_probe */
	pdx = calloc(1, sizeof(*pdx));
	pdx->m_bStartRun = 1;
	pdx->m_DeviceHW_Version = 1;
	pdx->m_Device_SupportYV12 = 0;
	pdx->m_MaxHWVideoBufferSize = MAX_MM_VIDEO_SIZE;
	pdx->m_nMaxChl = 4;
	pdx->m_nCurreMaxVideoChl = NCH;
	pdx->wq = &dummy_wq;
	spin_lock_init(&pdx->reg_lock);
	mutex_init(&pdx->pm_lock);
	if (getenv("SIM_NO_DIRECT")) direct_io = 0;
	printf("direct path %s\n", direct_io ? "on" : "off");
	for (ch = 0; ch < MAX_VID_CHANNELS; ch++) {
		spin_lock_init(&pdx->videoslock[ch]);
		spin_lock_init(&pdx->audiolock[ch]);
		pdx->video[ch].index = ch;
		pdx->video[ch].dev = pdx;
		mutex_init(&pdx->video[ch].engine_lock);
		spin_lock_init(&pdx->video[ch].consumers_lock);
		INIT_LIST_HEAD(&pdx->video[ch].consumers);
		INIT_WORK(&pdx->video[ch].videowork, video_data_process);
		INIT_DELAYED_WORK(&pdx->video[ch].nosig_work, hws_nosig_work);
		mutex_init(&pdx->direct_lock[ch]);
		INIT_LIST_HEAD(&pdx->direct_bufs[ch]);
		INIT_WORK(&pdx->video[ch].fanout_work, hws_fanout_work);
		spin_lock_init(&pdx->video[ch].fanout_lock);
		INIT_LIST_HEAD(&pdx->video[ch].fanout_q);
		INIT_LIST_HEAD(&pdx->video[ch].fanout_free);
		for (j = 0; j < HWS_FANOUT_POOL; j++) {
			INIT_LIST_HEAD(&pdx->video[ch].fanout_pool[j].bufs);
			list_add_tail(&pdx->video[ch].fanout_pool[j].node, &pdx->video[ch].fanout_free);
		}
		pdx->video[ch].current_out_width = OUT_W;
		pdx->video[ch].curren_out_height = OUT_H;
		for (j = 0; j < MAX_VIDEO_QUEUE; j++) {
			pdx->m_pVCAPStatus[ch][j].dwWidth = 1280;
			pdx->m_pVCAPStatus[ch][j].dwHeight = 720;
			pdx->m_pVCAPStatus[ch][j].dwFrameRate = fps;
		}
		SetVideoFormteSize(pdx, ch, 1280, 720);
		atomic_store(&regs[(CBVS_IN_BUF_BASE2 + ch * PCIE_BARADDROFSIZE) / 4], pdx->m_format[ch].HLAF_SIZE / 16);
		if (ch < NCH) {
			pdx->m_pbyVideoBuffer[ch] = calloc(1, MAX_MM_VIDEO_SIZE);
			pdx->m_VideoInfo[ch].m_pVideoScalerBuf = malloc(MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2);
			pdx->m_VideoInfo[ch].m_pVideoYUV2Buf = malloc(MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2);
			pdx->m_VideoInfo[ch].m_pRotateVideoBuf = malloc(MAX_VIDEO_HW_W * MAX_VIDEO_HW_H * 2);
			for (j = 0; j < MAX_VIDEO_QUEUE; j++)
				pdx->m_VideoInfo[ch].m_pVideoBufData[j] = calloc(1, MAX_MM_VIDEO_SIZE);
			atomic_store(&src_w[ch], 1280);
			atomic_store(&src_h[ch], 720);
			atomic_store(&src_present[ch], 1);
		}
	}

	pthread_create(&irq, NULL, irq_thread, NULL);
	for (ch = 0; ch < NCH; ch++)
		pthread_create(&hw[ch], NULL, hw_thread, (void *)(intptr_t)ch);
	pthread_create(&st, NULL, status_thread, NULL);
	for (i = 0; i < ncons; i++)
		pthread_create(&ct[i], NULL, consumer_thread, &cons[i]);

	sleep(seconds);
	atomic_store(&run, false);
	for (i = 0; i < ncons; i++)
		pthread_join(ct[i], NULL);
	for (ch = 0; ch < NCH; ch++)
		pthread_join(hw[ch], NULL);
	pthread_mutex_lock(&irq_lock); irq_runthread = true; pthread_cond_signal(&irq_cond); pthread_mutex_unlock(&irq_lock);
	pthread_join(irq, NULL);
	pthread_join(st, NULL);

	printf("fps=%d seconds=%d channels=%d consumers=%d\n", fps, seconds, NCH, ncons);
	printf("frames from hw           %ld  (lost second-half IRQs %ld, duplicate first-half IRQs %ld)\n", frames_hw, lost_irqs, dup_irqs);
	printf("buffers delivered (DONE) %ld  (no-signal frames %ld)\n", delivered, nosig_frames);
	printf("stream restarts          %ld\n", restarts);
	printf("torn/mixed, steady state %ld   <- must be 0\n", torn_steady);
	printf("torn, source transition  %ld   (driver not yet aware of new mode)\n", torn_transition);
	printf("duplicate frames         %ld   <- must be 0\n", true_dups);
	printf("frames out of order      %ld   <- must be 0 (exact frame identity)\n", order_err);
	printf("same content mod 100     %ld   (heuristic; legit when >=100 hw frames were dropped)\n", dups);
	printf("sequence not increasing  %ld   <- must be 0\n", seq_err);
	printf("timestamp not increasing %ld   <- must be 0\n", ts_err);
	printf("buffers left in driver after STREAMOFF  %ld <- must be 0\n", stop_leaks);
	printf("buffers completed after STREAMOFF       %ld <- must be 0\n", done_after_stop);
	printf("ERROR buffers            %ld\n", err_bufs);
	printf("2 s stalls               %ld\n", stalls);
	printf("IRQ->app latency ms: <2:%ld <4:%ld <8:%ld <16:%ld <33:%ld <66:%ld <133:%ld >=133:%ld  max %.1f\n",
	       lat_hist[0], lat_hist[1], lat_hist[2], lat_hist[3], lat_hist[4], lat_hist[5], lat_hist[6], lat_hist[7],
	       lat_max_ns / 1e6);
	{
		long tot = 0, acc = 0; int p50 = -1, p90 = -1, p99 = -1, mx = 0;
		for (i = 0; i <= DLAT_MAX; i++) { tot += dlat_us[i]; if (dlat_us[i]) mx = i; }
		for (i = 0; i <= DLAT_MAX; i++) {
			acc += dlat_us[i];
			if (p50 < 0 && acc * 2 >= tot) p50 = i;
			if (p90 < 0 && acc * 10 >= tot * 9) p90 = i;
			if (p99 < 0 && acc * 100 >= tot * 99) p99 = i;
		}
		printf("driver delivery latency (IRQ -> buffer done) us: p50 %d p90 %d p99 %d max %d%s (n=%ld)\n",
		       p50, p90, p99, mx, mx == DLAT_MAX ? "+" : "", tot);
	}
	for (ch = 0; ch < NCH; ch++) {
		struct hws_timing_stats *ts = &pdx->tstats[ch];
		printf("ch%d driver stats: direct %d dma_overrun %d overwrite %d skip %d stale_half %d no_app_buf %d empty %d coalesced %d\n", ch,
		       atomic_read(&ts->direct), atomic_read(&ts->dma_overrun), atomic_read(&ts->ring_overwrite), atomic_read(&ts->ring_skip), atomic_read(&ts->stale_half),
		       atomic_read(&ts->no_app_buf), atomic_read(&ts->worker_empty), atomic_read(&ts->wq_coalesced));
	}
	return (torn_steady || true_dups || order_err || seq_err || ts_err || stop_leaks || done_after_stop) ? 1 : 0;
}
