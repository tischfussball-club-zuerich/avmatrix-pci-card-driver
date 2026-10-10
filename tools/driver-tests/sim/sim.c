/*
 * Concurrency test of the driver's capture core, compiled from the driver's
 * own functions (extracted verbatim into core.inc) on top of shim.h.
 *
 *  hw thread      - simulated card: writes stamped half-frames into the DMA
 *                   buffer and raises the interrupt (runs the real irqhandler)
 *  irq thread     - runs the real hws_irq_thread when woken
 *  work threads   - per channel: a copy and a conversion kthread_worker; one per wq work item
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
static int nt_copy = 1;	/* the non-temporal variant of the direct path (plain memcpy here) */

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
static int fake_area_hit(const void *d);
/* SIM_HOSTILE: status registers return garbage (random, 0, all-ones, plausible) */
static int hostile;
/* hostile phases: 1 s of garbage, then 1 s of sane hardware the driver must recover in */
static atomic_int hostile_now;
static _Thread_local unsigned hseed = 12345;
static _Atomic u32 hval[0x6000 / 4];
static atomic_bool hset[0x6000 / 4];
/* a register keeps its (garbage) value and re-rolls now and then: stable
 * windows the pipeline can run in, interrupted by bursts of nonsense */
static u32 hostile_value(u32 off, u32 plausible)
{
	u32 i = off / 4;
	if (!atomic_load(&hset[i]) || rand_r(&hseed) % 100 < 3) {
		unsigned r = rand_r(&hseed) % 100;
		u32 v = r < 1 ? 0xFFFFFFFF : r < 3 ? 0 : r < 35 ? (((u32)rand_r(&hseed) << 16) ^ (u32)rand_r(&hseed)) : plausible;
		atomic_store(&hval[i], v);
		atomic_store(&hset[i], true);
	}
	return atomic_load(&hval[i]);
}
static int hostile_status_reg(u32 off)
{
	u32 i = (off - CVBS_IN_BASE) / PCIE_BARADDROFSIZE;
	if (off < CVBS_IN_BASE)
		return 0;
	return i == 0 || i == 5 || i == 8 || (i >= 88 && i <= 97) || (i >= 110 && i <= 113) ||
	       (i >= 120 && i <= 123) || (i >= 130 && i <= 133) || (i >= 32 && i <= 35) || (i >= 40 && i <= 43);
}
static u32 READ_REGISTER_ULONG(struct hws_pcie_dev *pdx, u32 off)
{
	(void)pdx;
	if (atomic_load(&hostile_now) && off != CVBS_IN_BASE + 1 * PCIE_BARADDROFSIZE && hostile_status_reg(off)) {
		u32 i = (off - CVBS_IN_BASE) / PCIE_BARADDROFSIZE;
		u32 plausible = i == 0 ? 0x05 : i == 5 ? 0x0303 : (i >= 90 && i <= 97 && !(i & 1)) ? (720u << 16 | 1280u) :
				(i >= 110 && i <= 113) ? 60 : atomic_load(&regs[off / 4]);
		return hostile_value(off, plausible);
	}
	if (off == CVBS_IN_BASE)
		return 0x04 | 0x01;	/* "interrupt pending" + "running" */
	return atomic_load(&regs[off / 4]);
}
static atomic_long periods_elapsed;
void snd_pcm_period_elapsed(struct snd_pcm_substream *sub) { (void)sub; atomic_fetch_add(&periods_elapsed, 1); }

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
static atomic_long cov_video_copy, cov_audio_copy, cov_audio_deliver;
static void *sim_memcpy(void *d, const void *s, size_t n)
{
	int ch;
	for (ch = 0; ch < MAX_VID_CHANNELS; ch++) {
		const u8 *b = pdx->m_pbyVideoBuffer[ch];
		if (b && (const u8 *)s >= b + MAX_MM_VIDEO_SIZE - MAX_AUDIO_CAP_SIZE && (const u8 *)s < b + MAX_MM_VIDEO_SIZE)
			atomic_fetch_add(&cov_audio_copy, 1);
		else if (b && (const u8 *)s >= b && (const u8 *)s < b + MAX_MM_VIDEO_SIZE && n > 64)
			atomic_fetch_add(&cov_video_copy, 1);
	}
	if (fake_area_hit(d))
		atomic_fetch_add(&cov_audio_deliver, 1);
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
/* resume/re-init is not simulated (needs the PCI core) */
static void hws_hw_bringup(struct hws_pcie_dev *p) { (void)p; }
static const u8 hws_silence[MAX_DMA_AUDIO_PK_SIZE];	/* a variable: not extracted with the functions */
/* V4L2 events and the default-format update need the V4L2 core: not simulated */
static void hws_notify_source_change(struct hws_video *v) { (void)v; }
static void hws_follow_input(struct hws_video *v, int w, int h) { (void)v; (void)w; (void)h; }
#include "core.inc"
#undef memcpy

/* ------------------------------------------------------------ shim impl */
static void *kworker_thread(void *arg)
{
	struct kthread_worker *wk = arg;
	pthread_mutex_lock(&wk->lock);
	for (;;) {
		struct kthread_work *w;
		while (!wk->head)
			pthread_cond_wait(&wk->cond, &wk->lock);
		w = wk->head;
		wk->head = w->next;
		if (!wk->head)
			wk->tail = NULL;
		w->pending = false;
		w->running = true;
		pthread_mutex_unlock(&wk->lock);
		w->func(w);
		pthread_mutex_lock(&wk->lock);
		w->running = false;
		pthread_cond_broadcast(&wk->cond);
	}
	return NULL;
}
struct kthread_worker *sim_kthread_worker(void)
{
	struct kthread_worker *wk = calloc(1, sizeof(*wk));
	pthread_mutex_init(&wk->lock, NULL);
	pthread_cond_init(&wk->cond, NULL);
	pthread_create(&wk->thr, NULL, kworker_thread, wk);
	return wk;
}
void kthread_init_work(struct kthread_work *w, void (*f)(struct kthread_work *))
{
	memset(w, 0, sizeof(*w));
	w->func = f;
}
bool kthread_queue_work(struct kthread_worker *wk, struct kthread_work *w)
{
	pthread_mutex_lock(&wk->lock);
	if (w->pending) {
		pthread_mutex_unlock(&wk->lock);
		return false;
	}
	w->worker = wk;
	w->pending = true;
	w->next = NULL;
	if (wk->tail)
		wk->tail->next = w;
	else
		wk->head = w;
	wk->tail = w;
	pthread_cond_broadcast(&wk->cond);
	pthread_mutex_unlock(&wk->lock);
	return true;
}
void kthread_flush_work(struct kthread_work *w)
{
	struct kthread_worker *wk = w->worker;
	if (!wk)
		return;
	pthread_mutex_lock(&wk->lock);
	while (w->pending || w->running)
		pthread_cond_wait(&wk->cond, &wk->lock);
	pthread_mutex_unlock(&wk->lock);
}
static void *kdwork_thread(void *arg)
{
	struct kthread_delayed_work *d = arg;
	pthread_mutex_lock(&d->lock);
	for (;;) {
		while (!d->expires)
			pthread_cond_wait(&d->cond, &d->lock);
		u64 now = ktime_get_ns();
		if (now >= d->expires) {
			struct kthread_worker *wk = d->worker;
			d->expires = 0;
			pthread_mutex_unlock(&d->lock);
			kthread_queue_work(wk, &d->work);
			pthread_mutex_lock(&d->lock);
			continue;
		}
		struct timespec ts = { (time_t)(d->expires / NSEC_PER_SEC), (long)(d->expires % NSEC_PER_SEC) };
		pthread_cond_timedwait(&d->cond, &d->lock, &ts);
	}
	return NULL;
}
void kthread_init_delayed_work(struct kthread_delayed_work *d, void (*f)(struct kthread_work *))
{
	pthread_condattr_t a;
	kthread_init_work(&d->work, f);
	pthread_mutex_init(&d->lock, NULL);
	pthread_condattr_init(&a);
	pthread_condattr_setclock(&a, CLOCK_MONOTONIC);
	pthread_cond_init(&d->cond, &a);
	d->expires = 0;
	d->worker = NULL;
	pthread_create(&d->thr, NULL, kdwork_thread, d);
}
bool kthread_mod_delayed_work(struct kthread_worker *wk, struct kthread_delayed_work *d, unsigned long ms)
{
	/* like the kernel: never waits (the work may be re-arming itself) */
	pthread_mutex_lock(&d->lock);
	d->worker = wk;
	d->expires = ktime_get_ns() + (u64)ms * NSEC_PER_MSEC;
	pthread_cond_broadcast(&d->cond);
	pthread_mutex_unlock(&d->lock);
	return true;
}
bool kthread_cancel_delayed_work_sync(struct kthread_delayed_work *d)
{
	pthread_mutex_lock(&d->lock);
	d->expires = 0;
	pthread_mutex_unlock(&d->lock);
	return kthread_cancel_work_sync(&d->work);
}
void kthread_flush_worker(struct kthread_worker *wk)
{
	pthread_mutex_lock(&wk->lock);
	while (wk->head)
		pthread_cond_wait(&wk->cond, &wk->lock);
	pthread_mutex_unlock(&wk->lock);
}
bool kthread_cancel_work_sync(struct kthread_work *w)
{
	struct kthread_worker *wk = w->worker;
	bool was;
	if (!wk)
		return false;
	pthread_mutex_lock(&wk->lock);
	was = w->pending;
	if (w->pending) {
		struct kthread_work **pp = &wk->head, *prev = NULL;
		while (*pp && *pp != w) { prev = *pp; pp = &(*pp)->next; }
		if (*pp) { *pp = w->next; if (wk->tail == w) wk->tail = prev; }
		w->pending = false;
	}
	while (w->running)
		pthread_cond_wait(&wk->cond, &wk->lock);
	pthread_mutex_unlock(&wk->lock);
	return was;
}
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
static int ntsc;	/* SIM_NTSC=1: fps * 1000/1001, like 59.94 */
static atomic_long rate_ok, rate_bad;	/* exact frame rate measurements */

/* source state per channel, driven by the status thread */
static _Atomic int src_w[NCH], src_h[NCH], src_il[NCH], src_present[NCH];	/* src_h: field height when interlaced */
static _Atomic u64 last_src_change[NCH];

/* results */
static atomic_long frames_hw_ch[NCH];
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
static pthread_mutex_t hard = PTHREAD_MUTEX_INITIALIZER;	/* hard IRQs don't nest, nor run twice at once */
static void raise_irq(int ch, int half)
{
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
	u64 slot = (ntsc ? NSEC_PER_SEC * 1001 / (fps * 1000ULL) : NSEC_PER_SEC / fps) / 2, t0 = ktime_get_ns(), k = 0;
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
			atomic_fetch_add(&frames_hw_ch[ch], 1);
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
/* ---- SIM_HOSTILE: interrupt storm, DMA garbage, fake ALSA stream ---- */
static void *storm_thread(void *a)
{
	unsigned seed = 4242;
	(void)a;
	while (atomic_load(&run)) {
		if (!atomic_load(&hostile_now)) { sleep_ns(NSEC_PER_MSEC); continue; }
		/* mostly valid video/audio bits, sometimes anything (incl. bits nobody acks) */
		u32 bits = rand_r(&seed) % 10 ? ((u32)rand_r(&seed) & 0x0F0F) :
			   (((u32)rand_r(&seed) ^ ((u32)rand_r(&seed) << 16)) & 0x00ffffff);
		/* (bits 24-31 don't exist on the card: nobody could ever clear them) */
		int i;

		for (i = 0; i < MAX_VID_CHANNELS; i++) {
			atomic_store(&regs[(CVBS_IN_BASE + (32 + i) * PCIE_BARADDROFSIZE) / 4], rand_r(&seed));
			atomic_store(&regs[(CVBS_IN_BASE + (40 + i) * PCIE_BARADDROFSIZE) / 4], rand_r(&seed));
		}
		pthread_mutex_lock(&hard);
		atomic_fetch_or(&regs[(CVBS_IN_BASE + 1 * PCIE_BARADDROFSIZE) / 4], bits);
		if (irqhandler(0, pdx) == IRQ_WAKE_THREAD) {
			pthread_mutex_lock(&irq_lock);
			irq_runthread = true;
			pthread_cond_signal(&irq_cond);
			pthread_mutex_unlock(&irq_lock);
		}
		pthread_mutex_unlock(&hard);
		sleep_ns((500 + rand_r(&seed) % 2500) * 1000ULL);
	}
	return NULL;
}

static void *garbage_thread(void *a)
{
	unsigned seed = 777;
	(void)a;
	while (atomic_load(&run)) {
		int ch = rand_r(&seed) % NCH;
		u8 *b = pdx->m_pbyVideoBuffer[ch];
		if (!atomic_load(&hostile_now)) { sleep_ns(NSEC_PER_MSEC); continue; }
		size_t off = (size_t)rand_r(&seed) % MAX_MM_VIDEO_SIZE;
		size_t len = rand_r(&seed) % 65536;
		if (off + len > MAX_MM_VIDEO_SIZE)
			len = MAX_MM_VIDEO_SIZE - off;
		if (rand_r(&seed) % 4 == 0) {	/* plant "not rewritten" marks */
			u32 m = 0x55AAAA55;
			off &= ~3UL;
			if (off + 4 <= MAX_MM_VIDEO_SIZE)
				memcpy(b + off, &m, 4);
		} else {
			memset(b + off, rand_r(&seed), len);
		}
		sleep_ns((100 + rand_r(&seed) % 900) * 1000ULL);
	}
	return NULL;
}

static struct snd_pcm_substream fake_sub[NCH];
static struct snd_pcm_runtime fake_rt[NCH];
static int fake_area_hit(const void *d)
{
	for (int c = 0; c < NCH; c++)
		if (fake_rt[c].dma_area && (const u8 *)d >= fake_rt[c].dma_area &&
		    (const u8 *)d < fake_rt[c].dma_area + fake_rt[c].buffer_size * 4)
			return 1;
	return 0;
}
static void *alsa_thread(void *a)
{
	unsigned seed = 31337;
	(void)a;
	while (atomic_load(&run)) {
		int ch = rand_r(&seed) % NCH;
		struct hws_audio *drv = &pdx->audio[ch];
		unsigned long flags;
		int r = rand_r(&seed) % 4;
		bool start = r != 0 && rand_r(&seed) % 3 != 0;

		/* the driver's trigger STOP / START, as ALSA calls them */
		if (start)
			StartAudioCapture(pdx, ch);
		else
			StopAudioCapture(pdx, ch);
		spin_lock_irqsave(&drv->ring_lock, flags);
		if (r == 0) {			/* close */
			drv->running = false;
			drv->substream = NULL;
		} else {			/* (re)open + hw_params + prepare + start */
			unsigned long frames = 2048 + rand_r(&seed) % (16384 - 2048 + 1);
			unsigned long period = 1024 + rand_r(&seed) % (frames / 2 - 1024 + 1);
			free(fake_rt[ch].dma_area);
			fake_rt[ch].dma_area = malloc(frames * 4);
			fake_rt[ch].buffer_size = frames;
			fake_rt[ch].period_size = period;
			fake_sub[ch].runtime = &fake_rt[ch];
			drv->substream = &fake_sub[ch];
			drv->channels = 2;
			drv->ring_size_byframes = frames;
			drv->period_size_byframes = period;
			drv->ring_wpos_byframes = 0;
			drv->period_used_byframes = 0;
			drv->running = start;
		}
		spin_unlock_irqrestore(&drv->ring_lock, flags);
		sleep_ns((5 + rand_r(&seed) % 50) * NSEC_PER_MSEC);
	}
	return NULL;
}

static void *status_thread(void *a)
{
	(void)a;
	unsigned seed = 7;
	int tick = 0;
	while (atomic_load(&run)) {
		if (hostile) {
			/* the driver's own status poll, against garbage registers, 50x faster */
			sleep_ns(20 * NSEC_PER_MSEC);
			if (++tick % 50 == 0) {
				if (atomic_load(&hostile_now))
					for (u32 k = 0; k < 0x6000 / 4; k++)	/* fresh garbage next time */
						atomic_store(&hset[k], false);
				atomic_store(&hostile_now, !atomic_load(&hostile_now));
			}
			CheckVideFmt(pdx);
			continue;
		}
		sleep_ns(NSEC_PER_SEC / 4);
		tick++;
		for (int ch = 0; ch < NCH; ch++) {
			/* source events: loss of signal or a mode change every few seconds */
			if (tick % 12 == 3 + ch * 4) {
				int r = rand_r(&seed) % 3;
				if (r == 0) {
					atomic_store(&src_present[ch], !atomic_load(&src_present[ch]));
				} else {
					/* 720p (direct), 1080p (scaled), 720i (direct, line-doubled) */
					static const int modes[3][3] = { {1280, 720, 0}, {1920, 1080, 0}, {1280, 360, 1} };
					int m = rand_r(&seed) % 3;
					atomic_store(&src_w[ch], modes[m][0]);
					atomic_store(&src_h[ch], modes[m][1]);
					atomic_store(&src_il[ch], modes[m][2]);
				}
				atomic_store(&last_src_change[ch], ktime_get_ns());
			}
			/* the driver notices once per second */
			if (tick % 4 == 0) {
				int present = atomic_load(&src_present[ch]);
				int w = atomic_load(&src_w[ch]), h = atomic_load(&src_h[ch]), il = atomic_load(&src_il[ch]);
				int was_present = !READ_ONCE(pdx->m_curr_No_Video[ch]);
				if (present && (pdx->m_pVCAPStatus[ch][0].dwWidth != (DWORD)w ||
						pdx->m_pVCAPStatus[ch][0].dwHeight != (DWORD)h ||
						pdx->m_pVCAPStatus[ch][0].dwinterlace != (DWORD)il)) {
					ChangeVideoSize(pdx, ch, w, h, il);
					hws_reset_rate(pdx, ch);
				}
				WRITE_ONCE(pdx->m_curr_No_Video[ch], !present);
				/* the exact rate, as CheckVideFmt measures it */
				if (present != was_present)
					hws_reset_rate(pdx, ch);
				else if (present) {
					u32 before = atomic64_read(&pdx->video[ch].tpf) >> 32;
					hws_measure_rate(pdx, ch);
					u64 tpf = atomic64_read(&pdx->video[ch].tpf);
					u32 num = tpf >> 32, den = (u32)tpf;
					if (num && (ntsc ? num == 1001 && den == fps * 1000u : (u64)den == (u64)num * fps))
						atomic_fetch_add(&rate_ok, 1);
					else if (num || before)
						atomic_fetch_add(&rate_bad, 1);	/* wrong, or a known rate lost */
				}
				if (!present && pdx->m_bVCapStarted[ch] == TRUE)
					kthread_queue_work(pdx->conv_worker[ch], &pdx->video[ch].videowork);
			}
		}
	}
	return NULL;
}

/* driver state of a channel, for a stall report */
static void stall_dump(int ch)
{
	struct hws_video *v = &pdx->video[ch];
	ACAP_VIDEO_INFO *vi = &pdx->m_VideoInfo[ch];
	struct hws_vfh_ctx *ctx;
	struct list_head *pos;
	int nd = 0, i;

	list_for_each(pos, &pdx->direct_bufs[ch]) nd++;
	fprintf(stderr, "  ch%d: direct_frame=%d direct_bufs=%d fanout_inflight=%d halfdone=%d pending=%lx cnt=%u/%u done_cnt=%u/%u gen=%u half1_gen=%u fmt_cnt0=%u half1_cnt=%u fmt=%ux%u%s out=%dx%d\n",
		ch, pdx->direct_frame[ch], nd, v->fanout_inflight, pdx->m_nVideoHalfDone[ch], pdx->m_half_pending[ch],
		pdx->m_half_cnt[ch][1], pdx->m_half_cnt[ch][0], pdx->m_half_done_cnt[ch][1], pdx->m_half_done_cnt[ch][0],
		pdx->m_fmt_gen[ch], pdx->m_half1_gen[ch], pdx->m_fmt_cnt0[ch], pdx->m_half1_cnt[ch],
		pdx->m_pVCAPStatus[ch][0].dwWidth, pdx->m_pVCAPStatus[ch][0].dwHeight,
		pdx->m_pVCAPStatus[ch][0].dwinterlace ? "i" : "p", v->current_out_width, v->curren_out_height);
	fprintf(stderr, "  ring:");
	for (i = 0; i < MAX_VIDEO_QUEUE; i++)
		fprintf(stderr, " %d", vi->pStatusInfo[i].byLock);
	fprintf(stderr, " | copy_work pend=%d run=%d videowork pend=%d run=%d fanout pend=%d run=%d\n",
		v->copy_work.pending, v->copy_work.running, v->videowork.pending, v->videowork.running,
		v->fanout_work.pending, v->fanout_work.running);
	list_for_each_entry(ctx, &v->consumers, node) {
		int nq = 0;
		list_for_each(pos, &ctx->buf_queue) nq++;
		fprintf(stderr, "  consumer streaming=%d queued=%d\n", ctx->streaming, nq);
	}
	{
		struct hws_timing_stats *ts = &pdx->tstats[ch];
		fprintf(stderr, "  stats: direct %d dma_overrun %d stale_half %d no_app_buf %d empty %d skip %d\n",
			atomic_read(&ts->direct), atomic_read(&ts->dma_overrun), atomic_read(&ts->stale_half),
			atomic_read(&ts->no_app_buf), atomic_read(&ts->worker_empty), atomic_read(&ts->ring_skip));
	}
}

/* ---- consumers ---- */
struct consumer {
	int ch, nbufs, proc_ms, stop_every;
	int fixed_ms;		/* exactly this long per frame (instead of random proc_ms) */
	atomic_long frames;	/* real (not no-signal) frames received */
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
	{
		/* monotonic: a wall-clock step (NTP, VM time sync) must not look like a stall */
		pthread_condattr_t ca;
		pthread_condattr_init(&ca);
		pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
		pthread_cond_init(&q->cond, &ca);
	}
	INIT_LIST_HEAD(&q->done);
	spin_lock_irqsave(&vd->consumers_lock, flags);
	list_add_tail(&c->ctx.node, &vd->consumers);
	spin_unlock_irqrestore(&vd->consumers_lock, flags);
	c->bufs = calloc(c->nbufs, sizeof(*c->bufs));
	for (i = 0; i < c->nbufs; i++) {
		c->bufs[i].vb.vb2_buf.vb2_queue = q;
		/* hostile consumers switch output sizes: buffers fit the largest */
		size_t alloc = hostile ? 1920 * 1080 * 2 : out_size;
		c->bufs[i].vb.vb2_buf.plane_size = alloc;
		c->bufs[i].mem = malloc(alloc);
	}

	while (atomic_load(&run)) {
		u32 last_seq = 0; u64 last_ts = 0; int last_v = -1, have_last = 0; long last_f = -1;
		struct hwsvideo_buffer *held = NULL;

		if (hostile) {
			static const int outs[][2] = { {1920,1080},{1280,720},{720,480},{720,576},{800,600},{640,480},
				{1024,768},{1280,768},{1280,800},{1280,1024},{1360,768},{1440,900},{1680,1050},{1080,1920} };
			int o = rand_r(&seed) % 14;
			spin_lock_irqsave(&pdx->videoslock[c->ch], flags);
			vd->current_out_width = outs[o][0];
			vd->curren_out_height = outs[o][1];
			spin_unlock_irqrestore(&pdx->videoslock[c->ch], flags);
		}
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
			clock_gettime(CLOCK_MONOTONIC, &dl);
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
					stall_dump(c->ch);
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
				if (hostile) {
					/* garbage in, garbage out: only safety and ordering are checked */
				} else if (!is_nosig(p) && (atomic_fetch_add(&c->frames, 1), 0)) {
				} else if (is_nosig(p)) {
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
			if (c->fixed_ms) {
				/* a display app: it keeps showing this frame until the next one
				 * arrives, and only then gives the previous buffer back */
				sleep_ns((u64)c->fixed_ms * NSEC_PER_MSEC);
				if (held)
					qbuf(c, held);
				held = b;
				n++;
				continue;
			}
			if (c->proc_ms)
				sleep_ns((u64)(rand_r(&seed) % (2 * c->proc_ms + 1)) * NSEC_PER_MSEC);
			qbuf(c, b);
			n++;
		}

		/* STREAMOFF: every buffer must be back when stop_streaming returns */
		held = NULL;	/* the app's own buffer: vb2 doesn't count it as queued */
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
		/* 2 buffers, 10 ms per frame: re-queues well before the next frame is
		 * complete (16.7 ms at 60 fps), so it must get (nearly) every frame */
		{ .ch = 1, .nbufs = 2, .fixed_ms = 10, .stop_every = 0 },
	};
	int ncons = sizeof(cons) / sizeof(cons[0]);
	pthread_t ct[16];
	int ch, i, j;

	if (argc > 1) fps = atoi(argv[1]);
	if (argc > 2) seconds = atoi(argv[2]);
	dma_lock_mode = getenv("SIM_DMA_LOCK") != NULL;
	hostile = getenv("SIM_HOSTILE") != NULL;
	ntsc = getenv("SIM_NTSC") != NULL;
	atomic_store(&hostile_now, hostile);
	if (hostile) printf("HOSTILE hardware: garbage registers, interrupt storm, DMA garbage, fake ALSA\n");
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
	pdx->m_dwAudioPTKSize = MAX_DMA_AUDIO_PK_SIZE;
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
		/* per channel: copies on one thread, conversion + fan-out on another */
		pdx->copy_worker[ch] = sim_kthread_worker();
		pdx->conv_worker[ch] = sim_kthread_worker();
		kthread_init_work(&pdx->video[ch].videowork, video_data_process);
		kthread_init_delayed_work(&pdx->video[ch].nosig_work, hws_nosig_work);
		rt_mutex_init(&pdx->direct_lock[ch]);
		INIT_LIST_HEAD(&pdx->direct_bufs[ch]);
		kthread_init_work(&pdx->video[ch].fanout_work, hws_fanout_work);
		kthread_init_work(&pdx->video[ch].copy_work, hws_copy_work);
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
		pdx->audio[ch].dev = pdx;
		pdx->audio[ch].index = ch;
		spin_lock_init(&pdx->audio[ch].ring_lock);
		INIT_WORK(&pdx->audio[ch].audiowork, audio_data_process);
		INIT_DELAYED_WORK(&pdx->audio[ch].silence_work, hws_audio_silence_work);
		if (ch < NCH) {
			pdx->m_pbyVideoBuffer[ch] = calloc(1, MAX_MM_VIDEO_SIZE);
			pdx->m_pbyAudioBuffer[ch] = pdx->m_pbyVideoBuffer[ch] + MAX_MM_VIDEO_SIZE - MAX_AUDIO_CAP_SIZE;
			for (j = 0; j < MAX_AUDIO_QUEUE; j++)
				pdx->m_AudioInfo[ch].m_pAudioBufData[j] = calloc(1, MAX_DMA_AUDIO_PK_SIZE);
			if (hostile) {
				pdx->m_AudioInfo[ch].dwisRuning = 1;
				pdx->m_bACapStarted[ch] = 1;
				pdx->m_bAudioRun[ch] = 1;
			}
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
	pthread_t hst[3];
	if (hostile) {
		pthread_create(&hst[0], NULL, storm_thread, NULL);
		pthread_create(&hst[1], NULL, garbage_thread, NULL);
		pthread_create(&hst[2], NULL, alsa_thread, NULL);
	}
	for (i = 0; i < ncons; i++)
		pthread_create(&ct[i], NULL, consumer_thread, &cons[i]);

	/*
	 * Audio with no audio from the card (the simulated card sends none outside
	 * hostile mode): an ALSA capture on ch1 must keep moving with silence at
	 * the card's pace instead of stalling. 1024-frame periods at 48 kHz.
	 */
	u64 silence_t0 = 0;
	if (!hostile) {
		struct hws_audio *drv = &pdx->audio[1];
		unsigned long fl;
		fake_rt[1].dma_area = calloc(16384, 4);
		fake_rt[1].buffer_size = 16384;
		fake_rt[1].period_size = 1024;
		fake_sub[1].runtime = &fake_rt[1];
		spin_lock_irqsave(&drv->ring_lock, fl);
		drv->substream = &fake_sub[1];
		drv->channels = 2;
		drv->ring_size_byframes = 16384;
		drv->period_size_byframes = 1024;
		drv->ring_wpos_byframes = 0;
		drv->period_used_byframes = 0;
		drv->running = true;
		spin_unlock_irqrestore(&drv->ring_lock, fl);
		/* as trigger START does */
		silence_t0 = ktime_get_ns();
		atomic64_set(&drv->last_real_ns, silence_t0);
		drv->silence_next_ns = 0;
		mod_delayed_work(pdx->wq, &drv->silence_work, 64);
	}
	sleep(seconds);
	if (!hostile) {
		struct hws_audio *drv = &pdx->audio[1];
		unsigned long fl;
		double secs = (ktime_get_ns() - silence_t0) / 1e9;
		long got = atomic_load(&periods_elapsed);
		spin_lock_irqsave(&drv->ring_lock, fl);
		drv->running = false;
		spin_unlock_irqrestore(&drv->ring_lock, fl);
		cancel_delayed_work_sync(&drv->silence_work);
		printf("audio without card audio: %ld periods in %.1f s, expected %.0f: %.0f%%\n",
		       got, secs, secs * 48000 / 1024, 100.0 * got / (secs * 48000 / 1024));
	}
	atomic_store(&run, false);
	for (i = 0; i < ncons; i++)
		pthread_join(ct[i], NULL);
	for (ch = 0; ch < NCH; ch++)
		pthread_join(hw[ch], NULL);
	pthread_mutex_lock(&irq_lock); irq_runthread = true; pthread_cond_signal(&irq_cond); pthread_mutex_unlock(&irq_lock);
	pthread_join(irq, NULL);
	pthread_join(st, NULL);
	if (hostile) {
		for (i = 0; i < 3; i++)
			pthread_join(hst[i], NULL);
		printf("hostile: ALSA periods elapsed %ld\n", (long)periods_elapsed);
	}
	printf("coverage: video half copies %ld, audio packet copies %ld, ALSA ring writes %ld, rate-limited warnings %ld\n",
	       (long)cov_video_copy, (long)cov_audio_copy, (long)cov_audio_deliver, (long)sim_ratelimited_warnings);

	printf("fps=%d seconds=%d channels=%d consumers=%d\n", fps, seconds, NCH, ncons);
	printf("frames from hw           %ld  (lost second-half IRQs %ld, duplicate first-half IRQs %ld)\n", frames_hw, lost_irqs, dup_irqs);
	for (i = 0; i < ncons; i++)
		if (cons[i].fixed_ms)
			printf("steady app (ch%d, %d bufs, %d ms/frame) got %ld of %ld frames: %.0f%%\n",
			       cons[i].ch, cons[i].nbufs, cons[i].fixed_ms, (long)cons[i].frames, (long)frames_hw_ch[cons[i].ch],
			       frames_hw_ch[cons[i].ch] ? 100.0 * cons[i].frames / frames_hw_ch[cons[i].ch] : 0.0);
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
	/* the measured rate (G_PARM) */
	long rate_err = atomic_load(&rate_bad);
	printf("frame rate measurements: right %ld, wrong %ld <- must be 0\n", (long)atomic_load(&rate_ok), rate_err);
	return (torn_steady || true_dups || order_err || seq_err || ts_err || stop_leaks || done_after_stop || rate_err) ? 1 : 0;
}
