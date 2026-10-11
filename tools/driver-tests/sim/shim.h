/*
 * Minimal userspace model of the kernel APIs the driver's capture core uses.
 * Semantics that matter for the test are kept:
 *  - spinlocks/mutexes are real locks (pthread)
 *  - a work item never runs concurrently with itself; queue_work on a pending
 *    item returns false; queueing a running item runs it once more afterwards;
 *    flush_work waits for the running instance and anything pending;
 *    cancel_work_sync drops pending and waits for the running instance
 *  - delayed work fires once after its delay unless cancelled
 *  - vb2_buffer_done hands the buffer back to its queue (test checks ownership)
 */
#ifndef SHIM_H
#define SHIM_H
#define _GNU_SOURCE
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;
typedef int32_t s32; typedef int64_t s64; typedef uint32_t __le32; typedef u64 dma_addr_t;
typedef u64 __le64;
#define cpu_to_le64(x) ((__le64)(x))
#define le64_to_cpu(x) ((u64)(x))
typedef unsigned int __le32;
#define le32_to_cpu(x) ((u32)(x))
typedef u64 v4l2_std_id; typedef unsigned int __poll_t;
#define U64_MAX UINT64_MAX
#define NSEC_PER_SEC 1000000000ULL
#define NSEC_PER_MSEC 1000000ULL
#define NSEC_PER_USEC 1000ULL
#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))
#define LINUX_VERSION_CODE KERNEL_VERSION(7, 0, 0)
#define KERN_ERR ""
#define KERN_WARNING ""
#define KERN_INFO ""
#define printk printf
#define pr_debug(...) do { } while (0)
#define pr_info(...) do { } while (0)
extern atomic_long sim_ratelimited_warnings;
#define pr_warn_ratelimited(...) atomic_fetch_add(&sim_ratelimited_warnings, 1)
#define printk_ratelimited(...) atomic_fetch_add(&sim_ratelimited_warnings, 1)
#define cond_resched() sched_yield()
#define min(a, b) ((a) < (b) ? (a) : (b))
#define container_of(p, t, m) ((t *)((char *)(p) - offsetof(t, m)))
#define __iomem
#define __user
#define READ_ONCE(x) (*(volatile __typeof__(x) *)&(x))
#define WRITE_ONCE(x, v) (*(volatile __typeof__(x) *)&(x) = (v))
#define rmb() atomic_thread_fence(memory_order_acquire)
#define smp_rmb() atomic_thread_fence(memory_order_acquire)
#define smp_wmb() atomic_thread_fence(memory_order_release)
#define wmb() atomic_thread_fence(memory_order_release)
#define smp_mb() atomic_thread_fence(memory_order_seq_cst)
#define smp_store_release(p, v) do { atomic_thread_fence(memory_order_release); WRITE_ONCE(*(p), (v)); } while (0)
#define smp_load_acquire(p) ({ __typeof__(*(p)) ___v = READ_ONCE(*(p)); atomic_thread_fence(memory_order_acquire); ___v; })
#define max_t(t, a, b) ((t)(a) > (t)(b) ? (t)(a) : (t)(b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define div_s64(a, b) ((s64)(a) / (b))
#define num_online_cpus() 16
#define min_t(t, a, b) ((t)(a) < (t)(b) ? (t)(a) : (t)(b))
#define BIT_ULL(n) (1ULL << (n))
#define clamp_t(t, v, lo, hi) min_t(t, max_t(t, v, lo), hi)
#define queue_delayed_work(wq, d, j) mod_delayed_work((wq), (d), (j))
#define cpu_relax() sched_yield()
#define smp_mb__before_atomic() atomic_thread_fence(memory_order_seq_cst)
#define div_u64(a, b) ((u64)(a) / (b))
#define div64_u64(a, b) ((u64)(a) / (b))
#define msecs_to_jiffies(ms) (ms)
#define nsecs_to_jiffies(ns) ((unsigned long)((ns) / NSEC_PER_MSEC))	/* jiffies are ms here */
#define HZ 1000
#define DIV_ROUND_UP_ULL(n, d) (((unsigned long long)(n) + (d) - 1) / (d))

static inline u64 ktime_get_ns(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (u64)t.tv_sec * NSEC_PER_SEC + t.tv_nsec;
}

/* ---- bits / atomics ---- */
static inline void set_bit(int nr, unsigned long *a)
{ __atomic_fetch_or(a, 1UL << nr, __ATOMIC_SEQ_CST); }
static inline void clear_bit(int nr, unsigned long *a)
{ __atomic_fetch_and(a, ~(1UL << nr), __ATOMIC_SEQ_CST); }
typedef struct { atomic_int v; } atomic_t;
#define atomic_inc(a) atomic_fetch_add(&(a)->v, 1)
#define atomic_add(n, a) atomic_fetch_add(&(a)->v, (n))
#define atomic_read(a) atomic_load(&(a)->v)
#define atomic_set(a, x) atomic_store(&(a)->v, (x))
#define atomic_xchg(a, x) atomic_exchange(&(a)->v, (x))
struct kref { atomic_t refcount; };
typedef struct { _Atomic s64 v; } atomic64_t;
#define atomic64_set(a, x) atomic_store(&(a)->v, (x))
#define atomic64_read(a) atomic_load(&(a)->v)
#define atomic64_add(x, a) atomic_fetch_add(&(a)->v, (x))
#define atomic64_xchg(a, x) atomic_exchange(&(a)->v, (x))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* ---- locks ---- */
typedef struct { pthread_mutex_t m; } spinlock_t;
#define spin_lock_init(l) pthread_mutex_init(&(l)->m, NULL)
#define spin_lock(l) pthread_mutex_lock(&(l)->m)
#define spin_unlock(l) pthread_mutex_unlock(&(l)->m)
#define spin_lock_irqsave(l, f) do { (f) = 0; pthread_mutex_lock(&(l)->m); } while (0)
#define spin_unlock_irqrestore(l, f) do { (void)(f); pthread_mutex_unlock(&(l)->m); } while (0)
struct mutex { pthread_mutex_t m; };
#define mutex_init(l) pthread_mutex_init(&(l)->m, NULL)
/* rt_mutex: priority inheritance doesn't matter without RT scheduling here */
#define rt_mutex mutex
#define rt_mutex_init(l) mutex_init(l)
#define rt_mutex_lock(l) mutex_lock(l)
#define rt_mutex_unlock(l) mutex_unlock(l)
#define mutex_lock(l) pthread_mutex_lock(&(l)->m)
#define mutex_unlock(l) pthread_mutex_unlock(&(l)->m)

/* ---- lists ---- */
struct list_head { struct list_head *next, *prev; };
#define LIST_HEAD(n) struct list_head n = { &(n), &(n) }
static inline void INIT_LIST_HEAD(struct list_head *l) { l->next = l->prev = l; }
static inline void __list_add(struct list_head *n, struct list_head *p, struct list_head *x)
{ x->prev = n; n->next = x; n->prev = p; p->next = n; }
static inline void list_add_tail(struct list_head *n, struct list_head *h) { __list_add(n, h->prev, h); }
static inline void list_del(struct list_head *e)
{ e->next->prev = e->prev; e->prev->next = e->next; e->next = e->prev = (void *)0xdead; }
static inline void list_move_tail(struct list_head *e, struct list_head *h)
{ e->next->prev = e->prev; e->prev->next = e->next; list_add_tail(e, h); }
static inline int list_empty(const struct list_head *h) { return h->next == h; }
static inline int list_is_singular(const struct list_head *h) { return !list_empty(h) && h->next == h->prev; }
static inline void list_move(struct list_head *e, struct list_head *h)
{ e->next->prev = e->prev; e->prev->next = e->next; __list_add(e, h, h->next); }
static inline void list_splice_init(struct list_head *l, struct list_head *h)
{
	if (!list_empty(l)) {
		struct list_head *f = l->next, *t = l->prev, *at = h->next;
		f->prev = h; h->next = f; t->next = at; at->prev = t;
		INIT_LIST_HEAD(l);
	}
}
#define list_entry(p, t, m) container_of(p, t, m)
#define list_first_entry(h, t, m) list_entry((h)->next, t, m)
#define list_first_entry_or_null(h, t, m) (list_empty(h) ? NULL : list_first_entry(h, t, m))
#define list_for_each(p, h) for (p = (h)->next; p != (h); p = p->next)
#define list_for_each_entry(p, h, m) \
	for (p = list_entry((h)->next, __typeof__(*p), m); &p->m != (h); p = list_entry(p->m.next, __typeof__(*p), m))
#define list_for_each_entry_safe(p, n, h, m) \
	for (p = list_entry((h)->next, __typeof__(*p), m), n = list_entry(p->m.next, __typeof__(*p), m); \
	     &p->m != (h); p = n, n = list_entry(n->m.next, __typeof__(*n), m))
#define list_for_each_entry_safe_reverse(p, n, h, m) \
	for (p = list_entry((h)->prev, __typeof__(*p), m), n = list_entry(p->m.prev, __typeof__(*p), m); \
	     &p->m != (h); p = n, n = list_entry(n->m.prev, __typeof__(*n), m))

/* ---- workqueues ---- */
struct work_struct {
	void (*func)(struct work_struct *);
	pthread_mutex_t lock;
	pthread_cond_t cond;
	bool pending, running, started, stop;
	pthread_t thr;
};
struct delayed_work {
	struct work_struct work;
	pthread_mutex_t lock;
	pthread_cond_t cond;
	u64 expires;		/* 0 = not armed */
	bool started;
	pthread_t thr;
};
struct workqueue_struct { int dummy; };
void INIT_WORK(struct work_struct *w, void (*f)(struct work_struct *));
void INIT_DELAYED_WORK(struct delayed_work *d, void (*f)(struct work_struct *));
bool queue_work(struct workqueue_struct *wq, struct work_struct *w);
/* kthread_worker: one thread runs its works in queue order, like the kernel's */
struct kthread_worker;
struct kthread_work {
	void (*func)(struct kthread_work *);
	struct kthread_work *next;
	struct kthread_worker *worker;
	bool pending, running;
};
struct kthread_worker {
	pthread_mutex_t lock;
	pthread_cond_t cond;
	struct kthread_work *head, *tail;
	struct task_struct *task;
	pthread_t thr;
};
void kthread_init_work(struct kthread_work *w, void (*f)(struct kthread_work *));
bool kthread_queue_work(struct kthread_worker *wk, struct kthread_work *w);
void kthread_flush_work(struct kthread_work *w);
bool kthread_cancel_work_sync(struct kthread_work *w);
struct kthread_worker *sim_kthread_worker(void);
struct kthread_delayed_work {
	struct kthread_work work;
	struct kthread_worker *worker;
	pthread_mutex_t lock;
	pthread_cond_t cond;
	u64 expires;
	pthread_t thr;
};
void kthread_init_delayed_work(struct kthread_delayed_work *d, void (*f)(struct kthread_work *));
bool kthread_mod_delayed_work(struct kthread_worker *wk, struct kthread_delayed_work *d, unsigned long ms);
bool kthread_cancel_delayed_work_sync(struct kthread_delayed_work *d);
void kthread_flush_worker(struct kthread_worker *wk);
#define memcpy_flushcache(d, s, n) memcpy((d), (s), (n))
#define queue_work_on(cpu, wq, w) queue_work((wq), (w))
#define raw_smp_processor_id() 0
bool flush_work(struct work_struct *w);
bool cancel_work_sync(struct work_struct *w);
bool mod_delayed_work(struct workqueue_struct *wq, struct delayed_work *d, unsigned long ms);
bool cancel_delayed_work_sync(struct delayed_work *d);
#define to_delayed_work(w) container_of(w, struct delayed_work, work)

/* ---- vb2 / v4l2 (only what the capture core touches) ---- */
enum vb2_buffer_state { VB2_BUF_STATE_DEQUEUED, VB2_BUF_STATE_QUEUED, VB2_BUF_STATE_ACTIVE,
			VB2_BUF_STATE_DONE, VB2_BUF_STATE_ERROR };
#define V4L2_FIELD_NONE 1
struct vb2_queue {
	void *drv_priv;
	/* test bookkeeping */
	pthread_mutex_t lock;
	pthread_cond_t cond;
	struct list_head done;		/* completed, not yet dequeued by the test */
	int owned_by_driver;
	bool stopped;			/* stop_streaming has returned */
	long done_after_stop;		/* must stay 0 */
};
struct vb2_buffer {
	struct vb2_queue *vb2_queue;
	u64 timestamp;
	size_t plane_size, payload;
	enum vb2_buffer_state state;
	struct list_head done_entry;
};
struct vb2_v4l2_buffer { struct vb2_buffer vb2_buf; u32 sequence; u32 field; };
#define to_vb2_v4l2_buffer(vb) container_of(vb, struct vb2_v4l2_buffer, vb2_buf)
static inline size_t vb2_plane_size(struct vb2_buffer *vb, int p) { (void)p; return vb->plane_size; }
static inline void vb2_set_plane_payload(struct vb2_buffer *vb, int p, size_t s) { (void)p; vb->payload = s; }
void vb2_buffer_done(struct vb2_buffer *vb, enum vb2_buffer_state state);

/* opaque things the header only embeds */
struct v4l2_fh { int dummy; };
struct v4l2_ctrl_handler { int dummy; };	/* only embedded; the capture core never uses it */
struct v4l2_device { int dummy; void (*release)(struct v4l2_device *); };
struct video_device { int dummy; };
struct device { int unused; };
struct pci_dev { unsigned short device, vendor; struct device dev; };
/* the device argument isn't evaluated: pdx->pdev is NULL in the simulation */
#define dev_err(d, ...) fprintf(stderr, __VA_ARGS__)
#define dev_warn(d, ...) fprintf(stderr, __VA_ARGS__)
#define dev_info(d, ...) fprintf(stderr, __VA_ARGS__)
#define dev_dbg(d, ...) do { } while (0)
struct uio_info { struct { void *internal_addr; unsigned long addr, size; int memtype; } mem[1]; };
struct snd_card; struct task_struct;
/* just enough ALSA for the driver's audio delivery path */
struct snd_pcm_runtime { unsigned char *dma_area; unsigned long buffer_size, period_size; };
struct snd_pcm_substream { struct snd_pcm_runtime *runtime; };
void snd_pcm_period_elapsed(struct snd_pcm_substream *s);
struct msix_entry { int dummy; };

/* ---- irq ---- */
typedef int irqreturn_t;
#define IRQ_NONE 0
#define IRQ_HANDLED 1
#define IRQ_WAKE_THREAD 2

#endif
#define pci_device_is_present(p) (1)	/* the simulated card is never unplugged */
#define CONFIG_X86 1
#define PCI_COMMAND 0x04
#define PCI_COMMAND_MEMORY 0x2
#define pci_read_config_word(p, off, v) ((void)(p), (void)(off), *(v) = PCI_COMMAND_MEMORY, 0)
#define IS_ENABLED(x) (x)
#define dev_warn_ratelimited(d, ...) fprintf(stderr, __VA_ARGS__)
#ifndef SIM_SHIM_AVAIL
#define SIM_SHIM_AVAIL
/* the simulated apps read every period at once */
static inline unsigned long snd_pcm_capture_avail(struct snd_pcm_runtime *r) { (void)r; return 0; }
#endif
#define max3(a, b, c) max(max(a, b), c)
#define irq_get_irq_data(irq) ((struct irq_data *)0)
#define irqd_irq_disabled(d) 0
struct irq_data;
#ifndef SIM_SHIM_UNREAD
#define SIM_SHIM_UNREAD
struct hws_audio;
/* the simulated apps read every period at once */
static inline long hws_audio_unread(struct hws_audio *d, struct snd_pcm_runtime *r) { (void)d; (void)r; return 0; }
#endif
