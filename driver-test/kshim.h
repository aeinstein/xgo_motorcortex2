/* Minimale Nachbildung der benutzten Kernel-APIs im Userspace (Test-Harness). */
#pragma once
#define _GNU_SOURCE
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdatomic.h>

typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;
typedef int32_t s32; typedef int64_t s64;
#ifndef S32_MAX
#define S32_MAX INT32_MAX
#define S32_MIN INT32_MIN
#endif
#define NSEC_PER_SEC 1000000000ULL
#define NSEC_PER_MSEC 1000000ULL

#define pr_info(...)  do { printf(__VA_ARGS__); } while (0)
#define pr_warn(...)  do { printf("WARN: " __VA_ARGS__); } while (0)
#define pr_err(...)   do { printf("ERR: " __VA_ARGS__); } while (0)
#define pr_cont(...)  do { printf(__VA_ARGS__); } while (0)

#define READ_ONCE(x) (*(volatile __typeof__(x) *)&(x))
#define WRITE_ONCE(x, v) (*(volatile __typeof__(x) *)&(x) = (v))
static inline u64 div_u64(u64 a, u32 b) { return a / b; }

/* atomics */
typedef struct { atomic_int v; } atomic_t;
#define ATOMIC_INIT(i) { (i) }
#define atomic_inc(a) atomic_fetch_add(&(a)->v, 1)
#define atomic_read(a) atomic_load(&(a)->v)

/* Spinlock/Mutex -> pthread */
typedef pthread_mutex_t spinlock_t;
#define DEFINE_SPINLOCK(n) spinlock_t n = PTHREAD_MUTEX_INITIALIZER
#define spin_lock(l) pthread_mutex_lock(l)
#define spin_unlock(l) pthread_mutex_unlock(l)
#define spin_lock_irqsave(l, f) do { (void)(f); pthread_mutex_lock(l); } while (0)
#define spin_unlock_irqrestore(l, f) do { (void)(f); pthread_mutex_unlock(l); } while (0)
struct mutex { pthread_mutex_t m; };
#define DEFINE_MUTEX(n) struct mutex n = { PTHREAD_MUTEX_INITIALIZER }
#define mutex_lock(x) pthread_mutex_lock(&(x)->m)
#define mutex_unlock(x) pthread_mutex_unlock(&(x)->m)
#define mutex_lock_interruptible(x) (pthread_mutex_lock(&(x)->m), 0)

/* completion: jiffies = ms */
struct completion { pthread_mutex_t m; pthread_cond_t c; int done; };
#define DECLARE_COMPLETION(n) struct completion n = { PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, 0 }
static inline void reinit_completion(struct completion *x) { pthread_mutex_lock(&x->m); x->done = 0; pthread_mutex_unlock(&x->m); }
static inline void complete(struct completion *x) { pthread_mutex_lock(&x->m); x->done++; pthread_cond_signal(&x->c); pthread_mutex_unlock(&x->m); }
static inline unsigned long msecs_to_jiffies(unsigned int ms) { return ms; }
static inline long wait_for_completion_timeout(struct completion *x, unsigned long ms) {
	struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_nsec += (long)(ms % 1000) * 1000000L; ts.tv_sec += ms / 1000 + ts.tv_nsec / 1000000000L; ts.tv_nsec %= 1000000000L;
	pthread_mutex_lock(&x->m);
	int rc = 0;
	while (!x->done && rc == 0) rc = pthread_cond_timedwait(&x->c, &x->m, &ts);
	long r = x->done ? 1 : 0;
	if (x->done) x->done--;
	pthread_mutex_unlock(&x->m);
	return r;
}

/* Speicher */
#define GFP_KERNEL 0
#define kmalloc(n, f) malloc(n)
#define kfree(p) free(p)

/* Liste (wie linux/list.h, minimal) */
struct list_head { struct list_head *next, *prev; };
#define LIST_HEAD(n) struct list_head n = { &(n), &(n) }
static inline void list_add_tail(struct list_head *n, struct list_head *h) { n->prev = h->prev; n->next = h; h->prev->next = n; h->prev = n; }
static inline void list_del(struct list_head *e) { e->prev->next = e->next; e->next->prev = e->prev; }
#define container_of(p, t, m) ((t *)((char *)(p) - offsetof(t, m)))
#include <stddef.h>
#define list_for_each_entry_safe(pos, n, head, member) \
	for (pos = container_of((head)->next, __typeof__(*pos), member), n = container_of(pos->member.next, __typeof__(*pos), member); \
	     &pos->member != (head); pos = n, n = container_of(n->member.next, __typeof__(*n), member))

/* Datei/TTY */
struct file { int fd; };
#define O_SYNC_IGNORED 0
static inline bool IS_ERR(const void *p) { return (uintptr_t)p >= (uintptr_t)-4095; }
static inline bool IS_ERR_OR_NULL(const void *p) { return !p || IS_ERR(p); }
static inline long PTR_ERR(const void *p) { return (long)(intptr_t)p; }
extern const char *g_serial_path;
static inline struct file *filp_open(const char *path, int flags, int mode) {
	(void)path; (void)mode;
	struct file *f = malloc(sizeof(*f));
	f->fd = open(g_serial_path, O_RDWR | O_NOCTTY);
	if (f->fd < 0) { free(f); return (struct file *)(intptr_t)-ENOENT; }
	struct termios t; tcgetattr(f->fd, &t); cfmakeraw(&t); t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 1; tcsetattr(f->fd, TCSANOW, &t);
	return f;
}
static inline void filp_close(struct file *f, void *x) { (void)x; close(f->fd); free(f); }
static inline ssize_t kernel_read(struct file *f, void *b, size_t n, loff_t *p) { (void)p; ssize_t r = read(f->fd, b, n); return r < 0 ? -errno : r; }
static inline ssize_t kernel_write(struct file *f, const void *b, size_t n, loff_t *p) { (void)p; return write(f->fd, b, n); }

/* Zeit */
static inline u64 ktime_get_ns(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (u64)ts.tv_sec * 1000000000ULL + ts.tv_nsec; }
static inline void msleep(unsigned int ms) { usleep(ms * 1000); }
static inline void usleep_range(unsigned long a, unsigned long b) { (void)b; usleep(a); }

/* kthreads */
struct task_struct { pthread_t th; int (*fn)(void *); void *arg; atomic_int stop; };
static __thread struct task_struct *current_task;
static void *kthread_tramp(void *a) { struct task_struct *t = a; current_task = t; t->fn(t->arg); return NULL; }
static inline struct task_struct *kthread_run(int (*fn)(void *), void *arg, const char *name) {
	(void)name; struct task_struct *t = calloc(1, sizeof(*t)); t->fn = fn; t->arg = arg;
	pthread_create(&t->th, NULL, kthread_tramp, t); return t;
}
static inline bool kthread_should_stop(void) { return atomic_load(&current_task->stop); }
static inline int kthread_stop(struct task_struct *t) { atomic_store(&t->stop, 1); pthread_join(t->th, NULL); free(t); return 0; }

static inline int abs_s32(int x) { return x < 0 ? -x : x; }
