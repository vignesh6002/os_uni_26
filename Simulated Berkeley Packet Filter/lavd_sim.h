#ifndef __LAVD_SIM_H
#define __LAVD_SIM_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* Standard integer types */
typedef uint64_t u64;
typedef int64_t  s64;
typedef uint32_t u32;
typedef int32_t  s32;
typedef uint16_t u16;
typedef int16_t  s16;
typedef uint8_t  u8;
typedef int8_t   s8;
typedef int32_t  pid_t;

/* Common macros */
#define U64_MAX         ((u64)~0ULL)
#define S64_MAX         ((s64)(U64_MAX >> 1))
#define LAVD_COMP_TIME_INF (U64_MAX >> 2)
#define U32_MAX         ((u32)~0U)
#define S32_MAX         ((s32)(U32_MAX >> 1))

#define MAX_RT_PRIO     100
#define TASK_COMM_LEN   32


#define LAVD_SHIFT      10
#define LAVD_SCALE      (1ULL << LAVD_SHIFT)
#define p2s(percent)    (((percent) << LAVD_SHIFT) / 100)
#define s2p(scale)      (((scale) * 100) >> LAVD_SHIFT)

#define NSEC_PER_USEC   1000ULL
#define NSEC_PER_MSEC   (1000ULL * NSEC_PER_USEC)
#define LAVD_TIME_ONE_SEC (1000ULL * NSEC_PER_MSEC)
#define LAVD_MAX_RETRY  3

#define LAVD_TARGETED_LATENCY_NS  (10ULL * NSEC_PER_MSEC)
#define LAVD_SLICE_MIN_NS_DFL     (500ULL * NSEC_PER_USEC) /* 0.5 ms */
#define LAVD_SLICE_MAX_NS_DFL     (5ULL * NSEC_PER_MSEC)   /* 5 ms */
#define LAVD_SLICE_BOOST_BONUS    LAVD_SLICE_MIN_NS_DFL
#define LAVD_SLICE_BOOST_MAX      (500ULL * NSEC_PER_MSEC) /* 500 ms */
#define LAVD_SLICE_BOOST_UTIL_WALL p2s(95)
#define LAVD_ACC_RUNTIME_MAX      LAVD_SLICE_MAX_NS_DFL
#define LAVD_TASK_LAG_MAX         (500ULL * NSEC_PER_MSEC)
#define LAVD_DL_COMPETE_WINDOW    ((300ULL * NSEC_PER_MSEC) >> 16)

#define LAVD_LC_FREQ_MAX                100000
#define LAVD_LC_RUNTIME_MAX             LAVD_TIME_ONE_SEC
#define LAVD_LC_WEIGHT_BOOST_REGULAR    128
#define LAVD_LC_WEIGHT_BOOST_MEDIUM     (2 * LAVD_LC_WEIGHT_BOOST_REGULAR)
#define LAVD_LC_WEIGHT_BOOST_HIGH       (2 * LAVD_LC_WEIGHT_BOOST_MEDIUM)
#define LAVD_LC_WEIGHT_BOOST_HIGHEST    (2 * LAVD_LC_WEIGHT_BOOST_HIGH)
#define LAVD_LC_GREEDY_SHIFT            1 /* 50% */
#define LAVD_LC_WAKE_INTERVAL_MIN       LAVD_SLICE_MIN_NS_DFL
#define LAVD_LC_INH_RECEIVER_SHIFT      2 /* 25% */
#define LAVD_LC_INH_GIVER_SHIFT         3 /* 12.5% */
#define LAVD_LC_LATENCY_SENSITIVE_THRESH (LAVD_SCALE - (LAVD_SCALE >> 3))

/* Arithmetic helpers */
#ifndef min
#define min(X, Y) (((X) < (Y)) ? (X) : (Y))
#endif

#ifndef max
#define max(X, Y) (((X) < (Y)) ? (Y) : (X))
#endif

#ifndef clamp
#define clamp(val, lo, hi) min(max(val, lo), hi)
#endif

#define time_delta(a, b) (((a) > (b)) ? ((a) - (b)) : 0ULL)

#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, val) ((x) = (val))
#define __hidden
#define __weak


/* Integer log2 */
static inline u32 log2_u64(u64 v)
{
    if (v == 0)
        return 0;
#if defined(__GNUC__) || defined(__clang__)
    return 63 - __builtin_clzll(v);
#else
    u32 r = 0;
    while (v >>= 1)
        r++;
    return r;
#endif
}

/* Const flags */
enum consts_flags {
    LAVD_FLAG_FUTEX_BOOST           = (0x1 << 0),
    LAVD_FLAG_NEED_LOCK_BOOST       = (0x1 << 1),
    LAVD_FLAG_IS_GREEDY             = (0x1 << 2),
    LAVD_FLAG_IS_AFFINITIZED        = (0x1 << 3),
    LAVD_FLAG_IS_WAKEUP             = (0x1 << 4),
    LAVD_FLAG_IS_SYNC_WAKEUP        = (0x1 << 5),
    LAVD_FLAG_ON_BIG                = (0x1 << 6),
    LAVD_FLAG_ON_LITTLE             = (0x1 << 7),
    LAVD_FLAG_SLICE_BOOST           = (0x1 << 8),
    LAVD_FLAG_IDLE_CPU_PICKED       = (0x1 << 9),
    LAVD_FLAG_KSOFTIRQD             = (0x1 << 10),
    LAVD_FLAG_WOKEN_BY_RT_DL        = (0x1 << 11),
    LAVD_FLAG_WOKEN_BY_HARDIRQ      = (0x1 << 12),
    LAVD_FLAG_WOKEN_BY_SOFTIRQ      = (0x1 << 13),
    LAVD_FLAG_MIGRATION_AGGRESSIVE  = (0x1 << 14),
    LAVD_FLAG_DOMAIN_PINNED         = (0x1 << 15),
    LAVD_FLAG_IS_EFFECTIVELY_PINNED = (0x1 << 16),
    LAVD_FLAG_WARM_CPU              = (0x1 << 17),
};

/* Forward declarations */
struct task_struct;
struct task_ctx;
struct cpu_ctx;
struct sys_stat;

/* Task entities */
struct scx_entity {
    u64 weight;
    u64 slice;
    u64 dsq_vtime;
};

struct task_struct {
    pid_t pid;
    char comm[TASK_COMM_LEN + 1];
    struct scx_entity scx;
    u64 flags;
    int migration_disabled;
};

typedef struct task_ctx {
    u64 flags;
    u64 slice_wall;
    u64 last_measured_wall_clk;
    u64 acc_runtime_wall;
    u64 acc_runtime_invr;
    u64 svc_time_iwgt;

    u16 lat_cri;
    u16 normalized_lat_cri;
    u16 lat_cri_waker;
    u16 lat_cri_wakee;
    u16 perf_cri;

    u64 last_running_clk;
    u64 run_freq;
    u64 wait_freq;
    u64 wake_freq;
    u64 avg_runtime_invr;
    u64 avg_runtime_wall;

    u32 cpu_id;
    u32 prev_cpu_id;
    pid_t pid;
    u32 util_est;
} task_ctx;

struct cpu_ctx {
    u16 cpu_id;
    u16 effective_capacity;
    u16 max_capacity;
    u32 nr_pinned_tasks;
    u64 running_clk;
    u64 est_stopping_clk;
    u32 lat_headroom;
    u32 avg_util_wall;
    u32 cur_util_wall;
};

struct sys_stat {
    u64 slice_wall;
    u32 nr_queued_task;
    u32 avg_lat_cri;
    u64 avg_svc_time_iwgt;
    u32 cur_util_wall;
    u32 avg_util_wall;
};

/* Global scheduler simulation variables */
extern struct sys_stat sys_stat;
extern u64 cur_logical_clk;
extern u64 nr_cpus_onln;
extern bool have_little_core;
extern bool no_slice_boost;

/* Task and flag helpers */
static inline bool test_task_flag(task_ctx *taskc, u64 flag) {
    return (taskc->flags & flag) != 0;
}

static inline void set_task_flag(task_ctx *taskc, u64 flag) {
    taskc->flags |= flag;
}

static inline void reset_task_flag(task_ctx *taskc, u64 flag) {
    taskc->flags &= ~flag;
}

static inline bool is_kernel_task(struct task_struct *p) {
    return (p->flags & 0x00200000) != 0;
}

static inline bool is_kernel_worker(struct task_struct *p) {
    return (p->flags & 0x00000020) != 0;
}

static inline bool is_effectively_pinned(task_ctx *taskc) {
    return (taskc->flags & LAVD_FLAG_IS_EFFECTIVELY_PINNED) != 0;
}

static inline bool is_migration_disabled(struct task_struct *p) {
    return p->migration_disabled != 0;
}

static inline u64 scale_by_task_weight_inverse(struct task_struct *p, u64 val) {
    u64 w = p->scx.weight ? p->scx.weight : 1024ULL;
    return (val * 1024ULL) / w;
}

static inline u16 normalize_lat_cri(u16 lat_cri) {
    if (lat_cri > 1024)
        return 1024;
    return lat_cri;
}

static inline bool can_boost_slice(void) {
    return sys_stat.avg_util_wall < LAVD_SLICE_BOOST_UTIL_WALL;
}

/* Functions declared in latency_critical.c & main-bpf.c */
void calc_lat_cri(struct task_struct *p, task_ctx *taskc);
u64 calc_greedy_penalty(struct task_struct *p, task_ctx *taskc);
u64 calc_adjusted_runtime(task_ctx *taskc);
u64 calc_virtual_deadline_delta(struct task_struct *p, task_ctx *taskc);
u64 calc_compete_window(void);
u64 calc_when_to_run(struct task_struct *p, task_ctx *taskc);
u64 calc_time_slice(task_ctx *taskc, struct cpu_ctx *cpuc);
void advance_cur_logical_clk(struct task_struct *p);

#endif /* __LAVD_SIM_H */
