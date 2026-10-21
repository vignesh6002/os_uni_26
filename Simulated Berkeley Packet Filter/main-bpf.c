/*
 * LAVD Simulation Driver: Latency-criticality Aware Virtual Deadline
 * =================================================================
 * Standalone simulation that accepts synthetic tasks, calculates:
 *   1) Context-aware Latency Criticality (lat_cri)
 *   2) Fairness / Greedy Penalty
 *   3) Virtual Deadline (calc_when_to_run -> dsq_vtime)
 *   4) Allocated Time Slice (calc_time_slice -> slice_wall)
 *   5) Dispatch Queue Order (Earliest Virtual Deadline First)
 */

#include "lavd_sim.h"

/* Global scheduler state */
struct sys_stat sys_stat = {
    .slice_wall = LAVD_SLICE_MAX_NS_DFL,    /* 5 ms default */
    .nr_queued_task = 0,
    .avg_lat_cri = 100,
    .avg_svc_time_iwgt = 10ULL * NSEC_PER_MSEC, /* 10 ms average service time */
    .cur_util_wall = p2s(50),               /* 50% system load */
    .avg_util_wall = p2s(50)
};

u64 cur_logical_clk = LAVD_DL_COMPETE_WINDOW;
u64 nr_cpus_onln = 4;
bool have_little_core = false;
bool no_slice_boost = false;
const volatile u64 slice_min_ns = LAVD_SLICE_MIN_NS_DFL;
const volatile u64 slice_max_ns = LAVD_SLICE_MAX_NS_DFL;
const volatile u64 pinned_slice_ns = 0;

/* Virtual logical clock advance (from LAVD main-bpf.c) */
void advance_cur_logical_clk(struct task_struct *p)
{
    u64 vlc, clc, nr_queued, delta;

    vlc = READ_ONCE(p->scx.dsq_vtime);
    clc = READ_ONCE(cur_logical_clk);

    if (vlc <= clc)
        return;

    nr_queued = max(sys_stat.nr_queued_task, 1);
    delta = (vlc - clc) / nr_queued;
    cur_logical_clk = clc + delta;
}

/* Time slice calculation (from LAVD main-bpf.c) */
u64 calc_time_slice(task_ctx *taskc, struct cpu_ctx *cpuc)
{
    if (!taskc || !cpuc)
        return LAVD_SLICE_MAX_NS_DFL;

    if (pinned_slice_ns && cpuc->nr_pinned_tasks) {
        taskc->slice_wall = min(pinned_slice_ns, sys_stat.slice_wall);
        reset_task_flag(taskc, LAVD_FLAG_SLICE_BOOST);
        return taskc->slice_wall;
    }

    if (!no_slice_boost && !cpuc->nr_pinned_tasks &&
        (taskc->avg_runtime_wall >= sys_stat.slice_wall)) {
        
        if (can_boost_slice()) {
            u64 s = taskc->avg_runtime_wall + LAVD_SLICE_BOOST_BONUS;
            taskc->slice_wall = clamp(s, slice_min_ns, LAVD_SLICE_BOOST_MAX);
            set_task_flag(taskc, LAVD_FLAG_SLICE_BOOST);
            return taskc->slice_wall;
        }

        if (taskc->lat_cri > sys_stat.avg_lat_cri) {
            u64 b = (sys_stat.slice_wall * taskc->lat_cri) / (sys_stat.avg_lat_cri + 1);
            u64 s = sys_stat.slice_wall + b;
            taskc->slice_wall = clamp(s, slice_min_ns, min(taskc->avg_runtime_wall, sys_stat.slice_wall * 2));
            set_task_flag(taskc, LAVD_FLAG_SLICE_BOOST);
            return taskc->slice_wall;
        }
    }

    taskc->slice_wall = sys_stat.slice_wall;
    reset_task_flag(taskc, LAVD_FLAG_SLICE_BOOST);
    return taskc->slice_wall;
}

/* Definition for synthetic tasks */
typedef struct synthetic_task_def {
    const char *name;
    pid_t pid;
    u64 weight;             /* 1024 = normal priority (nice 0) */
    u64 avg_runtime_us;     /* average runtime in microseconds */
    u64 acc_runtime_us;     /* accumulated runtime since last sleep */
    u64 wait_freq;          /* times waited per second */
    u64 wake_freq;          /* times woken other tasks per second */
    u64 svc_time_ms;        /* historical invariant CPU time consumed (for lag) */
    u64 flags;              /* e.g. LAVD_FLAG_IS_WAKEUP, LAVD_FLAG_WOKEN_BY_HARDIRQ */
} synthetic_task_def;

/* Simulation Task container */
typedef struct sim_task {
    struct task_struct p;
    task_ctx ctx;
    u64 dl_delta;
    u64 greedy_penalty;
    u64 adjusted_runtime;
    u64 allocated_slice_ns;
} sim_task;

#define MAX_TASKS 64

/*
 * =========================================================================
 * DEFAULT SYNTHETIC TASK DATASET
 * You can modify or add your own tasks here!
 * =========================================================================
 */
static synthetic_task_def default_synthetic_tasks[] = {
    {
        .name = "Game_Input_Thread",
        .pid = 1001,
        .weight = 1024,
        .avg_runtime_us = 400,          /* 0.4 ms: very short interactive burst */
        .acc_runtime_us = 400,
        .wait_freq = 120,               /* 120 Hz input poll */
        .wake_freq = 120,               /* wakes render engine */
        .svc_time_ms = 4,               /* low CPU usage */
        .flags = LAVD_FLAG_IS_WAKEUP | LAVD_FLAG_WOKEN_BY_HARDIRQ
    },
    {
        .name = "Audio_DSP_Buffer",
        .pid = 1002,
        .weight = 1024,
        .avg_runtime_us = 250,          /* 0.25 ms: strict low-latency */
        .acc_runtime_us = 250,
        .wait_freq = 250,               /* high wait frequency */
        .wake_freq = 250,
        .svc_time_ms = 5,
        .flags = LAVD_FLAG_IS_WAKEUP | LAVD_FLAG_WOKEN_BY_SOFTIRQ
    },
    {
        .name = "Video_Renderer",
        .pid = 1003,
        .weight = 1024,
        .avg_runtime_us = 3500,         /* 3.5 ms: frame rendering */
        .acc_runtime_us = 3500,
        .wait_freq = 60,                /* 60 FPS */
        .wake_freq = 60,
        .svc_time_ms = 10,              /* on-target CPU usage */
        .flags = LAVD_FLAG_IS_WAKEUP
    },
    {
        .name = "Background_Compiler",
        .pid = 1004,
        .weight = 1024,
        .avg_runtime_us = 20000,        /* 20 ms: CPU-intensive batch */
        .acc_runtime_us = 5000,
        .wait_freq = 2,                 /* rarely yields */
        .wake_freq = 1,
        .svc_time_ms = 25,              /* high CPU usage -> greedy */
        .flags = 0
    },
    {
        .name = "Video_Encoder_Hog",
        .pid = 1005,
        .weight = 1024,
        .avg_runtime_us = 50000,        /* 50 ms: heavy CPU hog */
        .acc_runtime_us = 5000,
        .wait_freq = 0,                 /* pure compute */
        .wake_freq = 0,
        .svc_time_ms = 50,              /* greedy CPU hog -> max penalty */
        .flags = 0
    }
};

/* Comparator to sort tasks by Earliest Virtual Deadline First */
static int compare_vtime(const void *a, const void *b)
{
    const sim_task *ta = *(const sim_task **)a;
    const sim_task *tb = *(const sim_task **)b;

    if (ta->p.scx.dsq_vtime < tb->p.scx.dsq_vtime)
        return -1;
    else if (ta->p.scx.dsq_vtime > tb->p.scx.dsq_vtime)
        return 1;
    return 0;
}

void print_banner(void)
{
    printf("\n");
    printf("========================================================================================\n");
    printf("     LAVD (Latency-criticality Aware Virtual Deadline) Scheduler Simulation             \n");
    printf("========================================================================================\n");
    printf(" System Configuration:\n");
    printf("   - Targeted Latency     : %llu ms\n", LAVD_TARGETED_LATENCY_NS / NSEC_PER_MSEC);
    printf("   - Min / Max Time Slice : %.2f ms / %.2f ms\n", 
           (double)LAVD_SLICE_MIN_NS_DFL / NSEC_PER_MSEC, 
           (double)LAVD_SLICE_MAX_NS_DFL / NSEC_PER_MSEC);
    printf("   - Online CPUs          : %llu\n", nr_cpus_onln);
    printf("   - System Load (util)   : %u%%\n", s2p(sys_stat.cur_util_wall));
    printf("   - Initial Logical Clk  : %llu\n", cur_logical_clk);
    printf("========================================================================================\n\n");
}

int main(int argc, char *argv[])
{
    int num_tasks = sizeof(default_synthetic_tasks) / sizeof(default_synthetic_tasks[0]);
    synthetic_task_def *tasks_to_run = default_synthetic_tasks;
    synthetic_task_def custom_tasks[MAX_TASKS];

    /* Check if CSV file is provided */
    if (argc > 1) {
        const char *csv_path = argv[1];
        FILE *fp = fopen(csv_path, "r");
        if (fp) {
            char line[256];
            int count = 0;
            printf("[INFO] Loading synthetic tasks from '%s'...\n", csv_path);
            /* Skip header if present */
            if (fgets(line, sizeof(line), fp)) {
                if (strstr(line, "name") == NULL) {
                    /* Not header, parse */
                    rewind(fp);
                }
            }
            while (fgets(line, sizeof(line), fp) && count < MAX_TASKS) {
                char name_buf[64] = {0};
                int pid = 0;
                unsigned long weight = 1024, runtime = 1000, acc = 1000, wait = 10, wake = 10, svc = 10, flags = 0;
                int fields = sscanf(line, "%63[^,],%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
                                    name_buf, &pid, &weight, &runtime, &acc, &wait, &wake, &svc, &flags);
                if (fields >= 4) {
                    custom_tasks[count].name = strdup(name_buf);
                    custom_tasks[count].pid = pid ? pid : (2000 + count);
                    custom_tasks[count].weight = weight ? weight : 1024;
                    custom_tasks[count].avg_runtime_us = runtime;
                    custom_tasks[count].acc_runtime_us = acc;
                    custom_tasks[count].wait_freq = wait;
                    custom_tasks[count].wake_freq = wake;
                    custom_tasks[count].svc_time_ms = svc;
                    custom_tasks[count].flags = flags;
                    count++;
                }
            }
            fclose(fp);
            if (count > 0) {
                tasks_to_run = custom_tasks;
                num_tasks = count;
                printf("[INFO] Loaded %d synthetic tasks from file.\n", count);
            }
        } else {
            printf("[WARN] Could not open file '%s', using default synthetic tasks.\n", csv_path);
        }
    }

    print_banner();

    sim_task task_pool[MAX_TASKS];
    sim_task *queue[MAX_TASKS];

    struct cpu_ctx default_cpu = {
        .cpu_id = 0,
        .effective_capacity = 1024,
        .max_capacity = 1024,
        .nr_pinned_tasks = 0,
        .cur_util_wall = p2s(50),
        .avg_util_wall = p2s(50)
    };

    sys_stat.nr_queued_task = num_tasks;

    /* 1. Initialize tasks and compute inputs */
    printf("--- [STEP 1] SYNTHETIC TASKS INPUT DATA ---\n");
    printf("%-5s | %-22s | %-6s | %-12s | %-10s | %-10s | %-10s\n",
           "PID", "Task Name", "Weight", "Avg Run (us)", "Wait Freq", "Wake Freq", "Flags");
    printf("----------------------------------------------------------------------------------------\n");

    u64 total_lat_cri = 0;

    for (int i = 0; i < num_tasks; i++) {
        synthetic_task_def *def = &tasks_to_run[i];
        sim_task *st = &task_pool[i];
        memset(st, 0, sizeof(*st));

        st->p.pid = def->pid;
        strncpy(st->p.comm, def->name, TASK_COMM_LEN);
        st->p.scx.weight = def->weight ? def->weight : 1024;

        st->ctx.pid = def->pid;
        st->ctx.flags = def->flags;
        st->ctx.avg_runtime_invr = def->avg_runtime_us * NSEC_PER_USEC;
        st->ctx.avg_runtime_wall = def->avg_runtime_us * NSEC_PER_USEC;
        st->ctx.acc_runtime_invr = def->acc_runtime_us * NSEC_PER_USEC;
        st->ctx.acc_runtime_wall = def->acc_runtime_us * NSEC_PER_USEC;
        st->ctx.wait_freq = def->wait_freq;
        st->ctx.wake_freq = def->wake_freq;
        st->ctx.run_freq = max(def->wait_freq, 1);
        st->ctx.svc_time_iwgt = def->svc_time_ms * NSEC_PER_MSEC;

        queue[i] = st;

        printf("%-5d | %-22s | %-6llu | %-12llu | %-10llu | %-10llu | 0x%-8llx\n",
               st->p.pid, st->p.comm, st->p.scx.weight, 
               def->avg_runtime_us, st->ctx.wait_freq, st->ctx.wake_freq, st->ctx.flags);
    }
    printf("----------------------------------------------------------------------------------------\n\n");

    /* 2. Compute Latency Criticality and Deadlines */
    for (int i = 0; i < num_tasks; i++) {
        sim_task *st = &task_pool[i];
        calc_lat_cri(&st->p, &st->ctx);
        total_lat_cri += st->ctx.lat_cri;
    }
    sys_stat.avg_lat_cri = total_lat_cri / num_tasks;

    for (int i = 0; i < num_tasks; i++) {
        sim_task *st = &task_pool[i];
        st->greedy_penalty = calc_greedy_penalty(&st->p, &st->ctx);
        st->adjusted_runtime = calc_adjusted_runtime(&st->ctx);
        st->dl_delta = calc_virtual_deadline_delta(&st->p, &st->ctx);
        st->p.scx.dsq_vtime = calc_when_to_run(&st->p, &st->ctx);
        st->allocated_slice_ns = calc_time_slice(&st->ctx, &default_cpu);
    }

    printf("--- [STEP 2] LAVD METRIC ANALYSIS & VIRTUAL DEADLINE COMPUTATION ---\n");
    printf("%-22s | %-10s | %-14s | %-14s | %-16s | %-16s\n",
           "Task Name", "lat_cri", "Greedy Penalty", "Adj Runtime", "Deadline Delta", "Virtual Deadline");
    printf("--------------------------------------------------------------------------------------------------------\n");

    for (int i = 0; i < num_tasks; i++) {
        sim_task *st = &task_pool[i];
        double penalty_pct = (double)st->greedy_penalty / LAVD_SCALE * 100.0;
        printf("%-22s | %-10u | %6.1f%% (%-4llu) | %8.2f ms    | %12llu   | %-16llu\n",
               st->p.comm,
               st->ctx.lat_cri,
               penalty_pct, st->greedy_penalty,
               (double)st->adjusted_runtime / NSEC_PER_MSEC,
               st->dl_delta,
               st->p.scx.dsq_vtime);
    }
    printf("--------------------------------------------------------------------------------------------------------\n\n");

    /* 3. Enqueue and Sort: Dispatch Queue (DSQ) Order */
    qsort(queue, num_tasks, sizeof(sim_task *), compare_vtime);

    /* 4. Display Requested Outputs: Queue Order and Allocated Time */
    printf("========================================================================================================\n");
    printf("                             FINAL OUTPUT: QUEUE ORDER & ALLOCATED TIME                                 \n");
    printf("========================================================================================================\n");
    printf(" %-5s | %-5s | %-22s | %-18s | %-18s | %-15s\n",
           "ORDER", "PID", "TASK NAME", "VIRTUAL DEADLINE", "ALLOCATED TIME", "BEHAVIOR");
    printf("--------------------------------------------------------------------------------------------------------\n");

    for (int i = 0; i < num_tasks; i++) {
        sim_task *st = queue[i];
        double slice_ms = (double)st->allocated_slice_ns / NSEC_PER_MSEC;
        const char *behavior = "Normal";

        if (st->ctx.flags & LAVD_FLAG_SLICE_BOOST)
            behavior = "Slice-Boosted";
        else if (st->ctx.lat_cri > sys_stat.avg_lat_cri)
            behavior = "Latency-Critical";
        else if (st->ctx.flags & LAVD_FLAG_IS_GREEDY)
            behavior = "Greedy-Penalty";

        printf("   #%-3d | %-5d | %-22s | %-18llu | %6.2f ms (%7llu us) | %-15s\n",
               i + 1,
               st->p.pid,
               st->p.comm,
               st->p.scx.dsq_vtime,
               slice_ms,
               st->allocated_slice_ns / NSEC_PER_USEC,
               behavior);
    }
    printf("========================================================================================================\n\n");

    /* 5. Simulation Execution Trace */
    printf("--- [STEP 3] DISPATCH SIMULATION & LOGICAL CLOCK ADVANCEMENT ---\n");
    for (int i = 0; i < num_tasks; i++) {
        sim_task *st = queue[i];
        u64 prev_clk = cur_logical_clk;
        advance_cur_logical_clk(&st->p);
        printf(" [Dispatch #%d] Running: '%s' (PID %d)\n", i + 1, st->p.comm, st->p.pid);
        printf("              -> Allocated Time Slice: %.2f ms\n", (double)st->allocated_slice_ns / NSEC_PER_MSEC);
        printf("              -> Virtual Deadline    : %llu\n", st->p.scx.dsq_vtime);
        printf("              -> Clock Advance       : %llu -> %llu\n", prev_clk, cur_logical_clk);
        printf("----------------------------------------------------------------------------------------\n");
    }

    printf("\n[SUCCESS] Simulation completed successfully.\n");
    return 0;
}
