#!/usr/bin/env python3
"""
LAVD (Latency-criticality Aware Virtual Deadline) Scheduler Simulation
======================================================================
Faithfully models the LAVD scheduler algorithm extracted from scx_lavd:
1. Calculates context-aware Latency Criticality (lat_cri)
2. Calculates Fairness & Greedy Penalty based on task lag
3. Calculates Virtual Deadline (calc_when_to_run -> dsq_vtime)
4. Calculates Allocated Time Slice (calc_time_slice)
5. Enqueues and sorts tasks in Dispatch Queue (DSQ) order:
   Output = Order of each task in queue & each task's allocated time.
"""

import sys
import math
import csv
from dataclasses import dataclass, field
from typing import List, Optional

# Constants matching lavd-bpf.c / lavd_sim.h
LAVD_SHIFT = 10
LAVD_SCALE = 1 << LAVD_SHIFT  # 1024

NSEC_PER_USEC = 1000
NSEC_PER_MSEC = 1000 * NSEC_PER_USEC
LAVD_TIME_ONE_SEC = 1000 * NSEC_PER_MSEC

LAVD_TARGETED_LATENCY_NS = 10 * NSEC_PER_MSEC  # 10 ms
LAVD_SLICE_MIN_NS_DFL = 500 * NSEC_PER_USEC   # 0.5 ms
LAVD_SLICE_MAX_NS_DFL = 5 * NSEC_PER_MSEC     # 5.0 ms
LAVD_SLICE_BOOST_BONUS = LAVD_SLICE_MIN_NS_DFL
LAVD_SLICE_BOOST_MAX = 500 * NSEC_PER_MSEC
LAVD_ACC_RUNTIME_MAX = LAVD_SLICE_MAX_NS_DFL
LAVD_TASK_LAG_MAX = 500 * NSEC_PER_MSEC
LAVD_DL_COMPETE_WINDOW = (300 * NSEC_PER_MSEC) >> 16  # 4577

LAVD_LC_FREQ_MAX = 100000
LAVD_LC_RUNTIME_MAX = LAVD_TIME_ONE_SEC
LAVD_LC_WEIGHT_BOOST_REGULAR = 128
LAVD_LC_WEIGHT_BOOST_MEDIUM = 256
LAVD_LC_WEIGHT_BOOST_HIGH = 512
LAVD_LC_WEIGHT_BOOST_HIGHEST = 1024
LAVD_LC_GREEDY_SHIFT = 1
LAVD_SLICE_BOOST_UTIL_WALL = int((95 << LAVD_SHIFT) / 100)  # 972

# Flags
FLAG_NEED_LOCK_BOOST = 1 << 1
FLAG_IS_GREEDY       = 1 << 2
FLAG_IS_AFFINITIZED  = 1 << 3
FLAG_IS_WAKEUP       = 1 << 4
FLAG_IS_SYNC_WAKEUP  = 1 << 5
FLAG_SLICE_BOOST     = 1 << 8
FLAG_KSOFTIRQD       = 1 << 10
FLAG_WOKEN_BY_RT_DL  = 1 << 11
FLAG_WOKEN_BY_HARDIRQ= 1 << 12
FLAG_WOKEN_BY_SOFTIRQ= 1 << 13


@dataclass
class SyntheticTask:
    name: str
    pid: int
    weight: int = 1024          # Normal priority (nice 0)
    avg_runtime_us: int = 1000  # Microseconds per execution
    acc_runtime_us: int = 1000  # Accumulated since last sleep
    wait_freq: int = 10         # Waits per second (consumer frequency)
    wake_freq: int = 10         # Wakes per second (producer frequency)
    svc_time_ms: int = 10       # CPU time consumed historically
    flags: int = 0
    is_kernel: bool = False

    # Calculated metrics
    lat_cri: int = 0
    greedy_penalty: int = 1024
    adjusted_runtime_ns: int = 0
    dl_delta: int = 0
    virtual_deadline: int = 0
    allocated_slice_ns: int = 0
    behavior_note: str = ""


class LAVDSimulator:
    def __init__(self, nr_cpus: int = 4, sys_util_pct: int = 50):
        self.nr_cpus = nr_cpus
        self.cur_util_wall = int((sys_util_pct << LAVD_SHIFT) / 100)
        self.avg_util_wall = self.cur_util_wall
        self.slice_wall = LAVD_SLICE_MAX_NS_DFL
        self.avg_svc_time_iwgt = 10 * NSEC_PER_MSEC
        self.cur_logical_clk = LAVD_DL_COMPETE_WINDOW
        self.no_slice_boost = False

    def can_boost_slice(self) -> bool:
        return self.avg_util_wall < LAVD_SLICE_BOOST_UTIL_WALL

    @staticmethod
    def log2_u64(v: int) -> int:
        if v <= 0:
            return 0
        return v.bit_length() - 1

    def calc_weight_factor(self, task: SyntheticTask) -> int:
        boost = 1
        if task.flags & FLAG_IS_WAKEUP:
            boost += LAVD_LC_WEIGHT_BOOST_REGULAR
        if task.flags & FLAG_IS_SYNC_WAKEUP:
            boost += LAVD_LC_WEIGHT_BOOST_REGULAR
        if task.flags & FLAG_WOKEN_BY_HARDIRQ:
            boost += LAVD_LC_WEIGHT_BOOST_HIGHEST
        elif task.flags & FLAG_WOKEN_BY_SOFTIRQ:
            boost += LAVD_LC_WEIGHT_BOOST_HIGH
        if task.flags & FLAG_WOKEN_BY_RT_DL:
            boost += LAVD_LC_WEIGHT_BOOST_HIGH
        if task.is_kernel:
            boost += LAVD_LC_WEIGHT_BOOST_MEDIUM
        if task.flags & FLAG_IS_AFFINITIZED:
            boost += LAVD_LC_WEIGHT_BOOST_REGULAR
        if task.flags & FLAG_NEED_LOCK_BOOST:
            boost += LAVD_LC_WEIGHT_BOOST_REGULAR

        return task.weight * boost + 1

    def calc_lat_cri(self, task: SyntheticTask) -> int:
        wait_ft = min(task.wait_freq, LAVD_LC_FREQ_MAX) + 1
        wake_ft = min(task.wake_freq, LAVD_LC_FREQ_MAX) + 1

        avg_runtime_invr = task.avg_runtime_us * NSEC_PER_USEC
        if LAVD_LC_RUNTIME_MAX > avg_runtime_invr:
            delta = LAVD_LC_RUNTIME_MAX - avg_runtime_invr
            runtime_ft = (delta // LAVD_SLICE_MIN_NS_DFL) + 1
        else:
            runtime_ft = 1

        weight_ft = self.calc_weight_factor(task)

        log_wwf = self.log2_u64(wait_ft * wake_ft)
        lat_cri = log_wwf + self.log2_u64(runtime_ft * weight_ft)
        lat_cri = lat_cri * lat_cri  # Amplified differentiation
        task.lat_cri = max(lat_cri, 1)
        return task.lat_cri

    def calc_greedy_penalty(self, task: SyntheticTask) -> int:
        svc_time_iwgt = task.svc_time_ms * NSEC_PER_MSEC
        lag = self.avg_svc_time_iwgt - svc_time_iwgt
        lag_max = (LAVD_TASK_LAG_MAX * 1024) // (task.weight if task.weight else 1024)

        if lag >= 0:
            task.flags &= ~FLAG_IS_GREEDY
            if lag > lag_max:
                lag = lag_max
        else:
            task.flags |= FLAG_IS_GREEDY
            if lag < -lag_max:
                lag = -lag_max

        penalty = ((-lag + lag_max) << LAVD_SHIFT) // lag_max
        penalty = LAVD_SCALE + (penalty >> LAVD_LC_GREEDY_SHIFT)
        task.greedy_penalty = penalty
        return penalty

    def calc_adjusted_runtime(self, task: SyntheticTask) -> int:
        acc_runtime_invr = task.acc_runtime_us * NSEC_PER_USEC
        runtime = LAVD_ACC_RUNTIME_MAX + min(acc_runtime_invr, LAVD_ACC_RUNTIME_MAX)
        task.adjusted_runtime_ns = runtime
        return runtime

    def calc_virtual_deadline_delta(self, task: SyntheticTask) -> int:
        deadline = (task.adjusted_runtime_ns * task.greedy_penalty) // task.lat_cri
        task.dl_delta = deadline >> LAVD_SHIFT
        return task.dl_delta

    def calc_compete_window(self, nr_queued: int) -> int:
        if nr_queued > self.nr_cpus:
            return (LAVD_DL_COMPETE_WINDOW * self.nr_cpus) // nr_queued
        return LAVD_DL_COMPETE_WINDOW

    def calc_when_to_run(self, task: SyntheticTask, nr_queued: int) -> int:
        compete_window = self.calc_compete_window(nr_queued)
        clc = self.cur_logical_clk - compete_window
        task.virtual_deadline = clc + task.dl_delta
        return task.virtual_deadline

    def calc_time_slice(self, task: SyntheticTask, avg_lat_cri: int) -> int:
        avg_runtime_wall = task.avg_runtime_us * NSEC_PER_USEC

        if not self.no_slice_boost and (avg_runtime_wall >= self.slice_wall):
            if self.can_boost_slice():
                s = avg_runtime_wall + LAVD_SLICE_BOOST_BONUS
                task.allocated_slice_ns = max(LAVD_SLICE_MIN_NS_DFL, min(s, LAVD_SLICE_BOOST_MAX))
                task.flags |= FLAG_SLICE_BOOST
                task.behavior_note = "Slice-Boosted"
                return task.allocated_slice_ns

            if task.lat_cri > avg_lat_cri:
                b = (self.slice_wall * task.lat_cri) // (avg_lat_cri + 1)
                s = self.slice_wall + b
                task.allocated_slice_ns = max(LAVD_SLICE_MIN_NS_DFL, min(s, min(avg_runtime_wall, self.slice_wall * 2)))
                task.flags |= FLAG_SLICE_BOOST
                task.behavior_note = "Slice-Boosted (LC)"
                return task.allocated_slice_ns

        task.allocated_slice_ns = self.slice_wall
        task.flags &= ~FLAG_SLICE_BOOST
        if task.lat_cri > avg_lat_cri:
            task.behavior_note = "Latency-Critical"
        elif task.flags & FLAG_IS_GREEDY:
            task.behavior_note = "Greedy-Penalty"
        else:
            task.behavior_note = "Normal"
        return task.allocated_slice_ns

    def advance_cur_logical_clk(self, task: SyntheticTask, nr_queued: int) -> None:
        vlc = task.virtual_deadline
        clc = self.cur_logical_clk
        if vlc <= clc:
            return
        delta = (vlc - clc) // max(nr_queued, 1)
        self.cur_logical_clk = clc + delta

    def simulate(self, tasks: List[SyntheticTask]):
        nr_queued = len(tasks)

        # 1. Latency criticality
        total_lat_cri = 0
        for t in tasks:
            total_lat_cri += self.calc_lat_cri(t)
        avg_lat_cri = total_lat_cri // max(nr_queued, 1)

        # 2. Deadlines and time slices
        for t in tasks:
            self.calc_greedy_penalty(t)
            self.calc_adjusted_runtime(t)
            self.calc_virtual_deadline_delta(t)
            self.calc_when_to_run(t, nr_queued)
            self.calc_time_slice(t, avg_lat_cri)

        # 3. Queue ordering by Earliest Virtual Deadline First
        ordered_queue = sorted(tasks, key=lambda x: x.virtual_deadline)

        # Display results
        self.print_results(tasks, ordered_queue)

    def print_results(self, original_tasks: List[SyntheticTask], ordered_queue: List[SyntheticTask]):
        print("\n" + "=" * 96)
        print("     LAVD (Latency-criticality Aware Virtual Deadline) Scheduler Simulation")
        print("=" * 96)
        print(f" System: {self.nr_cpus} CPUs, Targeted Latency = {LAVD_TARGETED_LATENCY_NS // NSEC_PER_MSEC} ms, "
              f"Default Slice = {self.slice_wall / NSEC_PER_MSEC:.2f} ms, Initial Clock = {self.cur_logical_clk}")
        print("=" * 96 + "\n")

        print("--- [STEP 1] SYNTHETIC TASKS INPUT DATA ---")
        print(f"{'PID':<5} | {'Task Name':<22} | {'Weight':<6} | {'Avg Run (us)':<12} | {'Wait Freq':<10} | {'Wake Freq':<10} | {'Flags':<8}")
        print("-" * 96)
        for t in original_tasks:
            print(f"{t.pid:<5} | {t.name:<22} | {t.weight:<6} | {t.avg_runtime_us:<12} | {t.wait_freq:<10} | {t.wake_freq:<10} | 0x{t.flags:<6x}")
        print("-" * 96 + "\n")

        print("--- [STEP 2] LAVD METRIC ANALYSIS & VIRTUAL DEADLINE COMPUTATION ---")
        print(f"{'Task Name':<22} | {'lat_cri':<8} | {'Greedy Penalty':<14} | {'Adj Runtime':<12} | {'Deadline Delta':<14} | {'Virtual Deadline':<16}")
        print("-" * 96)
        for t in original_tasks:
            penalty_pct = (t.greedy_penalty / LAVD_SCALE) * 100.0
            adj_ms = t.adjusted_runtime_ns / NSEC_PER_MSEC
            print(f"{t.name:<22} | {t.lat_cri:<8} | {penalty_pct:5.1f}% ({t.greedy_penalty}) | {adj_ms:7.2f} ms   | {t.dl_delta:<14} | {t.virtual_deadline:<16}")
        print("-" * 96 + "\n")

        print("=" * 96)
        print("                             FINAL OUTPUT: QUEUE ORDER & ALLOCATED TIME")
        print("=" * 96)
        print(f" {'ORDER':<5} | {'PID':<5} | {'TASK NAME':<22} | {'VIRTUAL DEADLINE':<18} | {'ALLOCATED TIME':<18} | {'BEHAVIOR':<16}")
        print("-" * 96)
        for idx, t in enumerate(ordered_queue, 1):
            slice_ms = t.allocated_slice_ns / NSEC_PER_MSEC
            slice_us = t.allocated_slice_ns // NSEC_PER_USEC
            print(f"   #{idx:<3} | {t.pid:<5} | {t.name:<22} | {t.virtual_deadline:<18} | {slice_ms:6.2f} ms ({slice_us:5} us) | {t.behavior_note:<16}")
        print("=" * 96 + "\n")

        print("--- [STEP 3] DISPATCH SIMULATION & LOGICAL CLOCK ADVANCEMENT ---")
        for idx, t in enumerate(ordered_queue, 1):
            prev_clk = self.cur_logical_clk
            self.advance_cur_logical_clk(t, len(ordered_queue))
            slice_ms = t.allocated_slice_ns / NSEC_PER_MSEC
            print(f" [Dispatch #{idx}] Running: '{t.name}' (PID {t.pid})")
            print(f"              -> Allocated Time Slice: {slice_ms:.2f} ms")
            print(f"              -> Virtual Deadline    : {t.virtual_deadline}")
            print(f"              -> Clock Advance       : {prev_clk} -> {self.cur_logical_clk}")
            print("-" * 80)
        print("\n[SUCCESS] Simulation completed successfully.\n")


def load_tasks_from_csv(csv_path: str) -> List[SyntheticTask]:
    tasks = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            tasks.append(SyntheticTask(
                name=row['name'].strip(),
                pid=int(row.get('pid', 1000 + len(tasks))),
                weight=int(row.get('weight', 1024)),
                avg_runtime_us=int(row.get('avg_runtime_us', 1000)),
                acc_runtime_us=int(row.get('acc_runtime_us', 1000)),
                wait_freq=int(row.get('wait_freq', 10)),
                wake_freq=int(row.get('wake_freq', 10)),
                svc_time_ms=int(row.get('svc_time_ms', 10)),
                flags=int(row.get('flags', 0))
            ))
    return tasks


if __name__ == '__main__':
    sim = LAVDSimulator()

    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
        print(f"[INFO] Loading synthetic tasks from {csv_file}")
        tasks = load_tasks_from_csv(csv_file)
    else:
        # Default synthetic tasks
        tasks = [
            SyntheticTask("Game_Input_Thread", 1001, 1024, avg_runtime_us=400, acc_runtime_us=400,
                          wait_freq=120, wake_freq=120, svc_time_ms=4,
                          flags=FLAG_IS_WAKEUP | FLAG_WOKEN_BY_HARDIRQ),
            SyntheticTask("Audio_DSP_Buffer", 1002, 1024, avg_runtime_us=250, acc_runtime_us=250,
                          wait_freq=250, wake_freq=250, svc_time_ms=5,
                          flags=FLAG_IS_WAKEUP | FLAG_WOKEN_BY_SOFTIRQ),
            SyntheticTask("Video_Renderer", 1003, 1024, avg_runtime_us=3500, acc_runtime_us=3500,
                          wait_freq=60, wake_freq=60, svc_time_ms=10,
                          flags=FLAG_IS_WAKEUP),
            SyntheticTask("Background_Compiler", 1004, 1024, avg_runtime_us=20000, acc_runtime_us=5000,
                          wait_freq=2, wake_freq=1, svc_time_ms=25, flags=0),
            SyntheticTask("Video_Encoder_Hog", 1005, 1024, avg_runtime_us=50000, acc_runtime_us=5000,
                          wait_freq=0, wake_freq=0, svc_time_ms=50, flags=0),
        ]

    sim.simulate(tasks)
