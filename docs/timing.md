# Timing

`hwil timing` compares the frame timing of one or more run folders side by side.
It reads each run's `timing.csv` and `manifest.json`.

```bash
hwil run scenarios/long_hold_60m.yaml --executive cpp    --clock realtime --rt-cpu 2 --out runs
hwil run scenarios/long_hold_60m.yaml --executive python --clock realtime --rt-cpu 2 --out runs
hwil timing runs/<cpp run> runs/<python run> runs/<px4 run> --out timing_report.html
```

The command writes `timing_report.html`, which has a table and log-scale
histograms, and `timing_report.json`.

## Definitions

Each frame has period P = 1 / rate (4 ms at 250 Hz).

| Column | Meaning |
|---|---|
| `late_us` | Actual frame start minus scheduled start. Realtime clock only. |
| `compute_us` | Frame work time, excluding the PX4 lockstep wait. |
| `io_wait_us` | PX4 round trip, from HIL_SENSOR sent to HIL_ACTUATOR_CONTROLS received. |
| `overrun` | 1 if the frame finished after the next frame's scheduled start. |

## Results so far (stock kernel, shared cloud VM)

The runs are 4-minute realtime soaks of `long_hold_60m`: 62,727 frames at 250 Hz,
pinned to CPU 2, without `SCHED_FIFO`. Both executives sleep until 150 us before
each deadline and then spin. The PX4 row is a `waypoint_square` SITL run in afap
mode. All times are in microseconds.

| Run | Overruns | Lateness p50 / p99 / p99.9 / max | Compute p50 / p99 / p99.9 / max |
|---|---|---|---|
| C++ executive | 35 | 0 / 341 / 2,313 / 28,458 | 48 / 113 / 814 / 7,995 |
| Python executive | 87 | 1 / 299 / 2,728 / 22,413 | 324 / 787 / 2,806 / 23,689 |
| Python + PX4 SITL | (afap) | n/a | 373 / 844 / 2,003 / 113,132 |

The PX4 round trip, from HIL_SENSOR to HIL_ACTUATOR_CONTROLS, was 376 us at p50,
1,209 us at p99 and 3,695 us at p99.9. The worst frame waited 1.0 s, during PX4
start-up.

What this shows:

- **The C++ loop needs much less time per frame.** Its compute time is about
  7x lower at p50 and p99. That leaves about 3.9 ms of the 4 ms frame free
  for a PX4 lockstep wait or a higher frame rate.
- **The VM, not the code, causes the overruns.** Both executives have
  multi-millisecond stalls in start lateness, which happen when the hypervisor
  deschedules the vCPU. A faster loop cannot remove those. That is the job of an
  isolated core on a PREEMPT_RT kernel.
- **PX4 lockstep fits in the frame at p99**, at about 1.2 ms. Its tail does not
  fit, so on hardware the PX4 wait needs a timeout policy before it can run at a
  fixed rate.

The C++ loop also sets the timer slack to 1 ns. `--spin-us 0` switches it to a
pure `clock_nanosleep` wait. On this VM a pure wait woke 130 us late at the
median.

These numbers come from a shared VM, so they show what the software costs, not
what a quiet machine can hold. The PREEMPT_RT measurements on a dedicated Linux
PC are M7.
