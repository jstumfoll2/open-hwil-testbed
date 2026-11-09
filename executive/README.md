# executive/

Reserved for the compiled real-time executive.

M1 runs the frame loop in Python (`hwil/executive.py`), which is enough for SIL
and for a first timing baseline. This folder will hold:

- **M4:** a small C++ fixed-rate loop (`clock_nanosleep` on absolute deadlines,
  `SCHED_FIFO`, `mlockall`, pinned to an isolated CPU) linking the JSBSim C++
  library, used if the Python loop cannot hold the frame budget on PREEMPT_RT.
- **M6 (optional):** a NASA Trick simulation wrapping the same models, for Monte
  Carlo runs of wind and sensor-noise cases.

Both will read the same scenario files and write the same `log.csv`,
`timing.csv` and `manifest.json`, so the pytest harness and RTAMT requirements
grade them unchanged.
