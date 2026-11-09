# Project brief: Open HWIL Testbed

**Owner:** Jason Stumfoll · **Started:** 2026-11-09

## 1. Purpose

Build a small, fully open-source, **unclassified hardware-in-the-loop testbed** on a home bench: a real-time simulation running on Linux, with a real embedded flight controller in the loop. It serves three goals:

1. **Learning:** hands-on skill with open real-time simulation tools (NASA Trick, PREEMPT_RT, JSBSim, PX4, FMI).
2. **Portfolio:** a public GitHub repo and write-up showing HWIL lab engineering.
3. **Talks:** material for a conference paper or blog on *lab engineering* (automation, timing, test infrastructure, AI-assisted testing).

## 2. Ground rules (non-negotiable)

- **Civilian and generic.** The vehicle is a small fixed-wing UAV or quadcopter using public models. No weapons, guidance-to-target, seeker, or engagement content.
- **No employer material.** No code, data, models, documents, or know-how specific to any employer or program. Everything comes from public sources and is written fresh.
- **Public by design.** Assume everything is published. Get employer outside-activity / public-release approval before publishing if your company requires it.
- **Export control:** stay with public, general-purpose material. When in doubt, leave it out.

## 3. Scope

**In scope**
- A real-time simulation executive on PREEMPT_RT Linux running a 6-DOF flight model (JSBSim) at a fixed rate (target 250–1000 Hz).
- A real flight controller (Pixhawk running PX4 in HITL mode, or an STM32 board with simple autopilot code) connected over serial/USB (MAVLink) or CAN.
- Scenario files (YAML/JSON) defining initial conditions, wind and faults; a run manifest per run (versions, config hash, seed).
- Automated test harness (pytest) with requirement-style checks graded automatically (for example, with RTAMT signal temporal logic monitors).
- Data logging, plotting and an HTML run report; CI on the software-in-the-loop path (GitHub Actions).
- Timing instrumentation: frame overruns, jitter, end-to-end latency.
- Phase-2 add-ons: anomaly detection on run logs, Monte Carlo batch runs in SIL, an LLM helper that drafts test cases for human review.

**Out of scope**
- Weapon, targeting or guidance-to-intercept functions.
- Certified or safety-critical use; flying real aircraft with this setup.
- Expensive hardware (target budget under ~$1,000).

## 4. Architecture

See [architecture.md](architecture.md) for the diagram as built.

## 5. Suggested stack

| Layer | Choice | Notes |
|---|---|---|
| OS | Ubuntu LTS with PREEMPT_RT kernel | Use `isolcpus`, set IRQ affinity, run `cyclictest` to get a baseline |
| Executive | Start with a simple C++/Python fixed-rate loop; move to **NASA Trick** in milestone 5 | Trick adds Monte Carlo, checkpointing and data recording |
| Flight model | **JSBSim** (C++ library or Python bindings) | Use a public UAV/aircraft model from the JSBSim repo |
| Flight controller | **PX4** on Pixhawk (HITL) | PX4 SITL first, then the same scenarios in HITL |
| Interface | MAVLink (pymavlink / C library) | HIL_SENSOR, HIL_GPS in; HIL_ACTUATOR_CONTROLS out |
| Model exchange (later) | FMI / FMPy | Swap in an actuator or sensor model as an FMU |
| Tests | pytest, RTAMT, NumPy/pandas, Plotly | Automatic pass/fail with numeric margins |
| CI | GitHub Actions | SIL tests on every push; HITL runs on the bench |
| Docs | Markdown + Mermaid in the repo | |

## 6. Milestones

| # | Milestone | Done when |
|---|---|---|
| M0 | Repo, license (Apache-2.0), README, ground rules | Public repo exists with this brief |
| M1 | SIL loop: JSBSim + PX4 SITL, scripted takeoff/loiter/land | One command runs a scenario and produces plots |
| M2 | Real-time baseline on PREEMPT_RT | cyclictest and in-loop jitter report; frame overrun counter in logs |
| M3 | HITL with a real Pixhawk | Same scenario passes in SIL and HITL; latency measured end to end |
| M4 | Test harness | 10+ scenario tests with STL-graded requirements and an HTML report per run |
| M5 | Executive upgrade to NASA Trick (optional) | Trick runs the loop with Monte Carlo of wind/sensor-noise cases in SIL |
| M6 | AI-in-test add-on | Anomaly detection over run logs, plus LLM-drafted test cases reviewed by a human |
| M7 | Write-up | Blog post or paper draft on lab engineering lessons |

## 7. Hardware shopping list (approximate)

| Item | Est. cost |
|---|---|
| Pixhawk-class flight controller (e.g., Pixhawk 6C) | $150–250 |
| USB-serial / CAN adapter, cables | $30–60 |
| Linux PC (existing machine or mini PC) | $0–500 |
| Optional: STM32 Nucleo board for a custom-autopilot variant | $20–30 |

## 8. Success measures

- One command reproduces any run from its manifest.
- SIL and HITL results agree within stated tolerances for the M3 scenarios.
- Timing report shows the frame budget is met (no overruns in a 30-minute run).
- Public repo with docs good enough for someone else to rebuild the bench.

## 9. References

- JSBSim: https://jsbsim.org
- PX4 HITL: https://docs.px4.io/main/en/simulation/hitl.html
- NASA Trick: https://github.com/nasa/trick
- RTAMT: https://github.com/nickovic/rtamt
