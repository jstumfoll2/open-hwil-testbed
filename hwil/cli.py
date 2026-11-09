"""Command line: ``hwil run``, ``hwil reproduce``, ``hwil validate``."""

from __future__ import annotations

import argparse
import json
import os
import sys

from . import scenario as scn
from .run import reproduce, run_scenario


def _cmd_run(a) -> int:
    sc = scn.load(a.scenario)
    if a.seed is not None:
        sc["seed"] = a.seed
    opts = {"controller": a.controller, "clock": a.clock, "plots": not a.no_plots,
            "px4_sim_port": a.px4_sim_port, "px4_gcs": a.px4_gcs,
            "px4_serial": a.px4_serial, "px4_baud": a.px4_baud,
            "rt_priority": a.rt_priority, "rt_cpu": a.rt_cpu}
    run_dir, man = run_scenario(sc, opts, a.out, scenario_file=a.scenario)
    res = man["results"]
    tim = res["timing"]
    print(f"run dir     : {run_dir}")
    print(f"config hash : {man['config_hash']}")
    print(f"sim time    : {res['sim_time_s']:.2f} s in {res['wall_time_s']:.2f} s wall, landed={res['landed']}")
    print(f"timing      : {tim['frames']} frames, {tim['overruns']} overruns, "
          f"compute p99 {tim['compute_us_p99']:.0f} us of {tim['period_us']:.0f} us")
    if res["aborted"]:
        print(f"ABORTED     : {res['aborted']}")
    for r in res["requirements"]:
        verdict = "PASS" if r["passed"] else ("VACUOUS" if r.get("vacuous") else "FAIL")
        print(f"{verdict} {r['id']}: robustness {r['robustness']:.3f}  {r['text']}")
    print(f"report      : {run_dir / 'report.html'}")
    return 0 if res["all_requirements_passed"] else 1


def _cmd_reproduce(a) -> int:
    run_dir, _new, cmp = reproduce(a.manifest, a.out)
    print(f"re-run dir : {run_dir}")
    print(json.dumps(cmp, indent=2))
    if cmp["deterministic_expected"] and not cmp["log_match"]:
        print("MISMATCH: a deterministic run produced a different log", file=sys.stderr)
        return 2
    return 0


def _cmd_suite(a) -> int:
    from .suite import run_suite
    opts = {"controller": a.controller, "plots": not a.no_plots}
    suite_dir, summary = run_suite(a.scenarios, opts, a.out, jobs=a.jobs, px4_dir=a.px4_dir)
    for r in summary["results"]:
        if "error" in r:
            print(f"ERROR {r['scenario']}: {r['error']}")
            continue
        n_ok = sum(q["passed"] for q in r["requirements"])
        print(f"{'PASS' if r['passed'] else 'FAIL'} {r['scenario']:<22} {n_ok}/{len(r['requirements'])} requirements"
              + (f"  ({r['aborted']})" if r.get("aborted") else ""))
    print(f"{summary['passed']}/{summary['scenarios']} scenarios passed; report: {suite_dir / 'suite.html'}")
    return 0 if summary["passed"] == summary["scenarios"] else 1


def _cmd_montecarlo(a) -> int:
    from .montecarlo import run_montecarlo
    mc_dir, s = run_montecarlo(a.config, a.out, jobs=a.jobs, controller=a.controller,
                               px4_dir=a.px4_dir, runs=a.runs)
    lo, hi = s["pass_rate_ci95"]
    print(f"{s['passed']}/{s['runs']} runs passed ({100 * s['pass_rate']:.1f} %, 95 % CI {100 * lo:.1f}-{100 * hi:.1f} %), "
          f"{s['aborted']} aborted, {s['errors']} errors")
    for rid, r in s["requirements"].items():
        m = r["robustness_min"]
        print(f"  {rid:<14} {r['passed']}/{r['runs']}  min margin {'n/a' if m is None else f'{m:.3f}'}")
    print(f"report: {mc_dir / 'montecarlo.html'}")
    return 0


def _cmd_validate(a) -> int:
    for path in a.scenarios:
        scn.load(path)
        print(f"ok  {path}")
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="hwil", description="Open HWIL testbed")
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="run one scenario")
    r.add_argument("scenario")
    r.add_argument("--controller", choices=["builtin", "px4"], default="builtin")
    r.add_argument("--clock", choices=["afap", "realtime"], default="afap",
                   help="afap = as fast as possible (lockstep with PX4); realtime = wall-clock frames")
    r.add_argument("--seed", type=int, help="override the scenario seed")
    r.add_argument("--out", default="runs")
    r.add_argument("--no-plots", action="store_true")
    r.add_argument("--px4-sim-port", type=int, default=4560)
    r.add_argument("--px4-gcs", default="udpin:0.0.0.0:14540")
    r.add_argument("--px4-serial", help="serial device for HITL, e.g. /dev/ttyACM0")
    r.add_argument("--px4-baud", type=int, default=921600)
    r.add_argument("--rt-priority", type=int, help="SCHED_FIFO priority (needs privileges)")
    r.add_argument("--rt-cpu", type=int, help="pin the executive to this CPU")
    r.set_defaults(func=_cmd_run)

    rp = sub.add_parser("reproduce", help="re-run a run from its manifest.json and compare")
    rp.add_argument("manifest")
    rp.add_argument("--out", default="runs")
    rp.set_defaults(func=_cmd_reproduce)

    st = sub.add_parser("suite", help="run several scenarios and write a suite report")
    st.add_argument("scenarios", nargs="+")
    st.add_argument("--controller", choices=["builtin", "px4"], default="builtin")
    st.add_argument("--jobs", type=int, default=1, help="parallel runs (builtin controller only)")
    st.add_argument("--px4-dir", default=os.environ.get("HWIL_PX4_DIR"),
                    help="PX4-Autopilot checkout; a fresh SITL instance is started per scenario")
    st.add_argument("--out", default="runs")
    st.add_argument("--no-plots", action="store_true")
    st.set_defaults(func=_cmd_suite)

    mc = sub.add_parser("montecarlo", help="run a Monte Carlo batch from montecarlo/*.yaml")
    mc.add_argument("config")
    mc.add_argument("--runs", type=int, help="override the number of runs")
    mc.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    mc.add_argument("--controller", choices=["builtin", "px4"], default="builtin")
    mc.add_argument("--px4-dir", default=os.environ.get("HWIL_PX4_DIR"))
    mc.add_argument("--out", default="runs")
    mc.set_defaults(func=_cmd_montecarlo)

    v = sub.add_parser("validate", help="check scenario files")
    v.add_argument("scenarios", nargs="+")
    v.set_defaults(func=_cmd_validate)

    a = p.parse_args(argv)
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
