"""Run one scenario end to end: simulate, log, grade, plot, write the manifest."""

from __future__ import annotations

import datetime as _dt
import json
import sys
from pathlib import Path

import numpy as np

from . import manifest as mf
from . import models, report
from .executive import LOG_COLUMNS, TIMING_COLUMNS, Executive, set_realtime
from .mission import MissionSequencer
from .requirements import grade

DEFAULT_OPTIONS = {
    "controller": "builtin",   # builtin | px4
    "clock": "afap",           # afap | realtime
    "px4_sim_port": 4560,      # TCP port PX4 SITL connects to
    "px4_gcs": "udpin:0.0.0.0:14540",
    "px4_serial": None,        # e.g. /dev/ttyACM0 for HITL (M8)
    "px4_baud": 921600,
    "rt_priority": None,
    "rt_cpu": None,
    "plots": True,
}


def make_controller(sc: dict, opts: dict):
    if opts["controller"] == "builtin":
        from .controllers.builtin import BuiltinController
        return BuiltinController(MissionSequencer(sc["mission"]))
    if opts["controller"] == "px4":
        from .bridge.px4 import Px4Controller
        return Px4Controller(sc, opts)
    raise ValueError(f"unknown controller {opts['controller']!r}")


def _write_csv(path: Path, cols: list[str], arr: np.ndarray) -> None:
    np.savetxt(path, arr, delimiter=",", header=",".join(cols), comments="", fmt="%.9g")


def run_scenario(sc: dict, options: dict | None = None, out_root: str | Path = "runs",
                 scenario_file: str | None = None, run_name: str | None = None) -> tuple[Path, dict]:
    opts = {**DEFAULT_OPTIONS, **(options or {})}
    model = sc["vehicle"]["model"]
    model_hash = models.overlay_hash(model) if model in models.OVERLAYS else None
    # Only result-relevant options go into the config hash.
    result_opts = {k: opts[k] for k in ("controller", "clock")}
    man = mf.build(sc, result_opts, model_hash, scenario_file, sys.argv)
    man["run_options"] = opts
    stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = Path(out_root) / (run_name or f"{stamp}_{sc['name']}_{man['config_hash'][:8]}")
    run_dir.mkdir(parents=True, exist_ok=False)

    rt_info = {}
    if opts["clock"] == "realtime" and (opts["rt_priority"] is not None or opts["rt_cpu"] is not None):
        rt_info = set_realtime(opts["rt_priority"], opts["rt_cpu"])

    ctrl = make_controller(sc, opts)
    ex = Executive(sc, ctrl, realtime=opts["clock"] == "realtime")
    results = ex.run()
    results["realtime_setup"] = rt_info
    man["controller_info"] = ctrl.info()

    _write_csv(run_dir / "log.csv", LOG_COLUMNS, ex.log)
    _write_csv(run_dir / "timing.csv", TIMING_COLUMNS, ex.timing_rows)

    log = {c: ex.log[:, i] for i, c in enumerate(LOG_COLUMNS)}
    reqs = grade(sc["requirements"], log) if sc["requirements"] else []
    results["requirements"] = [r.as_dict() for r in reqs]
    results["all_requirements_passed"] = all(r.passed for r in reqs) and not results["aborted"]
    man["results"] = results

    images = report.make_plots(run_dir) if opts["plots"] else []
    man["outputs"] = {name: mf.sha256_file(run_dir / name) for name in ("log.csv", "timing.csv")}
    (run_dir / "manifest.json").write_text(json.dumps(man, indent=2, default=_json_default), encoding="utf-8")
    report.write_html(run_dir, man, images)
    return run_dir, man


def _json_default(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def reproduce(manifest_path: str | Path, out_root: str | Path = "runs") -> tuple[Path, dict, dict]:
    """Re-run a manifest and compare. Returns (run_dir, new_manifest, comparison)."""
    old = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    run_dir, new = run_scenario(old["scenario"], old.get("run_options", old["options"]), out_root,
                                scenario_file=old.get("scenario_file"),
                                run_name=Path(manifest_path).parent.name + "_repro")
    cmp = {
        "config_hash_match": old["config_hash"] == new["config_hash"],
        "log_match": old["outputs"].get("log.csv") == new["outputs"].get("log.csv"),
        "git_commit_match": old["git"].get("commit") == new["git"].get("commit"),
        "versions_match": old["versions"] == new["versions"],
        "deterministic_expected": old["options"].get("clock") == "afap" and old["options"].get("controller") == "builtin",
    }
    return run_dir, new, cmp
