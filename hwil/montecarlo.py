"""Monte Carlo batches: one scenario, many dispersed copies, statistics at the end.

A Monte Carlo file (see ``montecarlo/*.yaml``) names a base scenario and the
parameters to disperse::

    scenario: ../scenarios/takeoff_hold_land.yaml
    runs: 200
    seed: 1000
    dispersions:
      wind.north_mps:            {dist: uniform, low: -5, high: 5}
      wind.turbulence_severity:  {dist: choice, values: [0, 1, 2, 3]}
      sensors.imu.accel_noise_mps2: {dist: normal, mean: 0.02, std: 0.005, min: 0.0}
      faults[0].factor:          {dist: uniform, low: 0.7, high: 1.0}

Run ``i`` gets the scenario seed ``seed + i`` (sensor noise, turbulence) and
its dispersed values from a separate generator seeded with ``seed``. Every run
writes a normal run folder with its own manifest holding the fully dispersed
scenario, so any single case, typically a failure, can be replayed with
``hwil reproduce``.
"""

from __future__ import annotations

import copy
import csv
import datetime as _dt
import html
import json
import math
import os
import re
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml

from . import scenario as scn
from .manifest import git_info
from .suite import Px4Sitl
from .run import run_scenario

DISTS = {"uniform", "normal", "choice", "constant"}
_PATH_TOKEN = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)|\[(\d+)\]")


class MonteCarloError(ValueError):
    pass


def load_config(path: str | Path) -> dict:
    path = Path(path)
    cfg = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    for key in ("scenario", "runs", "dispersions"):
        if key not in cfg:
            raise MonteCarloError(f"{path.name}: missing '{key}'")
    cfg["scenario_path"] = str((path.parent / cfg["scenario"]).resolve())
    cfg.setdefault("seed", 0)
    cfg["name"] = cfg.get("name", path.stem)
    for key, spec in cfg["dispersions"].items():
        if spec.get("dist") not in DISTS:
            raise MonteCarloError(f"{key}: dist must be one of {sorted(DISTS)}")
    return cfg


def _split(path: str) -> list:
    parts = []
    for name, idx in _PATH_TOKEN.findall(path):
        parts.append(name if name else int(idx))
    if not parts:
        raise MonteCarloError(f"bad parameter path {path!r}")
    return parts


def set_path(obj, path: str, value) -> None:
    parts = _split(path)
    for p in parts[:-1]:
        obj = obj[p]
    last = parts[-1]
    if isinstance(obj, dict) and last not in obj:
        raise MonteCarloError(f"{path}: '{last}' does not exist in the scenario")
    obj[last] = value


def draw(spec: dict, rng: np.random.Generator):
    d = spec["dist"]
    if d == "uniform":
        v = float(rng.uniform(spec["low"], spec["high"]))
    elif d == "normal":
        v = float(rng.normal(spec["mean"], spec["std"]))
    elif d == "choice":
        v = spec["values"][int(rng.integers(len(spec["values"])))]
        return v
    else:
        return spec["value"]
    if "min" in spec:
        v = max(v, float(spec["min"]))
    if "max" in spec:
        v = min(v, float(spec["max"]))
    return v


def make_cases(cfg: dict) -> list[tuple[dict, dict]]:
    """Return [(scenario, dispersed_values)] for every run, deterministically."""
    base = scn.load(cfg["scenario_path"])
    rng = np.random.default_rng(int(cfg["seed"]))
    keys = sorted(cfg["dispersions"])
    cases = []
    for i in range(int(cfg["runs"])):
        sc = copy.deepcopy(base)
        values = {k: draw(cfg["dispersions"][k], rng) for k in keys}
        for k, v in values.items():
            set_path(sc, k, v)
        sc["seed"] = int(cfg["seed"]) + i
        sc["name"] = f"{base['name']}_mc{i:04d}"
        scn.validate(sc)
        cases.append((sc, values))
    return cases


def _run_case(args) -> dict:
    i, sc, values, opts, out_root = args
    try:
        run_dir, man = run_scenario(sc, opts, out_root, run_name=f"run{i:04d}")
    except Exception as exc:
        return {"index": i, "values": values, "error": f"{type(exc).__name__}: {exc}"}
    res = man["results"]
    return {
        "index": i, "values": values, "run_dir": str(run_dir), "seed": sc["seed"],
        "passed": bool(res["all_requirements_passed"]), "aborted": res["aborted"], "landed": res["landed"],
        "requirements": {r["id"]: {"robustness": r["robustness"], "passed": r["passed"]}
                         for r in res["requirements"]},
    }


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """95 % Wilson score interval for a pass rate."""
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    den = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, centre - half), min(1.0, centre + half))


def run_montecarlo(cfg_path: str | Path, out_root: str | Path, jobs: int = 1, controller: str = "builtin",
                   px4_dir: str | None = None, runs: int | None = None) -> tuple[Path, dict]:
    cfg = load_config(cfg_path)
    if runs is not None:
        cfg["runs"] = runs
    cases = make_cases(cfg)
    stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    mc_dir = Path(out_root) / f"mc_{stamp}_{cfg['name']}_{controller}"
    mc_dir.mkdir(parents=True)
    opts = {"controller": controller, "plots": False}
    args = [(i, sc, vals, opts, mc_dir) for i, (sc, vals) in enumerate(cases)]
    if controller == "px4":
        if not px4_dir:
            raise MonteCarloError("--px4-dir is required with --controller px4")
        results = []
        for a in args:
            with Px4Sitl(px4_dir, mc_dir / "px4" / f"run{a[0]:04d}"):
                results.append(_run_case(a))
    elif jobs > 1:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            results = list(pool.map(_run_case, args, chunksize=1))
    else:
        results = [_run_case(a) for a in args]

    summary = summarize(cfg, results, controller)
    summary["created_utc"] = stamp
    summary["git"] = git_info()
    (mc_dir / "montecarlo.json").write_text(json.dumps(summary, indent=2, default=float), encoding="utf-8")
    write_csv(mc_dir / "montecarlo.csv", cfg, results)
    write_html(mc_dir, cfg, summary, results)
    return mc_dir, summary


def summarize(cfg: dict, results: list[dict], controller: str) -> dict:
    ok = [r for r in results if "error" not in r]
    req_ids = sorted({k for r in ok for k in r["requirements"]})
    reqs = {}
    for rid in req_ids:
        rob = np.array([r["requirements"][rid]["robustness"] for r in ok if rid in r["requirements"]])
        k = int(sum(r["requirements"][rid]["passed"] for r in ok if rid in r["requirements"]))
        n = len(rob)
        finite = rob[np.isfinite(rob) & (np.abs(rob) < 1e5)]
        lo, hi = wilson(k, n)
        reqs[rid] = {
            "passed": k, "runs": n, "pass_rate": k / n if n else 0.0, "pass_rate_ci95": [lo, hi],
            "robustness_min": float(finite.min()) if finite.size else None,
            "robustness_p05": float(np.percentile(finite, 5)) if finite.size else None,
            "robustness_median": float(np.median(finite)) if finite.size else None,
        }
    k = sum(1 for r in ok if r["passed"])
    lo, hi = wilson(k, len(results))
    return {
        "name": cfg["name"], "scenario": cfg["scenario"], "controller": controller,
        "runs": len(results), "errors": len(results) - len(ok), "passed": k,
        "pass_rate": k / len(results) if results else 0.0, "pass_rate_ci95": [lo, hi],
        "aborted": sum(1 for r in ok if r["aborted"]),
        "dispersions": cfg["dispersions"], "seed": cfg["seed"], "requirements": reqs,
        "failures": [r["index"] for r in results if not r.get("passed")],
    }


def write_csv(path: Path, cfg: dict, results: list[dict]) -> None:
    keys = sorted(cfg["dispersions"])
    req_ids = sorted({k for r in results for k in r.get("requirements", {})})
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["run", "seed", "passed", "aborted", *keys, *[f"rob_{rid}" for rid in req_ids]])
        for r in results:
            w.writerow([r["index"], r.get("seed"), r.get("passed", False), r.get("aborted") or r.get("error") or "",
                        *[r["values"][k] for k in keys],
                        *[r.get("requirements", {}).get(rid, {}).get("robustness", "") for rid in req_ids]])


def _hist_svg(values: np.ndarray, width: int = 260, height: int = 70) -> str:
    """Tiny inline histogram of robustness with the pass/fail line at 0."""
    if values.size == 0:
        return ""
    lo, hi = float(min(values.min(), 0.0)), float(max(values.max(), 0.0))
    if hi - lo < 1e-9:
        hi = lo + 1.0
    counts, edges = np.histogram(values, bins=24, range=(lo, hi))
    bw = width / len(counts)
    top = counts.max() or 1
    bars = "".join(
        f"<rect x='{i * bw + 1:.1f}' y='{height - 14 - (c / top) * (height - 18):.1f}' width='{bw - 2:.1f}' "
        f"height='{(c / top) * (height - 18):.1f}' rx='2' fill='{'#e34948' if edges[i + 1] <= 0 else '#2a78d6'}'>"
        f"<title>{edges[i]:.2f} to {edges[i + 1]:.2f}: {c} runs</title></rect>"
        for i, c in enumerate(counts))
    x0 = (0.0 - lo) / (hi - lo) * width
    return (f"<svg width='{width}' height='{height}' role='img' aria-label='robustness histogram'>{bars}"
            f"<line x1='{x0:.1f}' x2='{x0:.1f}' y1='0' y2='{height - 14}' stroke='#52514e' stroke-dasharray='3 2'/>"
            f"<text x='0' y='{height - 2}' font-size='10' fill='#52514e'>{lo:.2f}</text>"
            f"<text x='{width}' y='{height - 2}' font-size='10' fill='#52514e' text-anchor='end'>{hi:.2f}</text></svg>")


def _strip_svg(vals: list[float], passed: list[bool], width: int = 320, height: int = 46) -> str:
    """One dot per run along the parameter axis: failures on the top row, passes below."""
    v = np.asarray(vals, dtype=float)
    lo, hi = float(v.min()), float(v.max())
    if hi - lo < 1e-12:
        hi = lo + 1.0
    dots = "".join(
        f"<circle cx='{6 + (x - lo) / (hi - lo) * (width - 12):.1f}' cy='{8 if not ok else 24}' r='4' "
        f"fill='{'#2a78d6' if ok else '#e34948'}' fill-opacity='0.75' stroke='#fcfcfb' stroke-width='1'>"
        f"<title>{x:.4g}: {'pass' if ok else 'FAIL'}</title></circle>"
        for x, ok in zip(v, passed))
    return (f"<svg width='{width}' height='{height}' role='img' aria-label='pass and fail by parameter value'>{dots}"
            f"<text x='0' y='{height - 2}' font-size='10' fill='#52514e'>{lo:.3g}</text>"
            f"<text x='{width}' y='{height - 2}' font-size='10' fill='#52514e' text-anchor='end'>{hi:.3g}</text></svg>")


def write_html(mc_dir: Path, cfg: dict, summary: dict, results: list[dict]) -> Path:
    rows = []
    for rid, s in summary["requirements"].items():
        rob = np.array([r["requirements"][rid]["robustness"] for r in results
                        if "requirements" in r and rid in r["requirements"]])
        rob = rob[np.isfinite(rob) & (np.abs(rob) < 1e5)]
        lo, hi = s["pass_rate_ci95"]
        fmt = (lambda v: "n/a" if v is None else f"{v:.2f}")
        rows.append(f"<tr><td>{html.escape(rid)}</td><td>{s['passed']}/{s['runs']}</td>"
                    f"<td>{100 * s['pass_rate']:.1f} % <span class='m'>({100 * lo:.1f} to {100 * hi:.1f})</span></td>"
                    f"<td>{fmt(s['robustness_min'])}</td><td>{fmt(s['robustness_p05'])}</td>"
                    f"<td>{fmt(s['robustness_median'])}</td><td>{_hist_svg(rob)}</td></tr>")
    fails = []
    keys = sorted(cfg["dispersions"])
    for r in results:
        if r.get("passed"):
            continue
        link = os.path.relpath(Path(r["run_dir"]) / "manifest.json", mc_dir) if "run_dir" in r else ""
        bad = ", ".join(k for k, v in r.get("requirements", {}).items() if not v["passed"]) or r.get("error", "")
        vals = ", ".join(f"{k}={r['values'][k]:.3g}" if isinstance(r["values"][k], float) else f"{k}={r['values'][k]}"
                         for k in keys)
        fails.append(f"<tr><td>{r['index']}</td><td>{html.escape(bad)}</td><td>{html.escape(vals)}</td>"
                     f"<td><code>hwil reproduce {html.escape(link)}</code></td></tr>")
    lo, hi = summary["pass_rate_ci95"]
    done = [r for r in results if "values" in r]
    disp = ""
    for k, v in cfg["dispersions"].items():
        xs = [r["values"][k] for r in done]
        strip = ""
        if xs and all(isinstance(x, (int, float)) for x in xs):
            strip = _strip_svg([float(x) for x in xs], [bool(r.get("passed")) for r in done])
        disp += f"<tr><td><code>{html.escape(k)}</code></td><td>{html.escape(json.dumps(v))}</td><td>{strip}</td></tr>"
    page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Monte Carlo report</title>
<style>
body{{font-family:system-ui,sans-serif;background:#fcfcfb;color:#0b0b0b;max-width:1100px;margin:24px auto;padding:0 16px}}
table{{border-collapse:collapse;margin:8px 0 24px}} th,td{{text-align:left;vertical-align:middle;padding:6px 12px 6px 0;border-bottom:1px solid #e4e3df;font-size:14px}}
th{{color:#52514e;font-weight:500}} .m{{color:#52514e}} code{{font-size:12px}}
</style></head><body>
<h1>Monte Carlo: {html.escape(summary['name'])} ({html.escape(summary['controller'])})</h1>
<p><b>{summary['passed']} of {summary['runs']}</b> runs passed every requirement:
{100 * summary['pass_rate']:.1f} % (95 % CI {100 * lo:.1f} to {100 * hi:.1f} %). {summary['aborted']} aborted,
{summary['errors']} errors. Base scenario <code>{html.escape(str(summary['scenario']))}</code>, seed {summary['seed']}.</p>
<h2>Requirements</h2>
<p class="m">Margins are in each requirement's units. Results of exactly ±1000000 (boolean outcomes such as
"never landed") count in the pass rate but are left out of the margin columns. Bars left of the dashed zero line are failures.</p>
<table><tr><th>ID</th><th>Passed</th><th>Pass rate (95 % CI)</th><th>Min margin</th><th>5th pct</th><th>Median</th><th>Distribution</th></tr>
{''.join(rows)}</table>
<h2>Dispersions</h2>
<p class="m">Each dot is one run at its drawn value: failures (red) on the top row, passes (blue) below.
A cluster of red at one end points at the parameter that drives the failures.</p>
<table><tr><th>Parameter</th><th>Distribution</th><th>Pass / fail by value</th></tr>{disp}</table>
<h2>Failed runs</h2>
{'<p>None.</p>' if not fails else '<table><tr><th>Run</th><th>Failed</th><th>Dispersed values</th><th>Replay</th></tr>' + ''.join(fails) + '</table>'}
</body></html>"""
    out = mc_dir / "montecarlo.html"
    out.write_text(page, encoding="utf-8")
    return out
