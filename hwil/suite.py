"""Run many scenarios and summarize them in one suite report.

``hwil suite scenarios/*.yaml`` runs every file with the builtin controller (in
parallel with ``--jobs``). With ``--controller px4 --px4-dir <PX4-Autopilot>``
it starts a fresh PX4 SITL instance for each scenario, so no parameter or
estimator state leaks from one run into the next.
"""

from __future__ import annotations

import datetime as _dt
import html
import json
import os
import subprocess
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from . import scenario as scn
from .manifest import git_info
from .run import run_scenario

REPO_ROOT = Path(__file__).resolve().parent.parent
PX4_LAUNCHER = REPO_ROOT / "ci" / "run_px4_sitl.sh"


class Px4Sitl:
    """Context manager around ci/run_px4_sitl.sh."""

    def __init__(self, px4_dir: str | Path, work_dir: Path):
        self.px4_dir = Path(px4_dir)
        self.work_dir = work_dir
        self.proc: subprocess.Popen | None = None

    def __enter__(self):
        self.work_dir.mkdir(parents=True, exist_ok=True)
        self._log = open(self.work_dir / "px4.log", "w")
        env = {**os.environ, "PX4_DIR": str(self.px4_dir), "PX4_WORK_DIR": str(self.work_dir / "px4")}
        self.proc = subprocess.Popen([str(PX4_LAUNCHER)], env=env, stdout=self._log,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(0.5)
        return self

    def __exit__(self, *exc):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self._log.close()


def _one(args) -> dict:
    path, opts, out_root = args
    sc = scn.load(path)
    t0 = time.monotonic()
    try:
        run_dir, man = run_scenario(sc, opts, out_root, scenario_file=str(path))
    except Exception as exc:  # a crash in one scenario must not stop the suite
        return {"scenario": sc["name"], "file": str(path), "error": f"{type(exc).__name__}: {exc}",
                "wall_s": time.monotonic() - t0}
    res = man["results"]
    return {
        "scenario": sc["name"], "file": str(path), "run_dir": str(run_dir),
        "passed": bool(res["all_requirements_passed"]), "landed": res["landed"], "aborted": res["aborted"],
        "requirements": res["requirements"], "sim_time_s": res["sim_time_s"],
        "wall_s": time.monotonic() - t0, "config_hash": man["config_hash"],
    }


def run_suite(paths: list[str | Path], opts: dict, out_root: str | Path, jobs: int = 1,
              px4_dir: str | None = None) -> tuple[Path, dict]:
    stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    suite_dir = Path(out_root) / f"suite_{stamp}_{opts.get('controller', 'builtin')}"
    suite_dir.mkdir(parents=True)
    paths = [Path(p) for p in paths]
    results: list[dict] = []
    if opts.get("controller") == "px4":
        if not px4_dir:
            raise ValueError("--px4-dir is required for a PX4 suite")
        for i, p in enumerate(paths):
            with Px4Sitl(px4_dir, suite_dir / "px4" / f"{i:02d}_{p.stem}"):
                results.append(_one((p, opts, suite_dir)))
    elif jobs > 1:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            results = list(pool.map(_one, [(p, opts, suite_dir) for p in paths]))
    else:
        results = [_one((p, opts, suite_dir)) for p in paths]

    summary = {
        "created_utc": stamp,
        "controller": opts.get("controller", "builtin"),
        "git": git_info(),
        "scenarios": len(results),
        "passed": sum(1 for r in results if r.get("passed")),
        "requirements_total": sum(len(r.get("requirements", [])) for r in results),
        "requirements_passed": sum(sum(q["passed"] for q in r.get("requirements", [])) for r in results),
        "results": results,
    }
    (suite_dir / "suite.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_suite_html(suite_dir, summary)
    return suite_dir, summary


def write_suite_html(suite_dir: Path, summary: dict) -> Path:
    rows = []
    for r in summary["results"]:
        name = html.escape(r["scenario"])
        if "error" in r:
            rows.append(f"<tr><td>{name}</td><td class='fail'>ERROR</td><td colspan='3'>{html.escape(r['error'])}</td></tr>")
            continue
        link = os.path.relpath(Path(r["run_dir"]) / "report.html", suite_dir)
        reqs = "".join(
            f"<div class='req {'pass' if q['passed'] else 'fail'}' title='{html.escape(q['text'])}'>"
            f"{html.escape(q['id'])} "
            f"{'PASS' if q['passed'] else ('NOT EXERCISED' if q.get('vacuous') else 'FAIL')}"
            f" <span class='m'>{q['robustness']:.2f}</span></div>"
            for q in r["requirements"])
        status = "PASS" if r["passed"] else "FAIL"
        note = html.escape(r["aborted"]) if r.get("aborted") else ("" if r["landed"] else "did not land")
        rows.append(
            f"<tr><td><a href='{html.escape(link)}'>{name}</a></td>"
            f"<td class='{status.lower()}'>{status}</td><td>{reqs}</td>"
            f"<td>{r['sim_time_s']:.0f} s / {r['wall_s']:.0f} s</td><td>{note}</td></tr>")
    g = summary["git"]
    page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Suite report</title>
<style>
body{{font-family:system-ui,sans-serif;background:#fcfcfb;color:#0b0b0b;max-width:1100px;margin:24px auto;padding:0 16px}}
table{{border-collapse:collapse;width:100%}} th,td{{text-align:left;vertical-align:top;padding:6px 10px 6px 0;border-bottom:1px solid #e4e3df;font-size:14px}}
th{{color:#52514e;font-weight:500}} .pass{{color:#008300;font-weight:600}} .fail{{color:#e34948;font-weight:600}}
.req{{font-size:13px}} .m{{color:#52514e;font-weight:400}} a{{color:#2a78d6}}
</style></head><body>
<h1>Suite report: {html.escape(summary['controller'])}</h1>
<p>{summary['passed']} of {summary['scenarios']} scenarios passed; {summary['requirements_passed']} of
{summary['requirements_total']} requirements passed. Git {html.escape((g.get('commit') or 'n/a')[:12])}{' (dirty)' if g.get('dirty') else ''}.
Robustness is the margin in each requirement's units; boolean-only specs show 1000000.</p>
<table><tr><th>Scenario</th><th>Result</th><th>Requirements (robustness)</th><th>Sim / wall</th><th>Notes</th></tr>
{''.join(rows)}</table></body></html>"""
    out = suite_dir / "suite.html"
    out.write_text(page, encoding="utf-8")
    return out
