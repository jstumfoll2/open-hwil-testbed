"""Timing report: frame jitter, compute time, overruns and PX4 round-trip latency.

    hwil timing runs/<run A> runs/<run B> ... --out timing_report.html

Each run folder contributes its ``timing.csv`` and ``manifest.json``. The page
compares the runs side by side (for example the Python and C++ executives, or
a stock and a PREEMPT_RT kernel), so it is the M4 and M7 deliverable.

Definitions (per frame, period P = 1 / rate):
  start lateness  = actual frame start - scheduled start (realtime clock only)
  compute         = frame work time, excluding time spent waiting on PX4
  PX4 round trip  = HIL_SENSOR sent -> HIL_ACTUATOR_CONTROLS received
  overrun         = frame finished after the next frame's scheduled start
"""

from __future__ import annotations

import base64
import html
import io
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .report import INK, INK_2, SERIES, SURFACE, _style  # noqa: E402

PCTS = (50, 99, 99.9)


def load_run(run_dir: Path) -> dict:
    run_dir = Path(run_dir)
    man = json.loads((run_dir / "manifest.json").read_text())
    t = np.loadtxt(run_dir / "timing.csv", delimiter=",", skiprows=1, ndmin=2)
    opts = man.get("run_options", man["options"])
    label = (f"{opts.get('executive', 'python')} executive, {opts['controller']}, {opts['clock']}"
             f"{', PREEMPT_RT' if man['platform'].get('preempt_rt') else ', stock kernel'}")
    return {"dir": run_dir, "manifest": man, "label": label, "period_us": 1e6 / man["scenario"]["sim"]["rate_hz"],
            "late": t[:, 2], "compute": t[:, 3], "io": t[:, 4], "overrun": t[:, 5],
            "realtime": opts["clock"] == "realtime", "px4": opts["controller"] == "px4"}


def stats(x: np.ndarray) -> dict:
    if x.size == 0:
        return {}
    out = {f"p{p:g}": float(np.percentile(x, p)) for p in PCTS}
    out["max"] = float(x.max())
    out["mean"] = float(x.mean())
    return out


def _fig_b64(fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, facecolor=SURFACE)
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def _hist(runs: list[dict], key: str, title: str, xlabel: str) -> str:
    fig, ax = plt.subplots(figsize=(9, 3.4), facecolor=SURFACE)
    data = [r[key] for r in runs if r[key].size and np.any(r[key] > 0)]
    if not data:
        plt.close(fig)
        return ""
    hi = max(float(np.percentile(d, 99.99)) for d in data) * 1.1 + 1e-9
    bins = np.linspace(0, hi, 90)
    for i, r in enumerate(runs):
        if r[key].size and np.any(r[key] > 0):
            ax.hist(np.clip(r[key], 0, hi), bins=bins, histtype="step", linewidth=2,
                    color=SERIES[i % len(SERIES)], label=r["label"])
    ax.set_yscale("log")
    _style(ax, "frames (log scale)")
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9)
    ax.set_title(title, loc="left", color=INK, fontsize=11)
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    return _fig_b64(fig)


def build(run_dirs: list[str | Path]) -> dict:
    runs = [load_run(Path(d)) for d in run_dirs]
    rows = []
    for r in runs:
        m = r["manifest"]
        rows.append({
            "run": str(r["dir"]), "label": r["label"], "frames": int(r["late"].size),
            "period_us": r["period_us"], "overruns": int(r["overrun"].sum()),
            "kernel": f"{m['platform']['release']}", "preempt_rt": bool(m["platform"].get("preempt_rt")),
            "late_us": stats(r["late"]) if r["realtime"] else {},
            "compute_us": stats(r["compute"]),
            "px4_round_trip_us": stats(r["io"]) if r["px4"] else {},
            "duration_s": float(m["results"].get("sim_time_s", 0)),
        })
    return {"runs": rows, "_runs": runs}


def write_html(report: dict, out: Path) -> Path:
    runs = report["_runs"]

    def cells(d: dict) -> str:
        if not d:
            return "<td colspan='4' class='m'>n/a</td>"
        return "".join(f"<td>{d[k]:,.0f}</td>" for k in ("p50", "p99", "p99.9", "max"))

    body = "".join(
        f"<tr><td>{html.escape(r['label'])}<br><span class='m'>{html.escape(r['run'])}</span></td>"
        f"<td>{r['frames']:,} @ {r['period_us']:,.0f} us</td><td>{r['overruns']:,}</td>"
        f"{cells(r['late_us'])}{cells(r['compute_us'])}{cells(r['px4_round_trip_us'])}</tr>"
        for r in report["runs"])
    imgs = ""
    for key, title, xl in (("late", "Frame start lateness", "start lateness (us)"),
                           ("compute", "Frame compute time", "compute time (us)"),
                           ("io", "PX4 round trip (HIL_SENSOR to HIL_ACTUATOR_CONTROLS)", "round trip (us)")):
        b64 = _hist(runs, key, title, xl)
        if b64:
            imgs += f"<img alt='{html.escape(title)}' src='data:image/png;base64,{b64}'>"
    page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Timing report</title>
<style>
body{{font-family:system-ui,sans-serif;background:#fcfcfb;color:#0b0b0b;max-width:1200px;margin:24px auto;padding:0 16px}}
.wrap{{overflow-x:auto}} table{{border-collapse:collapse}} th,td{{text-align:right;padding:5px 10px;border-bottom:1px solid #e4e3df;font-size:13px;white-space:nowrap}}
th{{color:#52514e;font-weight:500}} td:first-child,th:first-child{{text-align:left;white-space:normal}} .m{{color:#52514e;font-size:12px}}
img{{max-width:100%;margin:8px 0}}
</style></head><body>
<h1>Timing report</h1>
<p class="m">All times in microseconds. Start lateness applies to realtime-clock runs only. An overrun is a frame that
finished after the next frame's scheduled start. PX4 round trip is the lockstep wait from HIL_SENSOR to
HIL_ACTUATOR_CONTROLS.</p>
<div class="wrap"><table>
<tr><th rowspan="2">Run</th><th rowspan="2">Frames</th><th rowspan="2">Overruns</th>
<th colspan="4">Start lateness</th><th colspan="4">Compute</th><th colspan="4">PX4 round trip</th></tr>
<tr>{"<th>p50</th><th>p99</th><th>p99.9</th><th>max</th>" * 3}</tr>
{body}</table></div>
{imgs}
</body></html>"""
    out.write_text(page, encoding="utf-8")
    (out.with_suffix(".json")).write_text(json.dumps({"runs": report["runs"]}, indent=2), encoding="utf-8")
    return out
