"""Plots and a small HTML report for one run.

M1 keeps this to static PNGs plus an index page; the M4 test-harness report
(requirements table, interactive plots) builds on it.
"""

from __future__ import annotations

import html
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .executive import LOG_COLUMNS  # noqa: E402

# Categorical slots in fixed order, then neutral ink for "commanded".
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"


def _style(ax, ylabel: str) -> None:
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK_2)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9)


def _phase_spans(ax, t, phase) -> None:
    """Shade the hold phase lightly so the eye finds the loiter window."""
    hold = phase == 2
    if hold.any():
        idx = np.flatnonzero(hold)
        ax.axvspan(t[idx[0]], t[idx[-1]], color="#cde2fb", alpha=0.35, linewidth=0, label="hold")


def load_log(path: Path) -> dict[str, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    with open(path) as fh:
        cols = fh.readline().strip().split(",")
    return {c: data[:, i] for i, c in enumerate(cols)}


def make_plots(run_dir: Path) -> list[str]:
    log = load_log(run_dir / "log.csv")
    t = log["t"]
    files = []

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True, facecolor=SURFACE)
    ax = axes[0]
    _phase_spans(ax, t, log["phase"])
    ax.plot(t, log["alt_cmd_m"], color=INK_2, linewidth=1.5, linestyle="--", label="commanded")
    ax.plot(t, log["alt_m"], color=SERIES[0], linewidth=2, label="altitude")
    _style(ax, "altitude above home (m)")
    ax.legend(loc="upper right", fontsize=8, frameon=False)
    ax.set_title("Altitude, position and attitude", loc="left", color=INK, fontsize=11)

    ax = axes[1]
    ax.plot(t, log["n_m"], color=SERIES[0], linewidth=2, label="north")
    ax.plot(t, log["e_m"], color=SERIES[1], linewidth=2, label="east")
    _style(ax, "position from home (m)")
    ax.legend(loc="upper right", fontsize=8, frameon=False)

    ax = axes[2]
    ax.plot(t, np.degrees(log["roll_rad"]), color=SERIES[0], linewidth=1.5, label="roll")
    ax.plot(t, np.degrees(log["pitch_rad"]), color=SERIES[1], linewidth=1.5, label="pitch")
    _style(ax, "attitude (deg)")
    ax.set_xlabel("simulation time (s)", color=INK_2, fontsize=9)
    ax.legend(loc="upper right", fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(run_dir / "flight.png", dpi=120, facecolor=SURFACE)
    plt.close(fig)
    files.append("flight.png")

    fig, ax = plt.subplots(figsize=(9, 3.2), facecolor=SURFACE)
    names = ["front right", "aft left", "front left", "aft right"]
    for i in range(4):
        ax.plot(t, log[f"m{i}"], color=SERIES[i], linewidth=1.2, label=f"m{i} {names[i]}")
    _style(ax, "motor command (0-1)")
    ax.set_xlabel("simulation time (s)", color=INK_2, fontsize=9)
    ax.set_title("Motor commands", loc="left", color=INK, fontsize=11)
    ax.legend(loc="upper right", fontsize=8, frameon=False, ncol=4)
    fig.tight_layout()
    fig.savefig(run_dir / "motors.png", dpi=120, facecolor=SURFACE)
    plt.close(fig)
    files.append("motors.png")

    timing = np.loadtxt(run_dir / "timing.csv", delimiter=",", skiprows=1, ndmin=2)
    fig, ax = plt.subplots(figsize=(9, 3.2), facecolor=SURFACE)
    compute = timing[:, 3]
    hi = max(float(np.percentile(compute, 99.9)) * 1.2, 1.0)
    ax.hist(np.clip(compute, 0, hi), bins=80, color=SERIES[0], edgecolor=SURFACE, linewidth=0.5)
    _style(ax, "frames")
    ax.set_xlabel("frame compute time (us), clipped at 1.2x p99.9", color=INK_2, fontsize=9)
    ax.set_title("Frame compute time", loc="left", color=INK, fontsize=11)
    fig.tight_layout()
    fig.savefig(run_dir / "timing.png", dpi=120, facecolor=SURFACE)
    plt.close(fig)
    files.append("timing.png")
    return files


def write_html(run_dir: Path, manifest: dict, images: list[str]) -> Path:
    res = manifest.get("results", {})
    tim = res.get("timing", {})
    rows = [
        ("Scenario", manifest["scenario"]["name"]),
        ("Controller", manifest["options"]["controller"]),
        ("Clock", manifest["options"]["clock"]),
        ("Seed", manifest["seed"]),
        ("Config hash", manifest["config_hash"][:16]),
        ("Git commit", (manifest["git"].get("commit") or "n/a")[:12] + (" (dirty)" if manifest["git"].get("dirty") else "")),
        ("Sim time (s)", f"{res.get('sim_time_s', 0):.2f}"),
        ("Landed", res.get("landed")),
        ("Aborted", res.get("aborted") or "no"),
        ("Frames / overruns", f"{tim.get('frames')} / {tim.get('overruns')}"),
        ("Compute p50 / p99 (us)", f"{tim.get('compute_us_p50', 0):.0f} / {tim.get('compute_us_p99', 0):.0f}"),
    ]
    reqs = res.get("requirements", [])
    req_rows = "".join(
        f"<tr><td>{html.escape(r['id'])}</td><td>{html.escape(r['text'])}</td>"
        f"<td class='{'pass' if r['passed'] else 'fail'}'>"
        f"{'PASS' if r['passed'] else ('NOT EXERCISED' if r.get('vacuous') else 'FAIL')}</td>"
        f"<td>{r['robustness']:.3f}</td></tr>"
        for r in reqs
    )
    body = "".join(f"<tr><th>{html.escape(str(k))}</th><td>{html.escape(str(v))}</td></tr>" for k, v in rows)
    imgs = "".join(f"<img src='{i}' alt='{i}'>" for i in images)
    page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Run report</title>
<style>
body{{font-family:system-ui,sans-serif;background:#fcfcfb;color:#0b0b0b;max-width:960px;margin:24px auto;padding:0 16px}}
table{{border-collapse:collapse;margin:12px 0 24px}} th,td{{text-align:left;padding:4px 12px 4px 0;border-bottom:1px solid #e4e3df;font-size:14px}}
th{{color:#52514e;font-weight:500}} img{{max-width:100%;margin:8px 0}}
.pass{{color:#008300;font-weight:600}} .fail{{color:#e34948;font-weight:600}}
</style></head><body>
<h1>Run report: {html.escape(manifest['scenario']['name'])}</h1>
<table>{body}</table>
{"<h2>Requirements</h2><table><tr><th>ID</th><th>Requirement</th><th>Result</th><th>Robustness</th></tr>" + req_rows + "</table>" if reqs else ""}
{imgs}
<h2>Manifest</h2><pre>{html.escape(json.dumps({k: v for k, v in manifest.items() if k != 'scenario'}, indent=2))}</pre>
</body></html>"""
    out = run_dir / "report.html"
    out.write_text(page, encoding="utf-8")
    return out


def columns() -> list[str]:
    return list(LOG_COLUMNS)
