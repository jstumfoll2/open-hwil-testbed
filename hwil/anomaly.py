"""Anomaly detection over run logs (M5).

    hwil anomaly fit   <nominal runs or batch folders> --out baseline.json
    hwil anomaly score <runs> [--baseline baseline.json]

The detector learns what nominal flights look like and flags stretches of a new
run that fall outside that envelope, naming the signals responsible. It needs no
labels and no knowledge of the scenario's faults, which is what makes it useful
for spotting problems nobody wrote a requirement for.

Method, kept simple so every flag can be explained:

1. Each ``log.csv`` is averaged into 0.1 s blocks.
2. Features are computed as residuals that should stay near zero whatever the
   mission does: the roll, pitch and yaw torque the mixer is asking for, how much
   collective thrust the vertical acceleration needed against a fitted thrust
   curve, baro altitude against true altitude, GPS validity, and rate noise.
   Bias-type features use a 2 s rolling mean. Tracking error is left to the
   STL requirements, which know the mission.
3. ``fit`` stores, for each feature and each phase group (ground, climb or
   descent, cruise), the median and a robust spread from the 0.1 and 99.9
   percentiles of the nominal runs.
4. ``score`` turns each sample into a z-score per feature and reports a segment
   wherever any |z| stays above the threshold for at least ``min_s`` seconds.

The baseline belongs to one controller: the builtin controller and PX4 fly
differently, so fit one baseline per controller.
"""

from __future__ import annotations

import base64
import html
import io
import json
from pathlib import Path

import numpy as np

from .executive import LOG_COLUMNS

BLOCK_S = 0.1
ROLL_S = 2.0
NOISE_S = 1.0
DEFAULT_Z = 7.0
DEFAULT_MIN_S = 1.0

# Phase ids from hwil.mission.PHASE_IDS grouped into regimes with similar dynamics.
GROUPS = {"ground": (0, 4), "climb_descent": (1, 3), "cruise": (2, 5)}

# Quad-x mixer (PX4 order FR, AL, FL, AR); columns roll, pitch, yaw. Same as
# hwil/controllers/builtin.py. Its pseudo-inverse recovers the torque demand.
MIX = np.array([[-1, 1, 1], [1, -1, 1], [1, 1, -1], [-1, -1, -1]], dtype=float)
UNMIX = np.linalg.pinv(MIX)

# Feature name -> (description, unit, floor on the spread).
FEATURES = {
    "torque_roll": ("roll torque demand from the motor mix (2 s mean)", "cmd", 0.002),
    "torque_pitch": ("pitch torque demand from the motor mix (2 s mean)", "cmd", 0.002),
    "torque_yaw": ("yaw torque demand from the motor mix (2 s mean)", "cmd", 0.002),
    "thrust_resid": ("collective command beyond what the measured lift needs (2 s mean)", "cmd", 0.005),
    "baro_resid": ("baro altitude minus true altitude, start-up offset removed (2 s mean)", "m", 0.15),
    "gps_invalid": ("share of the last second without a valid GPS fix", "", 0.05),
    "rate_noise": ("body-rate noise, std of p, q, r over 1 s", "rad/s", 0.01),
}
FEATURE_NAMES = list(FEATURES)
_COL = {c: i for i, c in enumerate(LOG_COLUMNS)}


def _rolling_mean(x: np.ndarray, n: int) -> np.ndarray:
    """Trailing mean over n samples (shorter at the start)."""
    c = np.cumsum(np.insert(x, 0, 0.0))
    idx = np.arange(1, x.size + 1)
    lo = np.maximum(idx - n, 0)
    return (c[idx] - c[lo]) / (idx - lo)


def _rolling_std(x: np.ndarray, n: int) -> np.ndarray:
    m = _rolling_mean(x, n)
    m2 = _rolling_mean(x * x, n)
    return np.sqrt(np.maximum(m2 - m * m, 0.0))


def load_log(run_dir: Path) -> np.ndarray:
    return np.loadtxt(Path(run_dir) / "log.csv", delimiter=",", skiprows=1, ndmin=2)


def blocks(log: np.ndarray, dt: float) -> dict[str, np.ndarray]:
    """Average the log into BLOCK_S blocks. Phase takes the block's last value."""
    k = max(int(round(BLOCK_S / dt)), 1)
    n = (log.shape[0] // k) * k
    if n == 0:
        raise ValueError("log is shorter than one block")
    b = log[:n].reshape(-1, k, log.shape[1])
    out = {c: b[:, :, i].mean(axis=1) for c, i in _COL.items()}
    out["phase"] = b[:, -1, _COL["phase"]].round().astype(int)
    # Rate noise needs the within-block spread, not just the mean.
    out["rate_var"] = sum(b[:, :, _COL[c]].var(axis=1) for c in ("p_rps", "q_rps", "r_rps"))
    return out


def raw_features(log: np.ndarray, dt: float) -> tuple[np.ndarray, dict[str, np.ndarray], np.ndarray]:
    """Return block times, raw feature arrays (before the thrust fit) and phases."""
    b = blocks(log, dt)
    nroll = int(round(ROLL_S / BLOCK_S))
    nnoise = int(round(NOISE_S / BLOCK_S))
    m = np.stack([b[f"m{i}"] for i in range(4)], axis=1)
    tau = m @ UNMIX.T
    collective = m.mean(axis=1)
    phase = b["phase"]
    airborne = np.isin(phase, GROUPS["climb_descent"] + GROUPS["cruise"]) & (b["alt_agl_m"] > 0.5)

    # Baro offset: median over the first second on the ground.
    n0 = max(int(round(1.0 / BLOCK_S)), 1)
    baro_diff = b["baro_alt_m"] - b["alt_m"]
    baro_resid = baro_diff - np.median(baro_diff[:n0])

    feats = {
        "torque_roll": _rolling_mean(np.where(airborne, tau[:, 0], 0.0), nroll),
        "torque_pitch": _rolling_mean(np.where(airborne, tau[:, 1], 0.0), nroll),
        "torque_yaw": _rolling_mean(np.where(airborne, tau[:, 2], 0.0), nroll),
        "_collective": collective,
        "_lift": -b["az_mps2"],
        "_airborne": airborne.astype(float),
        "baro_resid": _rolling_mean(baro_resid, nroll),
        "gps_invalid": _rolling_mean(1.0 - b["gps_valid"], nnoise),
        "rate_noise": np.sqrt(_rolling_mean(b["rate_var"], nnoise)),
    }
    return b["t"], feats, phase


def thrust_residual(feats: dict[str, np.ndarray], coef: list[float]) -> np.ndarray:
    """Collective command minus the command the thrust curve predicts for the measured lift."""
    pred = np.polyval(coef, np.maximum(feats["_lift"], 0.0))
    resid = np.where(feats["_airborne"] > 0, feats["_collective"] - pred, 0.0)
    return _rolling_mean(resid, int(round(ROLL_S / BLOCK_S)))


def _dt(run_dir: Path, log: np.ndarray) -> float:
    if log.shape[0] > 1:
        return float(np.median(np.diff(log[:10, 0])))
    man = json.loads((Path(run_dir) / "manifest.json").read_text())
    return 1.0 / man["scenario"]["sim"]["rate_hz"]


def find_runs(paths: list[str | Path]) -> list[Path]:
    """Expand run folders and batch folders (suite or Monte Carlo) into run folders."""
    out: list[Path] = []
    for p in map(Path, paths):
        if (p / "log.csv").exists():
            out.append(p)
        elif p.is_dir():
            out.extend(sorted(d.parent for d in p.rglob("log.csv")))
    return out


def _controller(run_dir: Path) -> str:
    try:
        man = json.loads((run_dir / "manifest.json").read_text())
        return man.get("run_options", man.get("options", {})).get("controller", "unknown")
    except (OSError, ValueError):
        return "unknown"


def _healthy(run_dir: Path) -> bool:
    """True unless the manifest says the run aborted or failed a requirement."""
    try:
        res = json.loads((run_dir / "manifest.json").read_text()).get("results", {})
    except (OSError, ValueError):
        return True
    if res.get("aborted"):
        return False
    return all(r.get("passed", True) for r in res.get("requirements", []))


def fit(paths: list[str | Path], include_failed: bool = False) -> dict:
    """Fit a baseline. Runs that aborted or failed a requirement are left out
    unless include_failed is set, so a tip-over in a dispersed batch does not
    widen the nominal envelope."""
    found = find_runs(paths)
    runs = found if include_failed else [r for r in found if _healthy(r)]
    if not runs:
        raise ValueError("no usable run folders with log.csv found")
    ctrls = sorted({_controller(r) for r in runs})
    per_run = []
    for r in runs:
        log = load_log(r)
        t, feats, phase = raw_features(log, _dt(r, log))
        per_run.append((feats, phase))

    # Thrust curve: collective command as a quadratic in measured lift, cruise only
    # (steady flight), over all nominal runs.
    lift = np.concatenate([f["_lift"][(f["_airborne"] > 0) & np.isin(p, GROUPS["cruise"])] for f, p in per_run])
    coll = np.concatenate([f["_collective"][(f["_airborne"] > 0) & np.isin(p, GROUPS["cruise"])] for f, p in per_run])
    if lift.size < 10:
        raise ValueError("nominal runs have too little cruise data to fit the thrust curve")
    coef = np.polyfit(lift, coll, 2).tolist()

    stats: dict[str, dict] = {}
    for name in FEATURE_NAMES:
        floor = FEATURES[name][2]
        stats[name] = {}
        for g, ids in GROUPS.items():
            vals = []
            for f, p in per_run:
                x = thrust_residual(f, coef) if name == "thrust_resid" else f[name]
                vals.append(x[np.isin(p, ids)])
            v = np.concatenate(vals)
            if v.size == 0:
                stats[name][g] = {"median": 0.0, "spread": floor, "n": 0}
                continue
            lo, med, hi = np.percentile(v, [0.1, 50, 99.9])
            # Treat the 0.1 to 99.9 percentile range as about +/- 3.1 sigma.
            spread = max((hi - lo) / 6.18, floor)
            stats[name][g] = {"median": float(med), "spread": float(spread), "n": int(v.size)}
    return {
        "version": 1,
        "controllers": ctrls,
        "runs": [r.name for r in runs],
        "n_runs": len(runs),
        "n_excluded": len(found) - len(runs),
        "block_s": BLOCK_S,
        "thrust_curve": coef,
        "features": {k: {"description": FEATURES[k][0], "unit": FEATURES[k][1]} for k in FEATURE_NAMES},
        "stats": stats,
    }


def zscores(run_dir: Path, baseline: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    """Return block times, phases, the z matrix (blocks x features) and raw feature values."""
    log = load_log(run_dir)
    t, feats, phase = raw_features(log, _dt(run_dir, log))
    vals = {n: (thrust_residual(feats, baseline["thrust_curve"]) if n == "thrust_resid" else feats[n])
            for n in FEATURE_NAMES}
    z = np.zeros((t.size, len(FEATURE_NAMES)))
    for g, ids in GROUPS.items():
        sel = np.isin(phase, ids)
        for j, n in enumerate(FEATURE_NAMES):
            s = baseline["stats"][n][g]
            z[sel, j] = (vals[n][sel] - s["median"]) / s["spread"]
    return t, phase, z, vals


def segments(t: np.ndarray, z: np.ndarray, z_thresh: float, min_s: float) -> list[dict]:
    """Group consecutive over-threshold blocks into segments at least min_s long."""
    over = np.abs(z) > z_thresh
    any_over = over.any(axis=1)
    segs = []
    i = 0
    n = t.size
    while i < n:
        if not any_over[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and any_over[j + 1]:
            j += 1
        dur = t[j] - t[i] + BLOCK_S
        if dur >= min_s - 1e-9:
            zz = np.abs(z[i:j + 1])
            peak = zz.max(axis=0)
            order = [k for k in np.argsort(-peak) if peak[k] > z_thresh]
            segs.append({
                "t_start": round(float(t[i] - BLOCK_S / 2), 2),
                "t_end": round(float(t[j] + BLOCK_S / 2), 2),
                "peak_z": round(float(peak.max()), 1),
                "signals": [{"feature": FEATURE_NAMES[k], "peak_z": round(float(peak[k]), 1),
                             "sign": "+" if z[i:j + 1, k][np.argmax(zz[:, k])] > 0 else "-"} for k in order],
            })
        i = j + 1
    return segs


def score(run_dir: Path, baseline: dict, z_thresh: float = DEFAULT_Z, min_s: float = DEFAULT_MIN_S) -> dict:
    run_dir = Path(run_dir)
    t, phase, z, _ = zscores(run_dir, baseline)
    segs = segments(t, z, z_thresh, min_s)
    ctrl = _controller(run_dir)
    return {
        "run": str(run_dir),
        "controller": ctrl,
        "baseline_controllers": baseline.get("controllers", []),
        "controller_mismatch": ctrl not in baseline.get("controllers", [ctrl]),
        "z_threshold": z_thresh,
        "min_s": min_s,
        "anomalous": bool(segs),
        "max_z": {n: round(float(np.abs(z[:, j]).max()), 2) for j, n in enumerate(FEATURE_NAMES)},
        "segments": segs,
        "_t": t, "_z": z, "_phase": phase,
    }


def _plot(res: dict) -> str:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from .report import INK, INK_2, SERIES, SURFACE, _style

    t, z = res["_t"], np.abs(res["_z"])
    shown = sorted(range(len(FEATURE_NAMES)), key=lambda k: -z[:, k].max())[:4]
    fig, ax = plt.subplots(figsize=(9, 3.6), facecolor=SURFACE)
    for s in res["segments"]:
        ax.axvspan(s["t_start"], s["t_end"], color="#eb6834", alpha=0.12, linewidth=0)
    for i, k in enumerate(shown):
        ax.plot(t, z[:, k], color=SERIES[i % len(SERIES)], linewidth=1.6, label=FEATURE_NAMES[k])
    ax.axhline(res["z_threshold"], color=INK, linewidth=1, linestyle="--")
    ax.set_yscale("symlog", linthresh=1)
    _style(ax, "|z| (symlog)")
    ax.set_xlabel("time (s)", color=INK_2, fontsize=9)
    ax.set_title("Largest feature scores; shaded = flagged", loc="left", color=INK, fontsize=11)
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, facecolor=SURFACE)
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def public(res: dict) -> dict:
    return {k: v for k, v in res.items() if not k.startswith("_")}


def write_outputs(res: dict, baseline: dict, plots: bool = True) -> Path:
    run_dir = Path(res["run"])
    (run_dir / "anomaly.json").write_text(json.dumps(public(res), indent=2), encoding="utf-8")
    rows = "".join(
        f"<tr><td>{s['t_start']:.1f} to {s['t_end']:.1f} s</td><td>{s['peak_z']:.1f}</td><td>"
        + "<br>".join(f"{html.escape(g['feature'])} ({g['sign']}, z {g['peak_z']:.1f}): "
                      f"<span class='m'>{html.escape(baseline['features'][g['feature']]['description'])}</span>"
                      for g in s["signals"])
        + "</td></tr>" for s in res["segments"])
    verdict = (f"{len(res['segments'])} anomalous segment(s)" if res["segments"]
               else "No anomalies above the threshold")
    warn = ("<p><b>Warning:</b> this run's controller differs from the baseline's; expect false alarms.</p>"
            if res["controller_mismatch"] else "")
    img = f"<img alt='feature scores' src='data:image/png;base64,{_plot(res)}'>" if plots else ""
    page = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Anomaly report</title>
<style>
body{{font-family:system-ui,sans-serif;background:#fcfcfb;color:#0b0b0b;max-width:1000px;margin:24px auto;padding:0 16px}}
table{{border-collapse:collapse;width:100%}} th,td{{text-align:left;padding:6px 10px;border-bottom:1px solid #e4e3df;font-size:13px;vertical-align:top}}
th{{color:#52514e;font-weight:500}} .m{{color:#52514e}} img{{max-width:100%}}
</style></head><body>
<h1>Anomaly report</h1>
<p class="m">{html.escape(str(run_dir))}<br>Baseline from {baseline['n_runs']} nominal run(s)
({html.escape(', '.join(baseline.get('controllers', [])))} controller). A segment is flagged where a feature's
|z| stays above {res['z_threshold']:g} for at least {res['min_s']:g} s.</p>
{warn}<h2>{verdict}</h2>
{img}
{"<table><tr><th>When</th><th>Peak |z|</th><th>Signals, largest first</th></tr>" + rows + "</table>" if rows else ""}
</body></html>"""
    out = run_dir / "anomaly.html"
    out.write_text(page, encoding="utf-8")
    return out


def default_baseline(controller: str = "builtin") -> Path:
    return Path(__file__).resolve().parent.parent / "models" / "anomaly" / f"baseline_{controller}.json"
