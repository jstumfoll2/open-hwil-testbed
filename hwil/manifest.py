"""Run manifest: everything needed to reproduce and audit a run.

Written as ``manifest.json`` next to the run's logs. It embeds the fully
resolved scenario, so ``hwil reproduce <manifest>`` needs nothing else.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import importlib.metadata as md
import platform
import subprocess
import sys
from pathlib import Path

from . import __version__
from .scenario import config_hash

MANIFEST_SCHEMA = 1
REPO_ROOT = Path(__file__).resolve().parent.parent


def _git(*args: str) -> str | None:
    try:
        out = subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def git_info() -> dict:
    commit = _git("rev-parse", "HEAD")
    status = _git("status", "--porcelain", "--untracked-files=no")
    return {
        "commit": commit,
        "branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(status) if status is not None else None,
    }


def _pkg_version(name: str) -> str | None:
    try:
        return md.version(name)
    except md.PackageNotFoundError:
        return None


def versions() -> dict:
    return {
        "hwil": __version__,
        "python": platform.python_version(),
        "jsbsim": _pkg_version("jsbsim"),
        "numpy": _pkg_version("numpy"),
        "pymavlink": _pkg_version("pymavlink"),
        "rtamt": _pkg_version("rtamt"),
        "matplotlib": _pkg_version("matplotlib"),
        "pyyaml": _pkg_version("PyYAML"),
    }


def platform_info() -> dict:
    uname = platform.uname()
    rt_flag = Path("/sys/kernel/realtime")
    preempt_rt = rt_flag.exists() and rt_flag.read_text().strip() == "1"
    # Deliberately no hostname or user name: manifests are meant to be published.
    return {
        "system": uname.system,
        "release": uname.release,
        "version": uname.version,
        "machine": uname.machine,
        "preempt_rt": preempt_rt or "PREEMPT_RT" in uname.version,
    }


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_config(scenario: dict, options: dict, model_hash: str | None) -> dict:
    """The part of a run that decides its result; hashed into config_hash."""
    return {"scenario": scenario, "options": options, "model_overlay_sha256": model_hash}


def build(scenario: dict, options: dict, model_hash: str | None, scenario_file: str | None,
          command: list[str]) -> dict:
    cfg = run_config(scenario, options, model_hash)
    return {
        "manifest_schema": MANIFEST_SCHEMA,
        "created_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="seconds"),
        "command": command,
        "scenario_file": scenario_file,
        "seed": scenario["seed"],
        "config_hash": config_hash(cfg),
        "git": git_info(),
        "versions": versions(),
        "platform": platform_info(),
        "options": options,
        "model_overlay_sha256": model_hash,
        "scenario": scenario,
        "controller_info": {},
        "results": {},
        "outputs": {},
    }


def describe_python() -> str:
    return sys.executable
