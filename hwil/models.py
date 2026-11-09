"""Locate and prepare JSBSim aircraft models.

Overlay models (a folder under ``models/jsbsim/<name>``) start from an upstream
JSBSim aircraft in the installed ``jsbsim`` package and replace some of its
files. The merged copy lives in a cache folder, so no upstream JSBSim data is
stored in this repo.
"""

from __future__ import annotations

import hashlib
import os
import shutil
from pathlib import Path

import jsbsim

REPO_MODELS = Path(__file__).resolve().parent.parent / "models" / "jsbsim"

# overlay name -> upstream aircraft it is built from
OVERLAYS = {"F450-hil": "F450"}


def jsbsim_root() -> Path:
    return Path(jsbsim.get_default_root_dir())


def cache_dir() -> Path:
    base = os.environ.get("HWIL_CACHE")
    return Path(base) if base else Path.home() / ".cache" / "hwil-testbed"


def overlay_hash(name: str) -> str:
    """Hash of the overlay files, part of the run manifest."""
    h = hashlib.sha256()
    folder = REPO_MODELS / name
    for p in sorted(folder.rglob("*")):
        if p.is_file():
            h.update(p.relative_to(folder).as_posix().encode())
            h.update(p.read_bytes())
    return h.hexdigest()


def prepare_aircraft(name: str) -> tuple[Path, str]:
    """Return (aircraft_path, model_name) ready for FGFDMExec.load_model."""
    root = jsbsim_root()
    if name not in OVERLAYS:
        if not (root / "aircraft" / name).is_dir():
            raise FileNotFoundError(f"JSBSim aircraft {name!r} not found in {root / 'aircraft'}")
        return root / "aircraft", name
    base = OVERLAYS[name]
    out_root = cache_dir() / "jsbsim" / f"{name}-{overlay_hash(name)[:12]}-{jsbsim.__version__}"
    out = out_root / name
    if not (out / f"{name}.xml").exists():
        tmp = out_root / f".{name}.tmp{os.getpid()}"
        if tmp.exists():
            shutil.rmtree(tmp)
        shutil.copytree(root / "aircraft" / base, tmp)
        (tmp / f"{base}.xml").rename(tmp / f"{name}.xml")
        for p in (REPO_MODELS / name).rglob("*"):
            if p.is_file():
                dest = tmp / p.relative_to(REPO_MODELS / name)
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(p, dest)
        try:
            tmp.rename(out)
        except OSError:  # another process won the race
            shutil.rmtree(tmp, ignore_errors=True)
    return out_root, name
