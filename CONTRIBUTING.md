# Contributing

Thanks for helping. Before anything else, read the **ground rules** in the
[README](README.md#ground-rules). Pull requests that add weapons, targeting or
guidance-to-intercept content, or anything from an employer or program, will be
closed.

## Workflow

1. Python 3.10 to 3.12, `pip install -e ".[dev]"`.
2. Add or change a scenario in `scenarios/` with at least one STL requirement.
3. `pytest` must pass. If you have PX4 built, also run
   `HWIL_PX4_DIR=... pytest -m px4`.
4. Keep runs reproducible: no unseeded randomness, no wall-clock dependence in
   `afap` mode.

## Sources

Only public, general-purpose sources. Cite them in the docs or in code
comments when a number comes from somewhere (for example a drag coefficient or
a sensor noise figure).
