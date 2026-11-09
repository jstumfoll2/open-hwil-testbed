# JSBSim models

The default vehicle is **`F450-hil`**: the public F450 quadcopter that ships with
JSBSim, with one change. Its aerodynamics file (all zeros upstream, so wind has
no effect) is replaced by [`F450-hil/Aero.xml`](F450-hil/Aero.xml), which adds a
simple isotropic drag term.

The other F450 files are **not copied into this repo**. At run time
`hwil.models.prepare_aircraft()` copies them from your installed `jsbsim`
Python package into a cache folder (`~/.cache/hwil-testbed/jsbsim/` by default,
or `$HWIL_CACHE`) and overlays the files in this folder. That keeps the JSBSim
data files under their own license (LGPL-2.1, see the JSBSim project) and this
repo under Apache-2.0.

To fly the unmodified upstream model, set `vehicle.model: F450` in a scenario.

Motor order (matches the PX4 quad-x convention):

| Index | Position | Spin |
|---|---|---|
| 0 | front right | CCW |
| 1 | aft left | CCW |
| 2 | front left | CW |
| 3 | aft right | CW |
