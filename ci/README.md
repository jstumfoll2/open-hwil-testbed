# ci/

Scripts used by CI and on the bench. The GitHub Actions workflows themselves
must live in `.github/workflows/`:

- `sil.yml`: on every push and pull request, runs the pytest suite with the
  builtin controller plus a reference run and a reproduce check.
- `px4-sitl.yml`: manual and weekly, builds PX4 SITL and flies every scenario
  with PX4 in the loop.

`run_px4_sitl.sh` starts a headless PX4 SITL instance that connects to the
testbed on TCP 4560.
