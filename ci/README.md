# CI

`shiftgate-tests.yml` is a GitHub Actions workflow.  Copy or move it to
`.github/workflows/shiftgate-tests.yml` (this requires the `workflow`
scope on the GitHub token) to enable:

- `pytest` for `shiftgate_core`
- `python software/pybullet/simulate.py --check`
- `python cad/verify_stls.py`

The kit's authoring push did not have the `workflow` scope, so the file
lives here instead of under `.github/`.
