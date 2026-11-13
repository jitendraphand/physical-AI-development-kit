#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT/software/shiftgate_core"
python -m pytest -q
cd "$ROOT"
python software/pybullet/simulate.py --check
python cad/verify_stls.py
