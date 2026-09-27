#!/usr/bin/env bash
# Offline complete suite, including the actual cached USD; no hardware/network.
set -euo pipefail
si_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONFAULTHANDLER=1
"${SI_PYTHON:-python}" "$si_dir/../../scripts/doctor.py" --usd
exec "${SI_PYTHON:-python}" -B -m unittest discover \
  -s "$si_dir/tests" -p 'test_*.py' -v
