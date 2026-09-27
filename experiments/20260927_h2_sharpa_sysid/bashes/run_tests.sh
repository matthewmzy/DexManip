#!/usr/bin/env bash
# Offline unittest suite; no hardware/network dependencies.
set -euo pipefail
si_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "${SI_PYTHON:-python}" -m unittest discover -s "$si_dir/tests" -p 'test_*.py' -v
