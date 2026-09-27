#!/usr/bin/env bash
# Offline fixed-body, empty-workspace SI planning. Output: assets/usd_plan_<timestamp>/.
set -euo pipefail
si_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "${SI_PYTHON:-python}" -B -u "$si_dir/scripts/si.py" usd-plan \
  --out "$si_dir/assets/usd_plan_$(date +%Y%m%d_%H%M%S)" \
  --amplitude-deg 2 --step-deg .25 --seconds 120 "$@"
