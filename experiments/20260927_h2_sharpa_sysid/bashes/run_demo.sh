#!/usr/bin/env bash
# Local synthetic verification only; output: assets/demo_<timestamp>/.
set -euo pipefail
si_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
si_output="${1:-$si_dir/assets/demo_$(date +%Y%m%d_%H%M%S)}"
exec "${SI_PYTHON:-python}" "$si_dir/scripts/si.py" demo --out "$si_output"
