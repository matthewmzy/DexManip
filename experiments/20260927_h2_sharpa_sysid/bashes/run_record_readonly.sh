#!/usr/bin/env bash
# Passive DDS only. Output: assets/record_<timestamp>/; never logs into/starts robot.
set -euo pipefail
si_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: bash $0 LOCAL_DDS_INTERFACE [OPERATOR_CONFIG_JSON]" >&2
  exit 2
fi
si_args=(record --interface "$1" --duration 60 --domain 0 --include-h2
  --out "$si_dir/assets/record_$(date +%Y%m%d_%H%M%S)")
if [[ $# -eq 2 ]]; then si_args+=(--config "$2"); fi
exec "${SI_PYTHON:-python}" "$si_dir/scripts/si.py" "${si_args[@]}"
