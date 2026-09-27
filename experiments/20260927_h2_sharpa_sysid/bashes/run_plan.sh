#!/usr/bin/env bash
# Save a reviewed config's excitation plan locally; never execute robot motion.
set -euo pipefail
si_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -ne 2 ]]; then
  echo "Usage: bash $0 OPERATOR_CONFIG_JSON NEW_OUTPUT_JSON" >&2
  exit 2
fi
exec "${SI_PYTHON:-python}" "$si_dir/scripts/si.py" plan --config "$1" --out "$2"
