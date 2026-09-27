#!/usr/bin/env bash
# Full original USD visual-mesh preview, written beside a generated center.json.
set -euo pipefail
si_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -ne 2 ]]; then
  echo "Usage: bash $0 CENTER_JSON NEW_OUTPUT_PNG" >&2
  exit 2
fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "${SI_PYTHON:-python}" -B "$si_dir/scripts/render_usd_pose.py" \
  --center "$1" --out "$2"
