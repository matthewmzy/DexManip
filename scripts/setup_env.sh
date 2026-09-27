#!/usr/bin/env bash
# Create only this checkout's .venv; installs offline SI dependencies, no DDS/robot startup.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
si_python="${PYTHON:-python3}"
"$si_python" -c 'import sys; assert (3,10) <= sys.version_info[:2] < (3,13), "Use Python 3.10–3.12 (recommended: 3.11); set PYTHON=/path/to/python"'
if [[ ! -d "$repo_dir/.venv" ]]; then
  "$si_python" -m venv "$repo_dir/.venv"
fi
"$repo_dir/.venv/bin/python" -m pip install -r "$repo_dir/requirements-usd.txt"
"$repo_dir/.venv/bin/python" "$repo_dir/scripts/doctor.py"
echo "Activate with: source $repo_dir/.venv/bin/activate"
