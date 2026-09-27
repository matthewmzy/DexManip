#!/usr/bin/env bash
# Source this file: activates only the checkout venv and optional local CycloneDDS.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Use: source scripts/activate.sh" >&2
  exit 2
fi
_dexmanip_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$_dexmanip_root/.venv/bin/activate" ]]; then
  echo "Run bash scripts/setup_env.sh first." >&2
  unset _dexmanip_root
  return 1
fi
source "$_dexmanip_root/.venv/bin/activate"
if [[ -d "$_dexmanip_root/.deps/cyclonedds/install" ]]; then
  export CYCLONEDDS_HOME="$_dexmanip_root/.deps/cyclonedds/install"
fi
unset _dexmanip_root
