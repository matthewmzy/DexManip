#!/usr/bin/env bash
# Install pinned DDS dependencies inside this checkout; never starts a participant/robot.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
si_python="$repo_dir/.venv/bin/python"
dds_sha=9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd
dds_source="$repo_dir/.deps/cyclonedds/source"
dds_build="$repo_dir/.deps/cyclonedds/build"
dds_install="$repo_dir/.deps/cyclonedds/install"
if [[ ! -x "$si_python" ]]; then
  echo "Run bash scripts/setup_env.sh first." >&2
  exit 1
fi
for command in git cmake cc; do
  if ! command -v "$command" >/dev/null; then
    echo "Missing build tool: $command. Install it before DDS setup; no system packages were changed." >&2
    exit 1
  fi
done
mkdir -p "$(dirname -- "$dds_source")"
if [[ ! -d "$dds_source" ]]; then
  git clone --depth 1 --branch 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git "$dds_source"
fi
if [[ "$(git -C "$dds_source" rev-parse HEAD)" != "$dds_sha" ]] ||
   [[ -n "$(git -C "$dds_source" status --porcelain)" ]]; then
  echo "CycloneDDS source differs from the clean audited pin; refusing to reset or overwrite it." >&2
  exit 1
fi
cmake -S "$dds_source" -B "$dds_build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dds_install" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
cmake --build "$dds_build" --parallel "${SI_BUILD_JOBS:-2}"
cmake --install "$dds_build"
export CYCLONEDDS_HOME="$dds_install"
"$si_python" -m pip install -r "$repo_dir/requirements-dds.txt"
"$si_python" "$repo_dir/scripts/doctor.py" --dds
echo "For subsequent terminals: source $repo_dir/scripts/activate.sh"
