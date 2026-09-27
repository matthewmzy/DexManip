#!/usr/bin/env python3
"""Recreate pinned public control-audit source snapshots in ../assets/upstream.

Read-only network downloads; never imports or runs the downloaded robot code.
Existing files must have the pinned content or this script fails without replacing
them. Download metadata and SHA256 hashes are written to upstream/manifest.json.
"""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1] / 'assets' / 'upstream'
XR_SHA = '573175a2936253fe41aebb1d246bed47f3ced947'
SDK_SHA = 'e4cd91f051aaa77a70600e3d2bf7f50889db1980'
SHARPA_SHA = '1c48c44ab7aa08e495bf603e8b68dc5290d01328'
SOURCES = {
    ('binliunls/xr_teleoperate', XR_SHA): [
        'teleop/robot_control/robot_hand_sharpa.py',
        'teleop/robot_control/robot_hand_sharpa_dds.py',
        'teleop/robot_control/robot_arm.py',
        'scripts/sharpa_dds_bridge.cpp',
        'scripts/sharpa_dds_bridge.py',
        'setup_h2.sh', 'setup_h2_thor.sh',
        'H2_SHARPA_GROOT17_TELEOP_DEPLOY.md',
    ],
    ('unitreerobotics/unitree_sdk2_python', SDK_SHA): [
        'unitree_sdk2py/idl/unitree_hg/msg/dds_/' + name
        for name in ('_MotorState_.py', '_LowState_.py', '_HandState_.py',
                     '_MotorCmd_.py', '_HandCmd_.py')
    ] + ['unitree_sdk2py/core/channel.py'],
    ('sharpa-robotics/sharpa-wave-sdk', SHARPA_SHA): ['README.md'],
}


def fetch(item):
    repo, sha, relative = item
    url = f'https://raw.githubusercontent.com/{repo}/{sha}/{relative}'
    with urllib.request.urlopen(url, timeout=45) as response:
        data = response.read()
    path = ROOT / (repo.split('/')[-1] + '-' + sha) / relative
    if path.exists() and path.read_bytes() != data:
        raise RuntimeError(f'Existing snapshot differs from pinned upstream: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        path.write_bytes(data)
    return {'path': str(path.relative_to(ROOT)), 'url': url,
            'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}


def main():
    items = [(repo, sha, path) for (repo, sha), paths in SOURCES.items() for path in paths]
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        records = list(pool.map(fetch, items))
    manifest = {'audit_date': '2026-09-27', 'xr_pr': 14, 'xr_head': XR_SHA,
                'xr_target_branch_tip_at_audit': 'f09e07011a44af4c2e2dc1f01bdba75ee4fa827b',
                'note': 'Public source snapshot only; deployed robot version unverified.',
                'files': sorted(records, key=lambda item: item['path'])}
    (ROOT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(f'Verified {len(records)} source snapshots; wrote {ROOT / "manifest.json"}')


if __name__ == '__main__':
    main()
