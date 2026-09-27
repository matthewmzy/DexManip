#!/usr/bin/env python3
"""Read-only local environment checks; no DDS participant, robot commands or SSH."""
import argparse
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", action="store_true")
    parser.add_argument("--dds", action="store_true")
    parser.add_argument("--interface", help="Check this machine's network interface name only")
    args = parser.parse_args()
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    results = []

    def check(name, function):
        try:
            detail = function()
            results.append(dict(check=name, ok=True, detail=detail))
        except Exception as exc:
            results.append(dict(check=name, ok=False, detail=f"{type(exc).__name__}: {exc}"))

    def package(name):
        importlib.import_module(name)
        return importlib.metadata.version(name)

    def priors():
        sys.path.insert(0, str(ROOT / "experiments/20260927_h2_sharpa_sysid/scripts"))
        import si_core
        return {"joints": len(si_core.priors()["joints"])}

    for name in ("numpy", "scipy"):
        check(name, lambda name=name: package(name))
    check("vendor_priors", priors)
    if args.usd:
        for name in ("trimesh", "rtree", "matplotlib"):
            check(name, lambda name=name: package(name))
        def geometry():
            from transfer_assets import DEST, MANIFEST, verify
            verify(DEST, json.loads(MANIFEST.read_text())["files"])
            from si_usd_geometry import SIUSDGeometry
            value = SIUSDGeometry()
            return {"movable": len(value.movable_joint_names), "fixed": len(value.fixed_joint_names)}
        check("private_USD_assets_and_loader", geometry)
    if args.dds:
        for name in ("cyclonedds", "unitree_sdk2py"):
            check(name, lambda name=name: package(name))
        def idl():
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_
            return [cls.__name__ for cls in (HandCmd_, HandState_, LowCmd_, LowState_)]
        check("DDS_IDL_import_only", idl)
    if args.interface:
        def interface():
            if Path(args.interface).name != args.interface:
                raise ValueError("Pass an interface name, not a path")
            state = (Path("/sys/class/net") / args.interface / "operstate").read_text().strip()
            return {"interface": args.interface, "operstate": state,
                    "DDS_reachability": "not_tested"}
        check("local_interface", interface)
    print(json.dumps({"python": sys.version, "executable": sys.executable,
                      "checks": results, "robot_connections_created": 0}, indent=2))
    return 0 if all(result["ok"] for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
