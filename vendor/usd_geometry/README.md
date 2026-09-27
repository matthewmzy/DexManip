# USD geometry helpers

Copied from the existing DexRL USD-only preflight implementation; file hashes and modifications are in `provenance.json`. The SI adapter uses the same full USD forward kinematics, official cooked vertex support sets, analytic primitives, complete visual triangles, and conservative pass/unknown/collision decisions. It does not use URDF, Pinocchio, hpp-fcl, Isaac Sim, or a replacement model.

The historical experiment-specific `main()` programs were omitted. Their unused demonstration import was removed, so SI does not need the old H5 demonstration or `h5py`. Numerical helper functions and classes are retained. The optional native BVH diagnostic in `usd_geometry27.py` is unused by SI.

Runtime assets are supplied separately under `assets/h2_sharpa/`. `model.json`, `geometry.npz`, the original `H2_with_sharpa_flat.usd`, and `cooked89/{export_manifest.json,results.json,cooked_shapes.npz}` retain their original bytes. Absolute paths inside these old manifests are provenance only: the loader resolves the selected local USD and verifies all existing content hashes. A changed or incomplete asset fails instead of falling back to another geometry.

This directory is geometry support for the SI code, not a copy of the complete historical preflight experiment.
