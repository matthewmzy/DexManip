# Third-party sources and redistribution boundaries

This file records provenance for the SI branch. It does not grant a license to
DexManip as a whole, change any upstream license, or grant rights to robot assets
that are not distributed here. No third-party controller or SDK binary is bundled.

## Public control code referenced by the audit

| Source | Pinned revision | Use in this branch | Upstream licensing reference |
| --- | --- | --- | --- |
| [Unitree xr_teleoperate fork](https://github.com/binliunls/xr_teleoperate) | `573175a2936253fe41aebb1d246bed47f3ced947` | Protocol and deployment audit; optional source downloader | [Apache-2.0 notice and upstream dependency credits](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/LICENSE) |
| [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python) | `e4cd91f051aaa77a70600e3d2bf7f50889db1980` | Optional installed DDS dependency; message layout references | [BSD-3-Clause](https://github.com/unitreerobotics/unitree_sdk2_python/blob/e4cd91f051aaa77a70600e3d2bf7f50889db1980/LICENSE) |
| [Sharpa Wave SDK](https://github.com/sharpa-robotics/sharpa-wave-sdk) | `1c48c44ab7aa08e495bf603e8b68dc5290d01328` / SDK 5.0.4 | README/API reference for the separate robot bridge | [Repository and distribution terms](https://github.com/sharpa-robotics/sharpa-wave-sdk/tree/1c48c44ab7aa08e495bf603e8b68dc5290d01328); no SDK binary or redistribution permission is supplied by this branch |
| [sharpa-teleop](https://github.com/isaac-for-healthcare/sharpa-teleop) | `ce5a2635f7d3ad84041d279868f009e9ceeba6c3` | Links to the publicly readable retarget/backend source | [Pinned repository](https://github.com/isaac-for-healthcare/sharpa-teleop/tree/ce5a2635f7d3ad84041d279868f009e9ceeba6c3); code is not copied into this branch |

The optional downloader writes upstream files to the ignored
`experiments/20260927_h2_sharpa_sysid/assets/upstream/` directory and does not run
them. Keep their upstream copyright, license and attribution files with any
later redistribution. Upstream downloads are not included in the Git publication.

## Sharpa numerical priors

The joint ordering and numerical vendor priors were recorded from Sharpa's
position-mode reference assets, including
[sharpa-urdf-usd-xml at 0d19cac](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml/tree/0d19cac)
and the matching
[sharpa-rl-lab revision 95ccda3d9488](https://github.com/sharpa-robotics/sharpa-rl-lab/tree/95ccda3d948801bb5da4cb7ffea766e03067a63b).
The latter carries [Sharpa Group's Apache-2.0 license](https://github.com/sharpa-robotics/sharpa-rl-lab/blob/95ccda3d948801bb5da4cb7ffea766e03067a63b/LICENSE).
Their presence does not mean they were identified on this H2 or its two hands.
The SI implementation distinguishes these priors from measured calibration results.

The original numerical extraction was maintained in the local REGRIND project's
`sharpa_official.py`. This branch retains a data-only transcription at
[assets/vendor_priors/sharpa_official.json](assets/vendor_priors/sharpa_official.json),
including the source SHA256; it does not import the REGRIND training code. REGRIND's accompanying MIT notice is retained in
[licenses/REGRIND-MIT.txt](licenses/REGRIND-MIT.txt) for that source contribution;
it is not the license of the H2/Sharpa robot asset or of the whole DexManip repository.

## Robot model and generated geometry are not distributed

The full H2 + two-Sharpa USD used for the offline checks comes from a separately
provided robot asset. Public redistribution rights for that combined asset have
not been established. This repository therefore excludes the USD, source meshes,
extracted model/mesh archives, cooked collision geometry, and local model bundles.
An operator with access to the asset can import a separately transferred local
bundle for the USD features. A local transfer does not expand the asset's license.

This boundary also excludes colleague workspace snapshots, unpublished changes,
network inventories, SSH material, robot recordings and hardware-specific filled
configurations. The public control audit cites public sources rather than those
private snapshots.

## Python dependencies and papers

NumPy, SciPy, trimesh, rtree, Matplotlib, OpenCV and the optional DDS packages are
installed separately under their own distribution licenses; they are not vendored.
The [research report](experiments/20260927_h2_sharpa_sysid/research.md) links to
papers and author repositories for attribution. Citation of a paper or repository
is not a license to redistribute its code, datasets, weights or robot models.

The native [Eclipse Cyclone DDS 0.10.2](https://github.com/eclipse-cyclonedds/cyclonedds/tree/9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd) and its Python binding are installed locally from their upstream distributions, under their EPL-2.0 / EDL-1.0 notices. They are not included in this repository.
