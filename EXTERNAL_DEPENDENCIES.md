# SI 分支依赖

| 依赖 | 来源/落点 | 缺失的影响 |
| --- | --- | --- |
| Python 3.10–3.12（推荐 3.11） | 新电脑解释器；独立 `.venv` | 无法运行脚本 |
| NumPy / SciPy | `requirements.txt` 固定版本 | 规划、拟合和离线测试不可用 |
| trimesh / rtree / matplotlib | `requirements-usd.txt` 固定版本 | USD 几何检查/渲染不可用 |
| 原 USD、导出 FK/几何、PhysX cooked cache | 私有迁移包 → `assets/h2_sharpa/`；六文件 SHA 见 `assets/h2_sharpa_manifest.json` | 无法执行 USD 规划与实际模型测试；采集/基础分析不依赖它们 |
| Unitree SDK2 Python | `requirements-dds.txt` pin `e4cd91f051aaa77a70600e3d2bf7f50889db1980` | 无法 DDS 被动记录 |
| CycloneDDS Python 0.10.2 及其原生运行库 | Python 放独立 `.venv`；原生 `9995905bce6c4cf9f740d6438bbf7fcfd1c83dfd` 编译到 `.deps/cyclonedds/install`，通过 `scripts/activate.sh` 设置路径 | DDS 导入/运行失败 |
| Thor Sharpa bridge / SDK 5.0.4 | 现场已有系统；本仓库不部署/启停 | 没有 Sharpa command/state 话题 |

SI 所需五个原 USD 几何 helper 已收进 `vendor/usd_geometry/`，先验常量收进 `assets/vendor_priors/`；没有跨原项目路径 import。不要求原 H5 轨迹、完整 REGRIND、Isaac Sim、h5py、Pinocchio 或 hpp-fcl。公开上游源码可由 `fetch_control_sources.py` 按固定版本下载至 Git 忽略目录，仅用于阅读，不运行。
