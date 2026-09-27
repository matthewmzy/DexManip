# DexManip · H2 + 双 Sharpa 系统辨识

开发分支：`si/h2-sharpa`。范围是双手、H2 手腕与手臂对操作任务的影响。SI 时移走桌子、双手空载、吊装固定，头、腰、腿保持不动；控制路径沿用 **工作站 DDS → Thor Sharpa bridge → Sharpa SDK**。

当前实现：离线轨迹生成、USD 几何检查、被动 DDS 记录、响应拟合及人工测量分析。**尚未实现主动运动执行器和硬件端命令超时处理，没有完成真机 SI。** 所有计划保留 `executable=false`。静止日志可用于姿态和通信检查，不能辨识运动响应。

## 换电脑继续开发

在机器人连接电脑上新建独立目录，不放进或修改同事已有工作区：

```bash
git clone --branch si/h2-sharpa --single-branch git@github.com:matthewmzy/DexManip.git ~/DexManip
cd ~/DexManip
bash scripts/setup_env.sh
source scripts/activate.sh
python scripts/doctor.py
bash experiments/20260927_h2_sharpa_sysid/bashes/run_tests.sh
bash experiments/20260927_h2_sharpa_sysid/bashes/run_demo.sh
```

环境入口只创建本目录的 `.venv`。需要 Python 3.10–3.12，推荐 3.11；选择其他解释器时使用 `PYTHON=/path/to/python bash scripts/setup_env.sh`。没有修改现有 `tv` 环境。所有 bash 入口使用当前 `python`，也可通过 `SI_PYTHON` 指定解释器。离线部分不需要 Isaac Sim、GPU、Sharpa SDK 二进制或完整 DexRL 项目。

### 导入现有机器人模型

这个仓库是公开仓库；现用 USD 来自私有上游，**模型及派生几何不放 Git**。从原开发电脑单独复制 `h2_sharpa_si_assets_20260927.tar.gz` 到新电脑，然后导入，例如文件在 `~/Downloads/` 时：

```bash
python scripts/transfer_assets.py import ~/Downloads/h2_sharpa_si_assets_20260927.tar.gz
python scripts/doctor.py --usd
bash experiments/20260927_h2_sharpa_sysid/bashes/run_usd_tests.sh
bash experiments/20260927_h2_sharpa_sysid/bashes/run_usd_plan.sh
```

导入工具检查全部六个文件的 SHA256、大小和归档成员，原 USD 与几何缓存字节保持不变；旧清单里的源路径仅作来源记录，不要求新机器存在原目录。模型放入已忽略的 `assets/h2_sharpa/`。**只有 git clone、尚未导入模型时，USD 规划不可用**；普通测试会明确跳过实际模型测试，完整 USD 测试入口会报缺少资产，不会把跳过视为通过。

原机可以重新打包：

```bash
python scripts/transfer_assets.py pack --out ../DexManip-transfer/h2_sharpa_si_assets_20260927.tar.gz
```

输出文件已存在时不会覆盖。此工具只处理本地文件，不上传模型、不连接机器人。模型包也不要提交到 GitHub Release。

### 在已连接机器人的电脑上记录

先在本仓库环境中安装审计版本的 DDS 依赖：

```bash
bash scripts/setup_dds.sh
source scripts/activate.sh
ip -br link
python scripts/doctor.py --dds --interface YOUR_DDS_INTERFACE
bash experiments/20260927_h2_sharpa_sysid/bashes/run_record_readonly.sh YOUR_DDS_INTERFACE
```

将 `YOUR_DDS_INTERFACE` 替换为**运行采集脚本的这台电脑**接机器人的网卡名。`doctor` 只检查本地环境和网卡存在性，不证明 DDS 话题可达。安装脚本按[官方源码安装方法](https://github.com/eclipse-cyclonedds/cyclonedds-python/blob/0.10.2/README.md#installing-from-source)，把固定版本 CycloneDDS 0.10.2 原生库编译进本仓库 `.deps/`，再安装 Python SDK；需要 Git、CMake、C 编译器及 Python 开发头文件。不会用 sudo 或改系统环境。以后终端使用 `source scripts/activate.sh`，同时设置原生库路径。

安装若缺编译工具或平台依赖，保留报错处理环境问题，不修改机器人部署、不自动切换 SDK。

按现场已有流程使机器人站稳、bridge 正常运行后再录制；本脚本不会启动它们。bridge 本身启停可能发送归零，不能作为只读连接探针。默认录制 60 秒、DDS domain 0、包括 H2 全身状态；不创建 publisher、不归零。`Ctrl-C` 仅停止记录。输出在 SI 实验的 `assets/record_<时间>/`，默认不进 Git，实测数据需另行备份。

取得真实静止姿态后：

```bash
bash experiments/20260927_h2_sharpa_sysid/bashes/run_usd_plan.sh \
  --from-run experiments/20260927_h2_sharpa_sysid/assets/record_TIMESTAMP
```

这一步固定实测头腰腿角度，检查从记录姿态进入候选姿态的路径及逐关节小幅范围，仍不发送动作。

## 继续开发的位置

| 内容 | 入口 |
| --- | --- |
| 论文与仓库调研 | [research.md](experiments/20260927_h2_sharpa_sysid/research.md) |
| 实际 DDS 接口、单位、控制限制 | [control_audit.md](experiments/20260927_h2_sharpa_sysid/control_audit.md) |
| CLI、采集与分析细节 | [usage_notes.md](experiments/20260927_h2_sharpa_sysid/usage_notes.md) |
| 迁移验证与待办 | [log.md](experiments/20260927_h2_sharpa_sysid/log.md) |
| 本地环境诊断 | `scripts/doctor.py` |
| USD 依赖实现 | `vendor/usd_geometry/` |

下一步开发顺序：真机只读采集 → 核对起始姿态与通信 → 完成 bridge 端超时行为、控制权管理及执行器 → 单关节小幅实测 → 扩展双手/双臂并做独立轨迹验证。上游 bridge 会持续重发最后目标，停止本机发送不等于停止机器人；不能直接给当前计划加一个“执行”按钮绕过这项缺口。

先验参数不是本机测量；一阶拟合得到等效增益、偏置、响应时间和延迟，不能直接当作惯量、摩擦或真实 PD 参数。[第三方来源与归属](THIRD_PARTY_NOTICES.md)。
