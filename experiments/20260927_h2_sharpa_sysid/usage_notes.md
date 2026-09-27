# H2 + 双 Sharpa SI：本地使用

范围：双手，以及 H2 手腕/手臂对桌面操作的影响。此工具是**SI 的本地准备、被动采集与离线分析入口**；尚未完成你的机器人标定。没有主动运动 executor，也没有修改训练配置。论文/仓库依据见 [research.md](research.md)，实际接口与远程只读核对见 [control_audit.md](control_audit.md)。

已确定的现场条件：用户有吊装固定，机器人启动后自行站立；**SI 时移走桌子，头、腰、腿保持不动，只动双臂双手**。规划锁定头腰腿的 17 个关节，保留它们作为碰撞障碍；控制路径按 PR 部署文档采用 **工作站 DDS → Thor Sharpa bridge → Sharpa SDK → 手**，不需要用户另选 direct SDK。

“空载姿态/局部运动范围”的意思：**手里不拿东西，双手及手臂周围留空，每次只让一个关节小幅来回动。**例如先测一个手指的弯曲关节，其他关节保持原位。用户无需提交关节角度表；新增 `usd-plan` 已能从项目 USD 自动选择候选姿态并检查各关节局部运动范围。

## 从现有 USD 自动生成

```bash
bash experiments/20260927_h2_sharpa_sysid/bashes/run_usd_plan.sh
```

固定使用项目 `H2_with_sharpa_flat.usd` 的已导出运动学和官方 PhysX cooked 几何，每次核对 USD/导出/缓存哈希；不解析 URDF、不启动 Isaac 或真机。入口使用当前 Python（或 `SI_PYTHON`），私有模型先按[根 README](../../README.md)导入，显式将 BLAS/OMP 设为单线程，避免该环境多线程 native 库崩溃。

输出 `assets/usd_plan_<时间>/center.json`、`usd_report.json` 和 `trials/<关节>/config.json,plan.json`。每个 trial 只允许一个关节在检验过的区间运动，另 74 个关节记录为保持位置；改变中心/被测关节或扩大范围会要求重新检查。相同轨迹可改变时间频率，但仍受目标速度上限约束。

首次实测软件结果：**58/58 个关节的 ±2° 区间通过，按不超过 0.25° 间隔共检查 990 个姿态**；头腰腿 17 个关节逐帧不变。几何覆盖为 USD 已有 cooked collider，加上 15 个无 collider 身体部件的原始可见几何补充检查。腕部凸包包含空洞导致的误报由完整原始三角面检查解决，没有删网格或新增碰撞豁免。检查是离散几何检查，未声称已完成真机响应或连续动力学验证。

默认使用项目的参考站姿生成**离线候选**，没有假设真机当前就在这个姿势，因此不认证真机进入候选姿态的路径。取得只读日志后运行：

```bash
bash experiments/20260927_h2_sharpa_sysid/bashes/run_usd_plan.sh \
  --from-run experiments/20260927_h2_sharpa_sysid/assets/record_TIMESTAMP
```

此时读取两手及 H2 全 31 个身体关节（合计 75），固定当时的头腰腿角度，并额外检查从记录姿态到候选姿态的插值路径。缺少身体反馈、双手不同步或几何不通过会保留原因，不输出对应合格轨迹。这里读取的是历史记录；后续执行器仍需比对新鲜真机状态。只读记录与 USD 规划不会写头/腰/腿指令，也不会替底层站立控制器锁住它们。

真实 USD 集成测试入口：`bashes/run_usd_tests.sh`，当前 **42 项全部通过**。普通 `run_tests.sh` 在未安装可选 USD 依赖或未导入私有模型时会明确跳过 5 项真实几何测试；完整 USD 测试入口遇到缺失则失败。预览使用 `bashes/render_usd_pose.sh <center.json> <新PNG路径>`。

## 先跑软件验证

在 DexManip 根目录运行：

```bash
bash experiments/20260927_h2_sharpa_sysid/bashes/run_tests.sh
bash experiments/20260927_h2_sharpa_sysid/bashes/run_demo.sh
```

demo 输出在 `assets/demo_<时间>/`：已知参数的合成命令/反馈、局部激励计划、`analysis/report.json`、`analysis/report.md` 与 SVG 曲线。标明 synthetic，不能当作真机参数。分析不依赖 IsaacLab 或 Sharpa SDK；需 NumPy/SciPy，版本见 requirements.txt。输入和结果默认不覆盖已有文件。

## 一个入口、分阶段操作

入口：[scripts/si.py](scripts/si.py)。所有角度输入与分析均用 **rad**，时间 s，力 N，力矩 Nm，几何 m。

| 命令 | 实现内容 | 边界 |
|---|---|---|
| `inspect` | 22×2 关节顺序、H2 臂顺序、现有官方参数来源与哈希 | 官方先验，不是本机 SI |
| `init --out <新目录>` | 未测配置模板、接触/铰链/对应点 CSV 表头 | 未测字段保留 null |
| `plan --config ... --out ...` | 逐关节正反向缓变平台、多频正弦、静置；检查范围、目标速度 | 离线计划；不检查三维碰撞、不自动执行 |
| `usd-plan --out ...` | 自动生成 USD 候选姿态和逐关节小幅轨迹；双臂/双手自碰撞及固定身体检查 | 无桌子；`--from-run` 才额外检查真实记录起点到候选姿态的路径 |
| `record --interface ... --out ...` | 同时被动订阅双手真实 cmd/state；`--include-h2` 保存臂状态、命令增益/前馈、腰与 IMU | 不建立 publisher，不启动 bridge，不连接 Sharpa SDK |
| `analyze --run ...` | 时间质量、闭环 gain/bias/tau/effective delay、留出预测、拒绝原因、曲线 | 一阶等效模型候选，不是真实 Kp/Kd/惯量 |
| `measure contact/hinge/extrinsics` | 从人工/治具实测表拟合滑移静摩擦、铰链力矩模型、3D 刚体对应变换 | 需要真实测量，不能从位置日志自动补齐 |

初始化例子（路径必须尚不存在）：

```bash
python experiments/20260927_h2_sharpa_sysid/scripts/si.py init \
  --out experiments/20260927_h2_sharpa_sysid/assets/my_calibration
```

先在 `config.json` 填真实序列号、firmware、bridge SHA、POSITION 模式、speed/current 系数。PR bridge 默认 speed=0.5/current=0.6，官方 RL demo current=0.3；必须按实际运行值记录，不能当作同一动力学条件。然后填 `plan.channel`、对应 names、全部中心位置及审核过的局部上下界；`selected_indices` 每次建议一个。改左手时同时改 names，不能只改 side 字符串。`reviewed_clearance` 只有在安装姿态/空载空间/碰撞审核后才设为 true。

默认计划数值 **2° 振幅、0.2/0.5/1 Hz、50 Hz、0.35 rad/s 目标速度上限**只是待审核的工程起点，不是厂商许可、实测安全上限或最终 RL 频率。脚本要求显式 center/limits，禁止自动寻找机械止挡。即使 joint-box 检查通过，也不表示机器人能安全执行。

```bash
bash experiments/20260927_h2_sharpa_sysid/bashes/run_plan.sh \
  experiments/20260927_h2_sharpa_sysid/assets/my_calibration/config.json \
  experiments/20260927_h2_sharpa_sysid/assets/my_calibration/right_index_plan.json
```

## 被动记录与分析

需要运行脚本的**本机**能加入已有机器人的 DDS 域，并已装审计版本的 `unitree_sdk2py`/CycloneDDS。SSH 登录成功不等于 DDS 可达；远程开发机的网卡名不能直接作为本机网卡名。脚本不安装依赖、不改路由、不启动机器人。DDS 域默认 0，可用原 Python 入口 `--domain` 调整。

已由现场人员运行控制系统后，可记录其命令与反馈：

```bash
bash experiments/20260927_h2_sharpa_sysid/bashes/run_record_readonly.sh \
  YOUR_LOCAL_DDS_INTERFACE experiments/20260927_h2_sharpa_sysid/assets/my_calibration/config.json
```

此入口记录 60 s。若只静止/没有 command，采集仍可用于检查话题，但动力学拟合会拒绝无激励/无命令的数据。Ctrl-C 只停止记录，**不会停止机器人**。脚本启动、关闭均不归零。

远程实际 `sharpa-teleop` 的 `--sdk` 与 `--dds` 是互斥模式，**现有部署入口已经选择 `--dds`**。direct `--sdk` 是源码中的可选支路，没有这些 DDS topic，且用了不同速度默认值和 SDK interpolation。可选 ZMQ action 还是平滑前的目标，不能当作 SDK 实际命令。此差异作为审计说明保留，不是当前待用户选择事项；只有将来明确改部署链时才需新增相应 logger adapter。

```bash
python experiments/20260927_h2_sharpa_sysid/scripts/si.py analyze \
  --run experiments/20260927_h2_sharpa_sysid/assets/record_TIMESTAMP \
  --channel right_hand --sample-hz 50
```

状态回调不够 50 Hz 时降低分析采样率，不能靠升采样制造测量带宽。超过三倍中位周期的断流、非单调时间、错维度、NaN、无命令重叠会拒绝；H2 tick 重复/回退也拒绝。连续日志可指定不同 `--out` 重做分析，原始文件不改。

H2 臂额外检查 `arm_sdk` weight=1、固定 kp/kd 和固定 dq/tau 前馈。正常 IK 重力前馈常随姿态变化，这种日志仍会被保存，但 q-only 拟合会拒绝；后续需在仿真回放真实 q/dq/tau/kp/kd 或多输入模型中处理。**不要为了迎合拟合程序关闭重力补偿。** 也不要用本工具启停腿部/平衡控制。

模型采用 `qdot = (gain*u(t-delay)+bias-q)/tau`，命令零阶保持。仅用前 70% 拟合，后 30% 连续自由递推，不重新注入测量状态；第二次实验还应改幅度/频率/工作点进行外部验证。`accepted` 只是软件初筛，阈值不是已验证的剪刀任务容差。一阶模型若不能解释超调、饱和、回差或负载效应，必须扩展模型/仿真回放，不能把其参数硬写成 PD。

顺序逐关节计划会先按**命令**相对初值的活动范围给各关节裁窗，前后留至少 1 s，再各自分 70/30；JSON 记录原日志对应时间/样本边界。不用 q 选择窗，也不把前一个关节的活动当作后一个关节的训练段。任意混合多次试验的日志仍应先按工况分段，该规则不是通用实验分割器。

所有时间为主机收到消息的时刻。Sharpa 当前 bridge 无真实采样时间、命令 ack，也可能重复发布旧 q；因此只能得到主机观察到的有效延迟，不能分离单向网络和电机延迟。command 重采样还引入最高约一个分析周期的时间量化，亚周期拟合数字不代表亚周期测量精度。

## 外部测量 CSV

`init` 生成空表；每种材料对、左右手/指腹、载荷条件分文件，保存测量条件。填完后例如：

```bash
python experiments/20260927_h2_sharpa_sysid/scripts/si.py measure contact \
  --input experiments/20260927_h2_sharpa_sysid/assets/my_calibration/contact.csv \
  --out experiments/20260927_h2_sharpa_sysid/assets/my_calibration/contact_fit.json
```

- `contact.csv`：已知法向力和**刚开始滑动**的切向力，至少 3 次；输出静摩擦系数。不是随便截取 sticking 数据，也不输出动态摩擦/接触刚度。
- `hinge.csv`：双向速度、独立变化的加速度和校准过的外部力矩；先去除重力/已知外载。拟合惯量、黏性与库仑项；恒速/单向/秩不足拒绝。位置误差和未校准电流不是力矩。
- `extrinsics.csv`：至少 3 组不共线的源/目标 3D 对应点，输出 target_from_source（列向量定义）。这是刚体点对应拟合，不会自动检测标定板或完成 AX=XB 手眼标定；另用未参与拟合的位姿验证。

质量、CoM、触觉标定、深度内参/视觉延迟、真实软接触、真实 simulator actuator 优化与 DR 范围仍需相应实验。JSON 数值不会自动进入 IsaacLab，需完成独立轨迹/接触验证再应用。

## 为什么没有一键自动运动

PR `573175a` 的 C++ bridge 会每 20 ms 重发最后目标，没有 command-age timeout；断开本地脚本不是可靠停止。它的 connect/disconnect 还会发全零。因此本入口只订阅现有话题，所有计划都标 `executable=false`。下一步应先把**硬件端超时停止/安全保持、命令所有权和现场急停操作**做成经过测试的控制约定，再接自动激励执行器；单加一个本机 `--yes` 不能修复该问题。

这一限制来自实际控制代码审计，而不是要用户重复批准只读/离线操作。未对同事工作区做任何改动。
