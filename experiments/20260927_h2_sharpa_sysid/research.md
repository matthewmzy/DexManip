# H2 + 双 Sharpa：RL sim-to-real 文献与系统辨识计划

调研日期：2026-09-27。时间窗为 **2024-09-27 至 2026-09-27**，按 arXiv 首次提交日期筛选，并检查作者项目与仓库现状。SI 在本文指 System Identification（系统辨识）。用户未指定论文，因此按公开论文、作者项目页与官方代码检索；不是完整文献计量调查。以下“论文事实”和“本项目建议”分开，所有数值先视为对应论文配置，不能当作这台 H2 的实测参数。

## 结论

应先辨识**实际部署控制链的等效响应**：关节顺序/单位/符号、左右手零位与可用范围、位置目标到实际位置的增益/动态/延迟、滤波与限速、通信和感知时序；随后处理腕部安装与负载、指尖接触与物体属性。只拟合几个 PD 增益不足以覆盖 H2 双手系统。

能做一个本地统一入口，组织配置、采集、激励生成、离线拟合、验证和仿真参数导出；**不能靠一段空载运动脚本把质量、惯量、真实关节摩擦、指尖接触参数、相机外参和剪刀铰链全部唯一辨识出来**。这些项目需要不同实验条件，部分需要力传感器、标定板或手工测量。报告把“可从日志估计”和“仍需实验/设备”明确列出。

## 确认使用 Sharpa 真机的相关工作

以下平台均不同于 H2。论文已有 Sharpa 的实机结果，不意味着整套策略或标定参数可以直接部署到 H2。

| 工作 | 真实平台 / 任务 | 对本项目最有用的部分 |
|---|---|---|
| [WM-Craftnet](https://arxiv.org/html/2609.07002v1) | 22-DoF Sharpa Wave、腕部 L515；掌内旋转 | 附录 A.3 直接给出正弦激励与仿真 PD 拟合；[官方代码](https://github.com/sharpa-robotics/WM-Craftnet) |
| [SimToolReal](https://arxiv.org/html/2602.16863v1) | KUKA iiwa14 + 左 Sharpa；抓取、重定向、工具轨迹 | Appendix A-C：gains/armature/damping、FIFO 延迟、噪声、EMA；[作者项目页](https://simtoolreal.github.io/) |
| [DEX-X](https://arxiv.org/html/2609.07747v2) | FR3 + Sharpa Wave，含双臂；视觉触觉操作 | 附录 D/H/J：真机关节范围、随机化与臂动态标定；[作者项目页](https://dexx-code.github.io/dexx-code/) |
| [Tacmap](https://arxiv.org/html/2602.21625v1) | SharpaWave + DTC 视触觉；掌内旋转 | 真实触觉到几何形变的标定、外部力真值；不是整机动力学 SI |
| [ADEPT](https://arxiv.org/html/2608.19182v1) | Flexiv Rizon + Sharpa（另有 KUKA + Allegro）；抓取重定向插接 | 几何 fabric 控制接口一致性、TacMap 输入、ADR |
| [POISE: Learning In-Hand Object Reaching to General 6D Poses](https://arxiv.org/html/2609.13761v1) | Sharpa Wave + D435；掌坐标系内 6D 到达 | 相机到手外参、感知延迟/掉帧、重心与惯量随机化 |
| [Assembling Two Parts in One Hand](https://arxiv.org/html/2609.10137v1) | FR3 固定 Sharpa Wave + D435；瓶/注射器/笔的插接 | 指间协作、接触几何、历史观测；不是螺纹或过盈配合验证 |
| [Morphometric Imitation](https://arxiv.org/html/2609.28660v1) | KUKA iiwa14 + Sharpa Wave + L515 | 形态/接触一致性、PD/摩擦/CoM 随机化；每类物体独立策略 |
| [CHORD](https://arxiv.org/html/2607.00033v2) | Dexmate 双臂 + 双 Sharpa + Vicon；刚体/铰接物操作 | 附录 F：双手部署、坐标系标定、状态时间戳与失效处理 |
| [MoDE-VLA / IMCopilot](https://arxiv.org/html/2603.08122v1) | SharpaNorth + 双 SharpaWave；RL 原子技能辅助采集并供 VLA 调用 | 确有仿真 RL 与真机技能，不是纯遥操作；应区分低层 RL 与上层 VLA 训练 |

### 时间窗复核与 2025 年检索结果

以下日期均从对应 arXiv 的 `Submitted on` / `Submission history` 核对，不从论文编号或搜索引擎“发布时间”推断。全部早于本次截止日期；例如 `2609.28660` 的首次提交明确为 **2026-09-23**，不是未来论文。

| 工作 | 首次提交 | 本报告技术细节查阅版本 |
|---|---|---|
| [SimToolReal](https://arxiv.org/abs/2602.16863) | 2026-02-18 | v1；arXiv 当前为 2026-02-24 的 v2 |
| [Tacmap](https://arxiv.org/abs/2602.21625) | 2026-02-25 | v1；arXiv 当前为 2026-05-12 的 v2 |
| [MoDE-VLA / IMCopilot](https://arxiv.org/abs/2603.08122) | 2026-03-09 | v1 |
| [CHORD](https://arxiv.org/abs/2607.00033) | 2026-06-22 | v2，修订 2026-08-14；编号前缀与首次提交月份不同，以元数据为准 |
| [ADEPT](https://arxiv.org/abs/2608.19182) | 2026-08-19 | v1 |
| [WM-Craftnet](https://arxiv.org/abs/2609.07002) | 2026-09-07 | v1 |
| [DEX-X](https://arxiv.org/abs/2609.07747) | 2026-09-07 | v2，修订 2026-09-09 |
| [单手双物体装配](https://arxiv.org/abs/2609.10137) | 2026-09-09 | v1 |
| [POISE](https://arxiv.org/abs/2609.13761) | 2026-09-12 | v1 |
| [Morphometric Imitation](https://arxiv.org/abs/2609.28660) | 2026-09-23 | v1 |

针对 2024–2025 又用 Sharpa/SharpaWave + RL/reinforcement learning/sim-to-real + 年份检索，并回查候选论文平台。本次**没有核实到该段时间内独立公开、明确使用 Sharpa 真机的 RL 论文**，这不构成“绝对不存在”的证明。[WM-Craftnet 作者项目页](https://wmcraftnet.github.io/) 回顾了 ICRA 2025/IROS 2025 演示，但论文首次公开是 2026 年，不能倒算成 2025 论文。[sharpa-rl-lab 提交记录](https://github.com/sharpa-robotics/sharpa-rl-lab/commits/main/) 当前可见提交为 2026-04-03 与 2026-06-15；LICENSE 的 Copyright 2025 也不等于代码在 2025 年已公开。

### 作者仓库与实际可用内容（2026-09-27 只读核对）

“有代码”指已看到源码入口和说明，**不代表本次已运行复现、所有权重齐全或可以不改就接 H2**。GitHub API 返回匿名速率限制 403 后，改为读取 GitHub 页面、目录和 raw README；没有向任何仓库写入。下表 SHA 取当时默认分支页面 `currentOid`，用于固定本次观察。

| 工作 | 作者/官方仓库或发布入口 | 当时可用内容与缺口 | 已核对 commit |
|---|---|---|---|
| WM-Craftnet | [sharpa-robotics/WM-Craftnet](https://github.com/sharpa-robotics/WM-Craftnet) | 训练脚本、Isaac Gym env、`deploy/` 真机示例、[HF checkpoint](https://huggingface.co/SharpaIT/WM-Craftnet)。README 提醒公开 checkpoint 是 proprioception+depth 的 legacy 配置，不能默认等于全文全部模态实验 | [`8d740b8a11cf`](https://github.com/sharpa-robotics/WM-Craftnet/tree/8d740b8a11cf850e601b717927007b1d61175a00) |
| SimToolReal | [tylerlum/simtoolreal](https://github.com/tylerlum/simtoolreal) | Isaac Gym/Isaac Sim 训练、policy 下载、DexToolBench、`deployment/sharpa_node.py`、日志/轨迹回放；真机感知与机器人栈仍有外部依赖。非作者复现仓库不替代本入口 | [`313d5aea1f50`](https://github.com/tylerlum/simtoolreal/tree/313d5aea1f507c6cfe097b672b62945d7b0bbff5) |
| Tacmap | [sharpa-robotics/sharpa-tacmap](https://github.com/sharpa-robotics/sharpa-tacmap) | Isaac Lab 触觉传感器、压入测试、仿真数据导出；README 未提供完整真实三轴台采集、真实图像校准训练或全文 MuJoCo 复现链 | [`e4f8827407e1`](https://github.com/sharpa-robotics/sharpa-tacmap/tree/e4f8827407e15625368ef0b9cf5a72c44304e16b) |
| DEX-X | [作者项目页](https://dexx-code.github.io/dexx-code/) | 明示 **Code soon**；未核实到作者公开训练/部署源码，项目页本身不是算法仓库 | 未有可核对算法 commit |
| ADEPT | [作者项目页](https://adept-dexterity.github.io/) | 有论文、视频、网页伪代码；Code 为不可点击文本，未核实到作者公开训练/真机仓库 | 未有可核对算法 commit |
| POISE | [作者项目页](https://junxiaolin.github.io/poise-website/) | 明示 **Coming soon**，承诺后续发布；未核实训练/部署源码 | 未有可核对算法 commit |
| 单手双物体装配 | [LTBGbird/in-hand-assembly](https://github.com/LTBGbird/in-hand-assembly) | **占位仓库，只有 README**；勾选列表尚未发布训练与真机代码，作者目标为 2026 年 11 月前 | [`b90909dce425`](https://github.com/LTBGbird/in-hand-assembly/tree/b90909dce4256e77a05a250b349e1c92aa234e6b) |
| Morphometric Imitation | [tsadja/morphometric](https://github.com/tsadja/morphometric) | README + assets；明示 **Code will be released soon**，不视为已开源完整 retargeting/RL/student 流程 | [`dde70d61e15e`](https://github.com/tsadja/morphometric/tree/dde70d61e15e2518f1e3a1f9757ac65504cbcb93) |
| CHORD | [nvidia-isaac/video_to_data/robotic_grounding](https://github.com/nvidia-isaac/video_to_data/tree/main/robotic_grounding) | 作者页面链接到此；Sharpa retargeting、Isaac Lab PPO、数据/回放和示例，当前还含 FlashCHORD。已核实训练，**尚未核实论文完整 Dexmate+Vicon 真机 bridge 已随仓库提供** | [`33129dd0f2d2`](https://github.com/nvidia-isaac/video_to_data/tree/33129dd0f2d2dcfd1164d43fd076542660756ed2/robotic_grounding) |
| MoDE-VLA / IMCopilot | [论文入口](https://arxiv.org/abs/2603.08122) | 本次未核实到与论文完整对应的作者算法仓库；不能把官方旋转 demo 等同为其完整 VLA/IMCopilot 实现 | 未有可核对算法 commit |
| 官方 RL demo（非独立论文） | [sharpa-robotics/sharpa-rl-lab](https://github.com/sharpa-robotics/sharpa-rl-lab) | grasp cache、训练、adaptation、仿真播放、真机 `deploy.py`、预训练目录；支持两种触觉部署方式 | [`95ccda3d9488`](https://github.com/sharpa-robotics/sharpa-rl-lab/tree/95ccda3d948801bb5da4cb7ffea766e03067a63b) |

若现在就要改代码复现，最直接的三个入口是 **sharpa-rl-lab → WM-Craftnet → SimToolReal**；前两个用于手内技能和接口对齐，后一个用于手臂协同工具操作。CHORD 用于参考动作驱动的双手任务，但还需单独审核真机适配层。

补充范围：[ConTrack](https://arxiv.org/html/2606.03177v2) 的附录有 **Sharpa 仿真 benchmark**，但其真机可行性试验是 xArm7+xHand，不能计为 Sharpa 实机 sim2real 证据；作者[仓库](https://github.com/EmptyBlueBox/ConTrack) 有 Sharpa 资产/任务可参考。BORA 虽有 Sharpa 研究人员参与，论文硬件是 DexHand021，故不纳入。纯遥操作、单纯 VLA 模仿学习与仅引用 Sharpa 的文章也不因关键词而纳入主表。

### 直接支持 SI 设计的证据

**WM-Craftnet，A.3 / Table 7。** 原文短摘：“exciting the real hand with 1 Hz sinusoidal action commands”。作者比较同一输入下真机与仿真轨迹的幅相，调整仿真 Kp/Kd，报告平均 sim-real 跟踪误差 <0.2°；这不是绝对编码器精度或所有工况误差上界。论文手控/触觉/相机 10 Hz，physics 60 Hz；PD scale [0.8,1.2]、friction scale [0.5,2]。未公开足以证明惯量、摩擦、延迟分别可辨识的多工况实验。[论文附录](https://arxiv.org/html/2609.07002v1)

**SimToolReal，Table I / A-C。** 原文短摘：“match the simulation’s joint gains, armature, and damping”。physics/control 为 120/60 Hz；action/observation delay 最多 3 steps，object-pose delay 最多 10 steps；姿态估计 30 Hz。若 steps 按 policy step 解释，分别约 50/167 ms，落地前仍应核对队列所在循环。对象观测噪声 1 cm、5°，关节速度噪声 0.1 rad/s，并用 EMA 平滑。并未给出可直接复现的全参数 SI 算法；评价重点是工具轨迹进度，不是所有功能性工具任务完成率。[论文](https://arxiv.org/html/2602.16863v1)

**DEX-X，D/H/J。** 原文短摘：“recording commanded and achieved positions”。逐个 FR3 关节用 0.2 Hz、0.3 rad 正弦标定，作者调低 joints 1–3 仿真 damping 约 40%，达到互相关 >0.996、lag <10 ms；这些是 FR3 结果。30 Hz 部署；hand PD ×[0.5,2]、arm PD ×[0.8,1.2]、action delay 0–3 steps。论文还用实测可达范围收紧 URDF 限位。触觉力有乘性噪声、5% 单指 dropout。对 H2 应复用方法、重新测量参数。[论文](https://arxiv.org/html/2609.07747v2)

**Tacmap，III-C / V。** 原文短摘：“high-precision 3-axis motion stage”。三轴台编码器、已知压头几何与接触姿态生成形变真值；外部高精度力传感器另用于力回归。真实 DTC 图像被映射到仿真共享的几何形变表示。论文没有支持“从关节位置日志自动得到准确触觉力”的结论；要复现完整标定，需治具、几何真值和相应触觉流。[论文](https://arxiv.org/html/2602.21625v1)

### 其他工作的边界

- **ADEPT** 使用同一 fabric action interface 连通仿真与真机，Sharpa 触觉采用 TacMap；Flexiv-Sharpa 方圆插接报告 8/10。Table 8 随机化 stiffness/damping ×[0.5,2] 与物体质量、摩擦。文中部署附录的 1 kHz KUKA、333 Hz Allegro 描述不能当作 Sharpa 或 H2 频率。它强化了“辨识控制栈”的必要性，没有提供 H2 SI 配方。[论文](https://arxiv.org/html/2608.19182v1)
- **POISE** 在 20 Hz 控制下给出 pose delay 0–3 steps（0–150 ms）、3% hold-last dropout；PD/摩擦 ×[0.5,2]，CoM 每轴最多 5 mm，惯量 multiplier LogU[1,4]。真实物体 pose 通过标定过的 camera-to-hand transform 转为掌坐标系。这些范围用于鲁棒训练，不能代替本机的基准测量。[论文](https://arxiv.org/html/2609.13761v1)
- **单手双物体装配** 采用 15 Hz、0.1 rad 增量位置目标；摩擦和 PD ×[0.5,2]，action delay 0–3 steps。SDF 接触和 solver 参数影响装配结果。真机是开始时人工放好零件的插接片段；不能由此推断自主拿起零件或精确螺纹/过盈配合已解决。[论文](https://arxiv.org/html/2609.10137v1)
- **Morphometric Imitation** 随机化 PD、手/物体摩擦、质量、CoM、各向异性尺寸及外力；真机是每类别策略的 KUKA-Sharpa 测试。核心帮助是接触位置和几何一致性，而非新的电机辨识程序。[论文](https://arxiv.org/html/2609.28660v1)
- **CHORD** 的真机附录最接近“双臂双 Sharpa”集成问题：Dexmate 61 维命令含双臂、躯干和双手；policy 20 Hz，state bridge 100 Hz，命令零阶保持最高 500 Hz。Vicon 用时间戳拒绝超过 0.25 s 的旧帧，并显式标定 policy/environment、robot base、Vicon、object 坐标系。该 0.25 s 是其部署配置，不是本机应照抄的安全阈值；全身 G1 仿真扩展也不等于 H2 真机验证。[论文附录 F](https://arxiv.org/html/2607.00033v2)
- **MoDE-VLA / IMCopilot** 的 III-C 明确：低层手内技能以 RL 在仿真学习，观察包含 3 帧本体/触觉力/目标轴历史，输出增量位置目标；随机化物体尺度、质量、摩擦、CoM、重力和 PD。RL 技能既用于辅助遥操作，也能被 VLA 调用，因此值得纳入“Sharpa 做 RL”，但整套削苹果策略不能称为完全由 sim-to-real RL 从零学得。[论文](https://arxiv.org/html/2603.08122v1)

### 官方参考实现：有用，但不能当作本机测量

[sharpa-rl-lab](https://github.com/sharpa-robotics/sharpa-rl-lab) 是官方掌内旋转 demo。其 [training config](https://github.com/sharpa-robotics/sharpa-rl-lab/blob/95ccda3d948801bb5da4cb7ffea766e03067a63b/rl_isaaclab/tasks/inhand_rotate/sharpa_wave_env_cfg.py) 使用 240 Hz physics、decimation 12，即 20 Hz 控制；随机化 PD 和摩擦 ×[0.5,2]；指腹/金属/物体 base friction 分别 0.8/0.1/0.5；CoM ±1 cm；质量 0.01–0.25 kg；关节范围 scale 0.9。这些只是该任务的 nominal 与随机化设置。

其 [deployment config](https://github.com/sharpa-robotics/sharpa-rl-lab/blob/95ccda3d948801bb5da4cb7ffea766e03067a63b/rl_isaaclab/tasks/inhand_rotate/sharpa_wave_deploy_env_cfg.py) 另有 `current_coef=0.3`、`speed_coef=0.5`，并明确指出 `control_freq=20` 只是循环 sleep，实际频率并不精确；真机最终限位取配置与 firmware 范围的交集。触觉还经过 force_scale、threshold。**SI 日志必须保存这些设置和 firmware/SDK 版本；换设置之后应重新验证。** 对应本次核对的固定 commit `95ccda3d948801bb5da4cb7ffea766e03067a63b`。

### 不应误标为 Sharpa 的背景工作

[DeXtreme](https://arxiv.org/abs/2210.13702) 的真机是 Allegro；[Beyond Binary](https://arxiv.org/html/2605.28812v1) 是带 XELA uSkin 的 16-DoF Allegro，包含触觉方向校准与视觉测延迟。它们可提供思路，但传感器校准方法、力/电流参数不能直接移植 Sharpa。厂商新闻、二手笔记仅用于发现论文，上述数值均回到作者论文或官方代码核对。

## 本项目建议：应做哪些 SI

下表为工程建议，**不是某篇论文声称已经完成的 H2 实验**。优先级针对桌面抓取/工具操作；若目标包括抓剪刀任务，额外关注指环几何、剪刀重心和铰链。

| 优先级 | 项目 | 输入/实验与可观测量 | 产物 / 自动化边界 |
|---|---|---|---|
| P0 | 通道契约与状态有效性 | 只读检查 2×22 手关节与 H2 臂关节的 names/order、单位、sign、topic、device ID、缺帧与时间戳 | 显式 mapping；拒绝 NaN、未初始化、陈旧、错维度数据。静止时无法凭空验证运动方向 |
| P0 | 时间链 | monotonic command issue/state receive，设备采样时间/序号若可得；记录双手及臂 | 控制周期、jitter、状态 age、重复帧/丢帧率、有效往返响应延迟。没有时钟同步时不能拆分单向网络延迟 |
| P0 | 可用范围与基准姿态 | 制造商/已审核限位、小范围人工核验、FK 与可见指尖 | 左右独立 offset、方向、可用区间、collision margin。禁止用撞机械限位的办法自动探索范围 |
| P1 | Sharpa 闭环动态 | 每次单关节，小幅慢阶跃/多频正弦/三角波；记录所有关节，正反向、多个工作点 | 静态 gain/bias、响应时间、有效 delay、速度饱和、回差、跨关节串扰、预测残差；先拟合有效模型 |
| P1 | H2 臂与手负载 | 固定实际部署模式和腕部负载，逐臂/逐关节受控激励；q/dq/tau_est（若可信） | 臂位置闭环、重力残差、载荷相关变化；不能默认启停底层平衡/腿部控制 |
| P1 | 腕到掌 / camera 到 base/hand / 桌面 | CAD 尺寸 + 标定板/视觉测量，多个关节姿态交叉验证 | 刚体变换、指尖 FK 偏差、桌高/法向与置信度；位置日志不包含外部几何真值 |
| P1 | 目标物 / 抓剪刀 | 电子秤、尺寸/mesh、悬挂或平衡实验；静态抓取与受控滑移 | 质量、CoM；足够激励下估计惯量。只抓闭合剪刀可先固定铰链；要开合/剪切则另测铰链力矩、阻尼、限位和材料负载 |
| P1–P2 | 指尖与材料接触 | 已知法向载荷、平面/圆柱治具、切向拉力或倾斜实验 | 接触原点、静/动摩擦、滑移阈值、形变与法向载荷关系；需要力或重量真值 |
| P2 | 触觉 / 视觉观测模型 | 静止与已知加载、温度变化、遮挡、运动同步记录 | 每指零偏/噪声/漂移、量程、dropout、延迟、阈值、坐标系；图像/形变标定需要外部参考 |
| P2 | 双手、双臂联合效应 | 单手标定后，两手同时运行的计时/供电/负载测试 | 带宽竞争、不同步、共用资源和姿态依赖误差；不能把左手模型镜像后当成右手实测 |

第一轮完成 P0 和两手空载 P1，就能开始有依据地搭建 RL actuator/delay/observation model；完整接触辨识可按任务分阶段补齐。H2 在支撑/吊装/站立时动力学条件不同，应按计划部署模式记录，不能把“手放桌上”得到的模型当作整机双手运动模型。

本机条件已补充：**有吊装固定，启动后自行站立；使用 PR 部署文档指定的 DDS bridge 路径。**因此先在实际站立控制开启、吊具状态明确记录的条件下测双手/双臂，保存腰和 IMU 的变化。“空载小范围实验”就是手里不拿东西、周围留空、每次一个关节小幅来回运动，测其实际响应；不是要求用户先提供完整关节角度表。

用户进一步明确：**SI 时移走桌子，头腰腿固定，只动胳膊和手；用项目现有 USD 作为几何可达性依据。**已据此实现 `usd-plan`：锁定 17 个身体关节，自动生成并检查 58 个臂/手关节的小幅范围，首次 ±2° 的 58 项均通过。几何结果与使用方式见 [log.md](log.md) 和 [usage_notes.md](usage_notes.md)，未把 USD 几何通过写成真机动力学已经辨识完成。

## 自动化程序应输出什么

建议一个入口组织分阶段命令，而非运行即让所有关节运动：

1. `inspect/record`：读取配置、版本和被动状态；生成通道与时序报告。
2. `plan`：离线生成有边界的激励，给出每关节范围、速度、加速度和总时长，输出可审查文件。
3. `collect`：通过独立硬件 adapter 执行已审核计划；每条命令与状态带时间戳和有效性；保存失败/中断原因。
4. `fit/validate`：估计等效闭环参数、holdout 预测误差、延迟与残差分布；拟合失败、低激励或时间戳异常要显式报错。
5. `export`：导出 nominal model + uncertainty，而不是复制论文随机化区间；包含数据路径、配置哈希、适用姿态/负载/速度电流系数。
6. 外部实验导入：质量、接触、外参、触觉校准用单独数据表导入，未完成字段标为 `unidentified`，不能用猜测填满后声称“全部 SI 完成”。

物理增益存在可辨识性限制。例如只给位置目标并观测位置，简化模型

\[
J\ddot q + b\dot q + \tau_c\operatorname{sign}(\dot q)
=K_p(u(t-d)-q)-K_d\dot q
\]

通常只能从响应辨识 `Kp/J`、`(b+Kd)/J` 等组合，不能唯一分开 J、Kp、Kd、黏性摩擦与 firmware 内环。ARX/一阶/二阶闭环模型参数因此不能直接标作真实 Nm/rad PD 增益；若要输出物理 Kp/Kd，需固定可信惯量或增加电流/力矩真值，并在目标模拟器里回放优化。

多频幅相、阶跃上升/稳态误差、正反向差异、残差与温度/姿态的相关性，应一起用于判断“延迟 + 一阶”是否已足够。单一 1 Hz 正弦拟合很好，只能说明该频率/姿态附近吻合。也不能用高相关系数取代幅值和绝对误差检查。

## 验收与 domain randomization

以下为拟议验证方法，阈值应由任务容差和硬件限制配置，不是假设本机已达到某论文结果：

- 分训练/验证轨迹，验证频率、幅值、姿态至少一项不同；不用拟合自身的同一条轨迹当成成功证据。
- 报每关节 MAE/RMSE/P95/P99、稳态偏差、幅值比、相位、速度饱和、窗口化 delay；同时检查非激励关节的运动。
- 对断流、重复帧、周期 deadline miss、跟踪误差、限位/电流/温度异常记录 fail reason；运动后的停止行为应与硬件控制模式一致。
- 对同一批命令在仿真回放，对齐**实际发出的目标**（含 EMA、增量积分、限速和裁剪），避免拟合尚未经过控制器处理的 policy action。
- 分开慢变偏差（episode bias）、逐帧噪声、随机丢帧、连续断流、执行延迟和传感器延迟。DR 围绕实测 nominal 和跨工况残差设置；不要盲目把所有参数扩大数倍。
- 保留 calibration revision；换 firmware、速度/电流系数、腕部安装、指腹材料或控制频率后重新做相应验证。

## 还需要的现场信息与物品

最少信息：两只手型号/序列号与 firmware/SDK 版本，实际硬件关节反馈字段，计划 policy/control/state 频率、速度和电流系数，以及 H2 操作模式及手腕负载。现场采用吊装支撑、站立控制开启、桌子移开、双手空载；先被动记录真实姿态，再用项目 USD 规划头腰腿固定时的单关节小幅范围，不要求操作者手填角度表。

物理校准最少物品：电子秤、尺寸量具、标定板和可固定视角相机、简单已知几何接触块。准确摩擦/接触刚度/力标定还需可信法向及切向力真值；Tacmap 级别原始触觉到形变标定需额外三轴台/压头或等价治具。若只使用厂商已标定触觉输出，首轮先检查零偏、噪声、延迟和少量已知载荷，不必立即重做整个 Tacmap 数据集。

本调研没有运行真机动作，也没有测出该 H2 或双 Sharpa 的参数。论文数值、推荐测试方法、代码中的示例配置必须与现场 SI 结果分别管理。

## 本分支实现状态

源码、使用入口和当前限制见 [usage_notes.md](usage_notes.md)；公开控制协议依据见 [control_audit.md](control_audit.md)。目前包含被动 DDS 记录、离线轨迹规划、闭环等效响应拟合及 USD 几何检查；**主动运动执行器、设备端断流处理和真实机器人 SI 结果尚未完成**。几何功能需要单独导入获授权的模型与缓存，代码仓库不附带这些资产。
