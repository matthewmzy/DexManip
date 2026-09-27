# H2 + Sharpa 控制接口静态审核

审核日期：2026-09-27。本公开版根据下列固定版本的公开源码说明协议、构造副作用与数据限制。未运行真机运动、启动 bridge 或安装控制栈。**源码审核不能证明当前机器人运行的二进制、固件与参数一致，首次采集仍需记录实际运行版本。** 项目运动学使用 `H2_with_sharpa_flat.usd`；模型资产单独迁移，不包含在公开仓库中。

## 固定来源

| 来源 | 版本 | 用途 |
| --- | --- | --- |
| [xr_teleoperate PR #14](https://github.com/binliunls/xr_teleoperate/pull/14) | head `573175a2936253fe41aebb1d246bed47f3ced947` | 本文实际读取的控制代码 |
| PR 目标分支 `binliu/support-agile` | 审核时分支 tip `f09e07011a44af4c2e2dc1f01bdba75ee4fa827b` | 仅记录分支身份，未把该 tip 当 PR merge-base |
| [Unitree Python SDK](https://github.com/unitreerobotics/unitree_sdk2_python/tree/e4cd91f051aaa77a70600e3d2bf7f50889db1980) | `e4cd91f051aaa77a70600e3d2bf7f50889db1980` | PR `setup_h2.sh` 指定版本；DDS IDL 与 channel 实现 |
| [Sharpa SDK](https://github.com/sharpa-robotics/sharpa-wave-sdk/tree/1c48c44ab7aa08e495bf603e8b68dc5290d01328) | `v5.0.4` / `1c48c44ab7aa08e495bf603e8b68dc5290d01328` | PR `setup_h2_thor.sh` 指定 SDK 5.0.4；未安装二进制 SDK |
| [sharpa-teleop](https://github.com/isaac-for-healthcare/sharpa-teleop/tree/ce5a2635f7d3ad84041d279868f009e9ceeba6c3) | `ce5a2635f7d3ad84041d279868f009e9ceeba6c3` | PR 配置的 retarget 依赖 pin，仅记录，未在此审核其实现 |

公开源码快照不随本仓库发布；[fetch_control_sources.py](scripts/fetch_control_sources.py) 可按表中固定版本下载到已忽略的 `assets/upstream/`，同时生成 URL 与 SHA256 清单。未下载 Sharpa 专有 SDK 二进制。PR 的部署文档明确指出启动 teleop/部署会抬臂，且进入 damping 前要扶住抬起的手臂，不能把这些入口作为只读探针。

## Sharpa：DDS 实际含义

**部署路径已由文档确定，不是待用户二选一。** [PR 部署文档 §2.2](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/H2_SHARPA_GROOT17_TELEOP_DEPLOY.md#L66-L75)要求启动并保持 Thor Sharpa DDS bridge；§3.2 的手部遥操作通过 `setup_h2.sh hand-retarget`，其[实现](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/setup_h2.sh#L799-L811) 799–811 行显式传入 `--dds`。§4 策略评估共用 §2 的 bridge 服务。故本项目 SI 采用工作站 DDS → Thor bridge → SDK → Sharpa；direct SDK 仅作为可选代码路径的差异记录。文档路径已确认与“该时刻现场服务是否已启动”是两件不同的事。

| 方向 | topic / type | 实际含义 |
| --- | --- | --- |
| command | `rt/sharpa/left/cmd`、`rt/sharpa/right/cmd` / `unitree_hg.msg.dds_.HandCmd_` | `motor_cmd[i].q` 为 **rad**；C++ bridge 只使用 q，忽略 mode、dq、tau、kp、kd |
| state | `rt/sharpa/left/state`、`rt/sharpa/right/state` / `unitree_hg.msg.dds_.HandState_` | `motor_state[i].q` 为 **degree**；bridge 只填 q |
| DDS domain | 默认 `0` | 相机默认另在 `10`，不能把相机 domain 用作手臂 domain |

依据：[DDS client](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/teleop/robot_control/robot_hand_sharpa_dds.py#L18-L73)、[C++ bridge](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/scripts/sharpa_dds_bridge.cpp#L88-L128)。手状态中的 `dq`、`ddq`、`tau_est`、温度、电压、压力/触觉和错误字段虽然在通用 IDL 中存在，但该 bridge 没有填入实际测量；默认零不能记录成“测得为零”。`SHARPA_TACTILE_PER_HAND=30` 等常量也不证明有触觉数据流。

SDK joint order（左右手相同命名顺序；左右手符号/机械零位仍须实测核对）：

| SDK index | joint suffix |
| --- | --- |
| 0, 1, 2, 3, 4 | `thumb_CMC_FE`, `thumb_CMC_AA`, `thumb_MCP_FE`, `thumb_MCP_AA`, `thumb_IP` |
| 5, 6, 7, 8 | `index_MCP_FE`, `index_MCP_AA`, `index_PIP`, `index_DIP` |
| 9, 10, 11, 12 | `middle_MCP_FE`, `middle_MCP_AA`, `middle_PIP`, `middle_DIP` |
| 13, 14, 15, 16 | `ring_MCP_FE`, `ring_MCP_AA`, `ring_PIP`, `ring_DIP` |
| 17, 18, 19, 20, 21 | `pinky_CMC`, `pinky_MCP_FE`, `pinky_MCP_AA`, `pinky_PIP`, `pinky_DIP` |

依据：[PR 的 22 DOF 定义](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/teleop/robot_control/robot_hand_sharpa.py#L13-L22)，本仓库 [si_core.py](scripts/si_core.py) 按该顺序定义手部通道。接到 Isaac 时按完整关节名映射，不能假设 articulation 内部排列就是 SDK 排列。

原生 Python 接口：

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowState_

# 只读采集：初始化指定 domain/interface，然后直接创建 Subscriber。
# 不导入、不实例化上游 Controller，不创建 Publisher。
```

发布侧 IDL 构造方式（仅作协议记录，本文不执行）：每个 `MotorCmd_(mode=0, q=target_rad, dq=0., tau=0., kp=0., kd=0., reserve=0)`，22 个组成 `HandCmd_(motor_cmd=motors, reserve=[0,0,0,0])`。这些 kp/kd 不会改变 Sharpa 的 POSITION 内环。

### 隐含行为与数据限制

1. C++ bridge 连接手时设置 `POSITION`、`speed_coeff=0.5`（可 CLI 改）、`current_coeff=0.6`（代码常量）、`ControlSource::SDK`，调用 `start()` 后发送全零姿态。默认关闭的是触觉 JPEG 传输，实际连接仍初始化触觉子系统。退出时再次发送全零，再 `stop()`。[连接/退出代码](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/scripts/sharpa_dds_bridge.cpp#L187-L237)
2. bridge 以标称 50 Hz 重发最后收到的目标，`set_joint_position(..., false)`；state 默认标称 50 Hz。循环 sleep 之外还叠加 SDK 调用/调度耗时，因此采集应测到达间隔，不能把标称频率当真实时间轴。C++ state 周期还向整数毫秒截断。[循环实现](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/scripts/sharpa_dds_bridge.cpp#L55-L128)
3. **没有命令超时/watchdog。** 工作站退出或网络断开后，bridge 仍重发旧目标。停止发布不等于机器人停止。仅在本地主机加 finally/timeout 不能补齐此限制。
4. C++ bridge 对少于 22 个目标的包将剩余关节补零；自身不校验 finite、关节限位或步长。不应用不完整包做“只动一个关节”。
5. `connect_hand_dds()` 和 `disconnect_hand_dds()` 均调用 `go_neutral()`；`SharpaHandDDSClient` 构造会建 publisher。只读采集直接订阅，避免使用这三个入口。[客户端](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/teleop/robot_control/robot_hand_sharpa_dds.py#L33-L45)
6. `SharpaWave_Controller` 把同一实际角度同时写入 `dual_hand_state_array` 和 `dual_hand_action_array`。后者**不是目标角**，不能用它辨识输入到输出动力学。应独立采集原始 command topic。[数组赋值](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/teleop/robot_control/robot_hand_sharpa.py#L123-L127)
7. HandState 没有硬件采样时间或 tick；client 的 sequence/age 是工作站自己收到包后生成的。只读日志能支持命令到反馈的整体闭环延迟/响应拟合，不能分离网络、bridge、SDK、传感器延迟。包抵达也不能证明手固件采样是新的。需要 bridge 扩展设备采样时间、SDK应用目标/应答及状态有效性后才能拆分。
8. Python bridge 的 `_make_hand_state_msg()` 使用无参数 `MotorState_()`、`HandState_()`；与 PR 所 pin 的 Python IDL 必需字段 dataclass 构造并不一致。部署说明使用 C++ bridge；此次没有运行或修复 Python bridge，不能假设两者都可直接启动。

SI 应在固定 POSITION、speed/current coefficient、SDK/firmware 和实际通讯路径下辨识。换设置后原拟合不自动有效。position-only 数据可识别有效闭环增益、滞后、时间常数、上升/下降非对称、速度/加速度饱和和跨关节耦合；**不能把这些响应参数冒充独立可辨识的真实惯量、粘滞摩擦、库仑摩擦或电机力矩常数**。

## H2：只读采集边界

`rt/lowstate` 类型是 `unitree_hg.msg.dds_.LowState_`，35 个 motor slots。`q`/`dq` 为关节角/角速度控制语义；按 rad/rad/s 保存，同时保留原始值和协议来源。`tau_est` 是估计力矩字段，不等同外置传感器力矩真值，必须核实固件实际填充、符号/标定。

| motor ID | 关节 |
| --- | --- |
| 0–5 / 6–11 | 左/右腿 hip pitch、roll、yaw，knee，ankle pitch、roll |
| 12、13、14 | waist yaw、roll、pitch |
| 15–21 | 左臂 shoulder pitch、roll、yaw，elbow，wrist roll、pitch、yaw |
| 22–28 | 右臂同序 |
| 29、30 | head pitch、yaw |
| 31–34 | unused slots；motion command 用 31 的 q 作使能权重 |

依据：[PR H2 index 与控制器](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/teleop/robot_control/robot_arm.py#L1150-L1250)。直接采集可保存 top-level `version[2]`、`mode_pr`、`mode_machine`、`tick`、`imu_state`、`crc`；每个 motor 的 `mode,q,dq,ddq,tau_est,temperature[2],vol,sensor[2],motorstate,reserve[4]`。字段存在不代表全部字段被本机固件提供。`tick` 的单位/回绕必须用观测与设备资料核实，先按整数原样保存。[LowState IDL](https://github.com/unitreerobotics/unitree_sdk2_python/blob/e4cd91f051aaa77a70600e3d2bf7f50889db1980/unitree_sdk2py/idl/unitree_hg/msg/dds_/_LowState_.py)、[MotorState IDL](https://github.com/unitreerobotics/unitree_sdk2_python/blob/e4cd91f051aaa77a70600e3d2bf7f50889db1980/unitree_sdk2py/idl/unitree_hg/msg/dds_/_MotorState_.py)。

**不要实例化 `H2_ArmController` 做只读工作。** 构造完成即启动发布线程，标称 250 Hz；默认 `motion_mode=False` 发布 `rt/lowcmd`，为 True 发布 `rt/arm_sdk`。它初始 `q_target=zeros(14)`，并主动向头部 home 目标移动。臂目标相对最新实测 q 以默认 20 rad/s 限幅；shoulder/elbow `kp=150,kd=10`，wrist `kp=50,kd=3`，head `kp=30,kd=1`；这些只是命令控制参数，不是 H2 的 SI 实测结论。state 过期逻辑仅告警，仍发布基于旧状态的目标。封装还只保留 q/dq，会丢失 `tau_est` 等原始信息。[构造/发布副作用](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/teleop/robot_control/robot_arm.py#L1220-L1435)

H2 motion-mode 的 waist API 明确只读，不能据此把仿真固定 pelvis/waist 等价成真机可控刚体。上层 SI 还需分开看：臂关节闭环、重力/负载变化、WBC站立造成的 pelvis/waist 移动、wrist-to-hand 实际安装变换；采集时记录机器人模式，不能混用不同 WBC/固定方式的数据。

## 进入真实激励前仍需的信息

- 实际部署 checkout SHA/未提交修改，运行的 bridge 二进制与构建源码对应关系、CLI speed/state_hz、固件/SDK版本、左右手序列号。
- 有效 DDS 网卡/路由/domain 与 command 唯一发布者；同一 topic 多发布者不能由本日志保证排他。
- 以双手和 H2 身体的实际反馈确定起点；用 USD 在移走桌子、双手空载、头腰腿固定的条件下规划中心、范围与进入路径。无需手填全关节角度表，不能将 CAD 极限直接当作动态激励范围。
- 在设备侧验证过的失联处理、超时处理和现场停止机制。此审计发现当前 bridge 没有 watchdog，因此只读采集和离线激励计划可以先做，不能据此批准无人值守自动扫全关节。
- 接触/剪刀铰链测量的真实外力、位移/角度与时间同步；仅现有 q topic 不足以完成这些 SI。

这些是源码显示的接口缺口，不是已测得的硬件故障。当前仓库提供被动采集、离线规划和分析，主动运动执行器及设备端断流处理尚未实现。

## Retarget 与动作标签

固定版本的公开 retarget DDS 脚本显示：默认 `hand-retarget` 使用 DDS backend；命令先经过本地平滑器（默认 250 Hz、w=100、z=0.2、内部 alpha=0.6）再发 `rt/sharpa/*/cmd`，然后 C++ bridge 50 Hz 取最新值。平滑器自己的 0.5 s 超时仅停止继续发布，不能解决 Thor 无 watchdog 的问题。若改用 `--sdk`，其直接 SDK backend 默认 `speed_coeff=0.3,current_coeff=0.6`，与 bridge 的 speed 默认不同。SI 要明确输入边界是平滑器前还是 DDS command 后，并沿最终部署链路验证。来源：[pin 的 retarget 脚本](https://github.com/isaac-for-healthcare/sharpa-teleop/blob/ce5a2635f7d3ad84041d279868f009e9ceeba6c3/workstation/sharpa-manus-sdk-1.1.0/retargeting_alg_release_V5.0/retargeting_manus_demo_dds.py)。

[teleop_hand_and_arm.py](https://github.com/binliunls/xr_teleoperate/blob/573175a2936253fe41aebb1d246bed47f3ced947/teleop/teleop_hand_and_arm.py) 对 Sharpa 使用“上一帧反馈作为 state、当前帧反馈作为 action”的时移标签。该 action 仍不是电机目标，不能用于输入输出辨识。

### Direct SDK 模式不能由当前 DDS 日志替代

固定版本的 `retargeting_manus_demo_dds.py` 中，`--dds`、`--sdk`、`--wave` 是互斥选项。`--dds` 才创建 `SharpaHandDDSClient`，发布 command topic 并订阅 state；`--sdk` 直接调用 SDK，**没有独立 DDS command 或 state publisher**。`read_state_degree()` 只用于本地状态打印，不生成 DDS 反馈。`robot_hand_sharpa.py` 开头声称旧 retarget 的 `-sdk -dds` 可产出 state，仅是旧注释，不能当当前 V5 pin 的功能证明。[互斥 CLI 与 backend 源码](https://github.com/isaac-for-healthcare/sharpa-teleop/blob/ce5a2635f7d3ad84041d279868f009e9ceeba6c3/workstation/sharpa-manus-sdk-1.1.0/retargeting_alg_release_V5.0/retargeting_manus_demo_dds.py#L1324-L1335)

`--sdk` 的 `send()` 调用 `set_joint_position(..., True)`，开启 SDK interpolation；C++ bridge 使用 False，除了默认 speed 不同，内插行为也不同。直接 SDK 模式的 SI 不能通过换成 bridge 测一次就声称已辨识同一链路。[SDK send 实现](https://github.com/isaac-for-healthcare/sharpa-teleop/blob/ce5a2635f7d3ad84041d279868f009e9ceeba6c3/workstation/sharpa-manus-sdk-1.1.0/retargeting_alg_release_V5.0/retargeting_manus_demo_dds.py#L475-L490)

可选 `--hand-action-address` 默认关闭，是 ZMQ 输出，发送 `result.filtered_angles`，位于平滑器之前；它不是实际 250 Hz backend send 的目标，也不是 SDK applied/ack。[ZMQ 输出位置](https://github.com/isaac-for-healthcare/sharpa-teleop/blob/ce5a2635f7d3ad84041d279868f009e9ceeba6c3/workstation/sharpa-manus-sdk-1.1.0/retargeting_alg_release_V5.0/retargeting_manus_demo_dds.py#L1092-L1134)

因此，现有 DDS 采集器适配 **bridge 的 command/state 路径**。若现场最终部署走 direct SDK，必须建立独立 logger adapter，在真实 backend send 点记录平滑后目标、调用时间、返回状态，并同步记录有效的 SDK feedback；不可把反馈复制为 command。未记录真实输入时，离线拟合应拒绝而不是猜测输入。
