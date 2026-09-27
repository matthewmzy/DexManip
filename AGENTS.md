# SI 开发约定

- 当前分支用于 H2 + 双 Sharpa SI，入口和阶段见 README.md。先读 SI/control_audit.md、usage_notes.md、log.md。
- 只在本仓库工作；不修改同事既有机器人工作区。不在公开仓库记录机器登录凭据、私有工作区快照或真实采集数据。
- 头、腰、腿保持不动，只辨识双手及手腕/手臂影响；使用当前固定 SHA 的原 USD，不换 URDF、不简化网格或放宽碰撞条件来取得通过。
- 私有 USD 和派生资产只通过本地迁移包导入 assets/h2_sharpa/，不提交 Git。缺失时明确报错；不下载替代模型。
- 当前没有主动运动执行器。默认工具只离线计算或被动 DDS 订阅；不导入会在构造时发布命令的机器人控制器。真实运动需结合现场就绪情况与明确的实验指令，不能把运行测试等同于授权启动机器人。
- 采用已有 DDS bridge 部署路径。Sharpa command.q 为 rad，state.q 为 degree；bridge 仅提供有效 q，没有真实力矩/触觉/时间戳。保留这些限制。
- 原始数据不覆盖；每次实验在独立目录输出。脚本/人工参数/测量记录为 A 级；新结果记录到 log.md 和 ARTIFACTS.md。不能把 synthetic 或离线通过写成真机完成。
- 依赖安装进本仓库 .venv；模型缺失或环境失败时不静默降级。几何过程保持 OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1。
- 验证应覆盖单位、通信失败、拟合拒绝条件和实际 USD。发布前检查 git diff 和暂存文件；保持模型、日志、密钥在 Git 外。
