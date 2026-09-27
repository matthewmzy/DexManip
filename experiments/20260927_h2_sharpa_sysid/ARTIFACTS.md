# 产物分级

- **A，不可丢失**：scripts/、bashes/、tests/、本目录文档；根迁移/诊断工具、vendor helper 及来源清单、先验常量、资产哈希清单。后续人工配置和真实 record 日志也属 A，虽 Git 忽略，仍必须单独备份。
- **B，保留私有副本**：原 USD、导出 model/geometry、PhysX cooked cache 及迁移包。当前分支只核验和读取既有导出，不包括完整 Isaac 导出流水线，不能仅凭本仓库重新生成缺失资产。
- **C，可重建**：`assets/demo_*` 合成数据、`assets/usd_plan_*` 未执行的离线轨迹与预览、Python 缓存。分别由 `bashes/run_demo.sh`、`bashes/run_usd_plan.sh` 重建；带实测工况或人工修改的文件不按 C 删除。

没有真实硬件实验结果随本分支发布。原工作区的历史快照与记录留在原工作区，未复制发布。
