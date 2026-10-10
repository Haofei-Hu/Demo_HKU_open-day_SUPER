# 配置归档

`observed_2026-10-08/mid360_real_ros2.yaml` 是用户实际提供的 SUPER 参数快照，用于追溯。它不是自动部署的首飞配置；包含 5 m/s 规划限速，不等于已设置正式演示的 2 m/s。

独立地面诊断副本由 `scripts/maintenance/create_super_ground_diagnostic.py` 生成。运行中的定位会话和实机参数应按 [完整工作流](../docs/FIRST_HOVER_2026-10-09.md) 回读，不能因存在归档文件就认定已经生效。
