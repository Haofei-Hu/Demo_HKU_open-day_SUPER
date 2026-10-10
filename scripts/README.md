# 代码分类与机载部署

| 目录 | 内容 | 当前用途 |
| --- | --- | --- |
| [localization/](localization/) | FAST_LIO 位姿转换、姿态配对、地面／连续视觉转发 | 终端 4、6、7，定位检查 |
| [flight/](flight/) | 单次悬停状态机、参考曲线、预览、参数辅助、地面 setpoint 检查 | 终端 11 的实现源文件 |
| [planning/](planning/) | SUPER 指令坐标预览和地面目标 | 终端 9／10 的调试辅助 |
| [analysis/](analysis/) | bag → XYZ、链路新鲜度、地面快照 | 录包与结果分析 |
| [maintenance/](maintenance/) | FAST_LIO 修复、历史安装／转换辅助、生成器、部署复制 | 工作区维护与发布 |

根目录保留 `first_hover_terminal.py`、`odom_age_check.py`、`update_preview_alignment.py` 三个已使用的下载入口。前者是生成的完整单文件，另两个是无仓库依赖的检查／会话更新脚本；不是缺少实现的链接文件。其余实现已搬入上表目录。[layout.json](layout.json) 是文件名到实现路径的机器可读索引。

## 安装到机载 Downloads

机载电脑的启动路径仍是 `~/Downloads/脚本名.py`。在已取得本仓库的机载 Ubuntu 上，飞机落地、未解锁、相关节点停止后，从仓库根目录执行：

```bash
python3 scripts/maintenance/install_scripts.py
```

默认只补齐缺失文件，保留已有安装和本次测量的对齐四元数。显式更新已有文件时：

```bash
python3 scripts/maintenance/install_scripts.py --overwrite
```

改动的旧文件先备份到 Downloads 下的 `super_scripts_backup_*`。它只复制文件，不启动任何 ROS 节点、不解锁、不修改 PX4 参数。替换会话对齐文件后，必须重新姿态配对；不要在飞行中更新。仅更新终端 11 时，用 [悬停工作流](../docs/FIRST_HOVER_2026-10-09.md) 的单文件下载，避免覆盖当前定位会话。

生成器需要仓库源文件，在仓库内执行，不部署到 Downloads：

```bash
python3 scripts/maintenance/build_first_hover_terminal.py --check
```

## 开发测试的导入路径

机载部署复制成同一目录，原有 Python 导入无需改变。开发时类别目录分开，在仓库根目录使用：

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD:$PWD/scripts/localization:$PWD/scripts/flight:$PWD/scripts/planning:$PWD/scripts/analysis:$PWD/scripts/maintenance${PYTHONPATH:+:$PYTHONPATH}"
python3 tests/test_repository_layout.py
python3 tests/test_hover_reference.py
python3 tests/test_bag_to_xyz.py
```

ROS 模拟话题测试只能在与飞机隔离的环境和 ROS domain 运行，见 [完整工作流](../docs/FIRST_HOVER_2026-10-09.md#开发验证)。离线 XYZ 导出不需要发布 ROS 话题。
