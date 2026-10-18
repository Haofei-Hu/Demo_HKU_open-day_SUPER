# 开机、重启后的快速启动

当前配套发布版本：`4b025e9f6819c9f82dea11f59e434983828b2f36`。
本页命令用于**真实雷达地图上的地面虚拟演习**。真实飞机保持未解锁、静止；飞行动作由虚拟飞机执行。

## 第一次更新：整套安装一次

先结束正在运行的自动测试。更新时使用完整安装器，避免新旧脚本混用。
安装器会先下载并校验全部 28 个文件，再备份和替换需要更新的文件；已有对齐文件和任务测量配置按安装器规则保留。

```bash
(
set -e
demo_revision='4b025e9f6819c9f82dea11f59e434983828b2f36'
demo_update_dir=$(mktemp -d "$HOME/Downloads/demo_full_update_XXXXXX")

curl -q --noproxy '*' -fL \
  --connect-timeout 10 --max-time 30 \
  "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/$demo_revision/scripts/maintenance/install_demo_test.py" \
  -o "$demo_update_dir/install_demo_test.py"

python3 -m py_compile "$demo_update_dir/install_demo_test.py"
python3 "$demo_update_dir/install_demo_test.py" --revision "$demo_revision"
)
```

这次参数回复、轨迹接续和 RViz 显示的更新只涉及 Python/配置，已装好当前 SUPER 补丁的电脑不用重新编译。
若从较早版本部署、尚未编译运动接受上限和场地边界补丁，先完成[一次性补丁与编译](REBOOT_QUICK_START.md#install-or-update-the-matching-scripts)。

## 以后每次开机：运行这一条

确认飞控、雷达已供电，飞机静止放在起飞位置。通过 NoMachine 连接机载 Ubuntu，打开一个终端：

```bash
python3 ~/Downloads/super_demo_test.py \
  --restart --fixed-lio-extrinsics \
  --fixture-geometry --motion-policy observe --fast-start
```

普通重启后无需重新下载、编译、手动 `source` 或复制四元数。
程序自动完成：

1. 打开或复用 QGC，启动雷达、FAST_LIO、MAVROS 和机体坐标转换。
2. 并行进行本次里程计观察和四元数测量，应用新对齐并启动外部视觉。
3. 并行准备地图/规划器和连续 30 秒的新鲜定位观察，通过后捕获本次地图。
4. 自动打开 RViz，使用真实 SUPER 和虚拟飞机完成完整演习。
5. 输出 `RESULT: SUCCESS` 或 `RESULT: FAIL`，保存报告和各阶段日志。

`--fast-start` 加速的是独立准备工作的并行执行，每次仍重新测量、重新建图。
30 秒用于准备阶段的数据观察，每一段航线没有额外的 30 秒等待。
关闭终端不会自动关闭后台 tmux 节点；再次运行上述命令会先检查地面状态，再清理旧会话。

## 当前演习设置

| 项目 | 设置 |
|---|---|
| 地面虚拟起飞高度 | 飞控中心距地面 0.5 m |
| 实飞任务高度 | 飞控中心距地面 1.5 m，使用单独的实飞入口 |
| 路线 | 前进 4.5 m → 返回原悬停点，共 5 次往返 |
| 返回方式 | SUPER 重新规划返回路径，保持初始机头方向 |
| 最后阶段 | 悬停 6 秒 → 降落 → 确认落地和未解锁 |
| 规划目标上限 | 2 m/s、1 m/s² |
| 规划结果接受上限 | 4 m/s、2 m/s² |
| 控制端运动观察 | `observe` 记录速度/加速度超限；仅该项超限不终止任务 |
| RViz 点云 | `Size (Pixels): 5` |
| 规划及整段执行截止时间 | 无；由到达、悬停和落地反馈推进 |

SUPER 可以沿曲线路径避障。起飞后使用含飞机外形的场地边界检查，不要求贴着起飞轴线飞。
`--fixture-geometry` 是明确标记的虚拟尺寸与摆放位置，不能当作实测场地数据用于实飞。
RViz 在演习结束或失败后自动关闭，QGC 保留。

## 看结果和保存报告

每次运行都会打印自己的目录：

```text
~/Downloads/super_demo_tests/<时间戳>/
  report.json             整体流程结果
  mission_report.json     五次往返、悬停、降落的结果
  alignment.json          本次四元数测量
  scene.json              本次捕获的地图和坐标信息
  planner_rehearsal.log   虚拟演习中的 SUPER 日志
  venue_reference.json    本次场地坐标和边界
```

早期准备阶段失败时，后面的文件可能还没有生成。读取最新一次结果：

```bash
python3 - <<'PY'
from pathlib import Path
import json
paths = sorted((Path.home() / 'Downloads/super_demo_tests').glob('*/report.json'))
if not paths:
    raise SystemExit('没有找到演习报告')
p = paths[-1]
for name in ('report.json', 'mission_report.json'):
    f = p.parent / name
    if not f.exists():
        continue
    r = json.loads(f.read_text())
    print('\nREPORT:', f)
    for key in ('result', 'stage', 'error', 'round_trips_completed'):
        print(key + ':', r.get(key))
PY
```

`RUNNING` 表示尚未完成；中断留下的 `RUNNING` 也不算成功。
保留整个目录，不能仅凭 RViz 中走完一段就判定五次演习成功。

## 结束后查看或关闭后台

```bash
python3 ~/Downloads/super_session.py status
```

确认实际飞机已落地、未解锁后：

```bash
python3 ~/Downloads/super_session.py stop
```

## 已知问题与恢复

| 现象 | 含义与处理 |
|---|---|
| `inverse_goal() takes 3 positional arguments but 4 were given` | 脚本版本混用；执行本页完整安装，再重跑 |
| `set_parameters response` 超时 | 当前版会只读回查 `EKF2_EV_CTRL`；确认写入成功才继续，写请求不会重发 |
| `mavros: EXITED`，停止时报地面状态超时 | 先看 MAVROS 日志；已退出的桥接进程无法提供地面状态，按[限定的核心会话恢复步骤](REBOOT_QUICK_START.md#recover-an-exited-mavros-in-a-core-only-session)处理 |
| `std::future_error: Promise already satisfied` | 已观察到的 MAVROS 内部崩溃，根因尚未根治；恢复进程与永久修复是两件事 |
| 小幅停车后接续触发 `position step` | 当前版核对两条已提交轨迹的停车接续；记录在 `stationary_replan_handoffs_by_leg`，较大跳变和越界仍需调查 |
| `WAIT_PLAN` 持续不动 | 正在等可执行轨迹；查看本次 `planner_rehearsal.log` 区分无路、走廊失败或优化失败 |

当前发布已完成 57 项参考/控制回归、5 项安装器检查，以及已有实际点云上的一次完整五次虚拟往返，结果为 `SUCCESS`。
停车接续修复另有针对性复现测试；完整回归并未触发该小幅接续分支，二者证据分别记录。
最新机载整套更新后的完成结果仍待回传。详见[验证记录](../data/software_checks_2026-10-18/stationary_handoff_release.json)。

实际飞机的五次完整任务尚未验证。实飞使用[独立准备流程](REBOOT_QUICK_START.md#prepare-physical-execution-separately)，需要填写[真实机体尺寸与场地位置](FLIGHT_SITE_MEASUREMENTS.md)。
