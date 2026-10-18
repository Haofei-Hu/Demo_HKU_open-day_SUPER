# 实验室完整表演试飞：1.8 m

流程：**升到飞控中心离地 1.8 m → 前进 4.5 m → 返回原悬停点，共 5 次往返 → 悬停 6 秒 → 降落并确认未解锁**。规划目标仍为 2 m/s、1 m/s²；接受上限为 4 m/s、2 m/s²。

`super_demo.py` 是实际飞机入口。先按[完整安装步骤](QUICK_START_ZH.md#第一次更新整套安装一次)安装配套版本 `c189b42ec6b81de11b13540036af2d3775d78128`；本次高度功能只更新 Python，不需额外编译已经装好的 SUPER 补丁。

## 1. 使用本次实测配置

任务文件为 `~/Downloads/open_day_mission_measured.json`。沿用已有测量时，核对它对应本次飞机和摆放位置。场地外边界为 5.8 × 2.8 × 2.8 m，机头沿 5.8 m 方向；文件中的后边界、左边界距离均从飞控中心测量。

先在终端检查文件和 1.8 m 高度下的边界，命令不启动 ROS 或飞行：

```bash
python3 - <<'PY'
from pathlib import Path
import json, sys
base=Path.home()/'Downloads'
sys.path.insert(0,str(base))
from demo_mission import DemoMission
p=base/'open_day_mission_measured.json'
if not p.is_file():
    raise SystemExit('缺少实测配置：'+str(p)+'；请按测量说明创建，不能用虚拟 fixture 替代')
c=DemoMission.physical_height(json.loads(p.read_text()),1.8)
m=DemoMission(c)
print('起飞位置：',c['takeoff'])
print('机体外形：',c['vehicle'])
print('目标：',m.waypoints())
print('往返次数：',m.round_trips)
if m.round_trips!=5:
    raise SystemExit('当前实测文件不是五次往返任务，请先核对任务配置')
print('CONFIG: PASS；这里只核对文件，不代表现场已具备起飞条件')
PY
```

若缺少文件，复制 `open_day_mission.json` 为上述独立文件，并填写[测量字段](FLIGHT_SITE_MEASUREMENTS.md)。尚未明确的左边界距离不能从“不会太偏”推算；模板中的 `null` 不会被自动替换。五寸桨与 205 mm 轴距的估算见测量说明，不能证明传感器、电池等也位于该外形内。

## 2. 开机后准备实飞

现场使用遥控器，确认控制器要求的 POSCTL/手动输入、有效定位和参数读回。程序会检查这些条件。实际 PX4 速度设置与规划器目标是两项配置；当前实飞入口要求 `MPC_XY_VEL_MAX=2.0`，较早记录为 0.5。如果读回不符，准备会停下并报告，不会自动改为 2.0。

终端 A：

```bash
python3 ~/Downloads/super_demo.py \
  --restart --fixed-lio-extrinsics \
  --mission-file "$HOME/Downloads/open_day_mission_measured.json" \
  --flight-height 1.8 \
  --motion-policy observe --execute --on-site-supervised
```

脚本自动打开或复用 QGC，启动本次定位、测量对齐四元数、准备视觉链路及 SUPER，捕获规划器与控制器共享的实测场地坐标，最后等待 `Phase: READY`。保持终端 A 运行。

`--flight-height 1.8` 将本次任务写到报告目录中的 `flight_mission.json`，不覆盖原实测文件。高度表示**飞控中心距起飞地面 1.8 m**，已有飞控离地 0.085 m 标定下对应上升约 1.715 m。地面演习仍固定为 0.5 m。

如只做准备检查，去掉 `--execute --on-site-supervised`；该模式不会执行飞行。

## 3. READY 后开始

终端 B：

```bash
source /opt/ros/humble/setup.zsh
ros2 service call /super_demo/start std_srvs/srv/Trigger '{}'
```

**这个调用开始实际飞行**，接受后将进入 OFFBOARD、解锁并执行任务。若返回拒绝，按返回原因处理；不要另发解锁命令绕过准备流程。

SUPER 可以改变路线及高度以避障。1.8 m 是起飞/悬停及任务目标高度，不是禁止升降的二维航线；返回目标为原悬停点，保持初始航向。场地和含桨外形边界始终参与检查。

## 4. 状态、提前降落与结果

终端 B 可查看任务状态：

```bash
timeout 10s ros2 topic echo \
  /super_debug/super_flight_status std_msgs/msg/String \
  --no-daemon --qos-reliability best_effort \
  --qos-durability volatile --full-length --field data
```

需要主动结束飞行时，在控制器仍正常运行的情况下请求降落：

```bash
ros2 service call /super_demo/land std_srvs/srv/Trigger '{}'
```

现场遥控器接管是独立通道。`Ctrl+C` 是终止进程，不等同于“悬停”或已确认降落；落地、未解锁后再停止后台。

报告保存在 `~/Downloads/super_demo_reports/<时间戳>/`：

- `flight_mission.json`：本次 1.8 m 配置副本。
- `preparation.json`：准备过程和各阶段日志路径。
- `report.json`：飞行阶段、五次往返完成数、最终结果。

本次发布已验证 1.8 m 配置和 ROS 控制器初始化/开始条件；**尚未有这架飞机的 1.8 m 完整表演实飞结果**。现场完成后保留整个报告目录。
