# 完整测试工作流：重启恢复与固定位置、离地 0.5／1 m 悬停

最新结果及下一步见 [10 月 10 日进度](PROGRESS_2026-10-10.md) 和 [后续待办](NEXT_STEPS.md)。用户反馈“成功了”，目前未明确是配置成功还是完整飞行成功，未收到本次实测日志。

本流程使用现有 10 Hz FAST_LIO／视觉定位，不修改雷达频率，不启动 SUPER 规划。10 月 9 日已看到 PX4 `vehicle_visual_odometry` 接收及水平／垂直位置、速度状态有效。该地面结果不等于已验证实际飞行。

当天用户回传的恢复证据：`/Odometry` 时间年龄 min／avg／max 为 `0.010/0.024/0.032 s`；原生视觉话题平均约 `10.006 Hz`、相邻消息 `0.088–0.116 s`；PX4 视觉消息 sample 比发布时间早 `31,619 µs`，水平相对位置和水平／垂直速度标志已为 true。这些是地面窗口数据，不是空中性能统计。

`first_hover_control.py` 默认只检查并输出 debug 目标。`--execute` 启用原生位置发送器，仍须单独调用 `/first_hover/start` 才开始一次自动飞行。控制程序尚需在机载电脑检查；隔离测试使用模拟遥测与命令服务，不能验证推力、实际定位精度、遥控失联或降落效果。

`first_hover_terminal.py` 是同一控制流程的自包含版本：普通启动只检查，`--execute` 后仍等待显式起飞命令。它不依赖同目录的其他 Python 脚本，适合终端 11 直接使用。之前聊天中的 `python3 - --execute <<'PY'` 只运行内存中的程序，不自动保存。更新命令明确写入 `~/Downloads/first_hover_terminal.py` 后才能重启复用；“更新完成”表示文件更新，不会改变旧的运行实例。1 m／6 秒需要新实例带相应 CLI 参数启动。

## 终端与数据流

全部命令运行在机载 Ubuntu 22.04／ROS2 Humble，使用 zsh。Mac 的 NoMachine 用来操作机载桌面，Mac 本地终端不运行 ROS 命令。

| 终端 | 进程／用途 | 本次悬停 |
| --- | --- | --- |
| 1 | MID360 驱动，约 10 Hz 雷达帧 | 保持运行 |
| 2 | FAST_LIO，`/Odometry` 与 `/cloud_registered` | 保持运行 |
| 3 | MAVROS，独占飞控 USB，转发 QGC UDP | 保持运行 |
| 4 | `fastlio_body_debug.py`，飞控中心姿态诊断 | 保持运行 |
| 5 | 参数回读、对齐检查、起飞／提前降落命令 | 操作窗口 |
| 6 | `aligned_pose_preview.py`，当前会话对齐位姿 | 保持运行 |
| 7 | `flight_pose_cov_transport.py`，持续视觉定位转发 | 保持运行，包含解锁后 |
| 8 | QGroundControl，MAVLink Console | 保持运行 |
| 9 | SUPER 地面规划诊断 | 悬停时停止 |
| 10 | `control_bridge_preview.py`，规划指令坐标预览 | 悬停时停止 |
| 11 | `first_hover_terminal.py`，本次固定位置悬停控制 | 检查，之后显式启动 |

```mermaid
flowchart LR
    L[终端 1 MID360] --> O[终端 2 FAST_LIO]
    O --> A[终端 6 会话对齐]
    O --> D[终端 4 姿态诊断]
    O --> V[终端 7 视觉转发]
    A --> V
    V --> M[终端 3 MAVROS]
    M <--> F[PX4 估计与位置控制]
    M <--> Q[终端 8 QGC]
    M --> H[终端 11 固定位置悬停]
    H -->|20 Hz ENU 位置与航向| M
    O -. 地面规划调试 .-> S[终端 9 SUPER]
    S -. planning/pos_cmd .-> P[终端 10 坐标预览]
    P -.-> B[super_debug/control_target]
```

实际悬停的控制来源是终端 11。终端 9／10 的输出用于规划调试，不参与这次固定点悬停。

## 重启后的准备

1. 先确认飞机在实际起飞地面、未解锁，解锁开关关闭。保持静止，完成初始化和对齐。
2. 恢复 Wi-Fi／NoMachine；完整驱动与无显示器恢复记录见 [10 月 9 日远程操作记录](REMOTE_RECOVERY_2026-10-09.md)。当前记录的无线 IP 为 `192.168.6.146`，重启后需以实际地址为准。
3. 先启动终端 1–4，再在终端 5 重新配对；随后启动 6／7、检查 8，最后进入 11。已编译的 ROS 工作区不需要每次重新编译。

在机载电脑确认当前无线地址与远程桌面服务：

```bash
nmcli device status
ip -4 -br addr show dev wlx6c1ff7880bdf
sudo /usr/NX/bin/nxserver --status
```

Mac 的 NoMachine 连接当前 IP、NX 协议、端口 4000，使用机载 Ubuntu 用户 `mars` 的本机账号。飞控的 USB 线留在飞机上；没有接到外部电脑的飞行拖线。

### 获取脚本，不使用安装包

已有工作区和终端 1–7 脚本时，只需获取终端 11 的文件。先停止旧的终端 11 程序，飞机保持未解锁：

```bash
hover_download_dir=$(mktemp -d "$HOME/Downloads/hover_download_XXXXXX")
if curl -q --noproxy '*' --connect-timeout 10 --max-time 30 -fL \
  'https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/main/first_hover_terminal.py' \
  -o "$hover_download_dir/first_hover_terminal.py" \
  && python3 -m py_compile "$hover_download_dir/first_hover_terminal.py"
then
  if [ -f "$HOME/Downloads/first_hover_terminal.py" ]; then
    cp "$HOME/Downloads/first_hover_terminal.py" "$hover_download_dir/previous.py"
  fi
  cp "$hover_download_dir/first_hover_terminal.py" "$HOME/Downloads/first_hover_terminal.py"
fi
```

`--noproxy '*'` 避开本机未启动的 `127.0.0.1:7897` 代理；TLS 和 HTTP 错误检查保留。下载失败时不覆盖已有文件。

如果终端 5 的辅助检查脚本缺失，可先获取缺失文件：

```bash
check_download_dir=$(mktemp -d "$HOME/Downloads/check_download_XXXXXX")
for script_name in odom_age_check.py update_preview_alignment.py; do
  if [ ! -f "$HOME/Downloads/$script_name" ]; then
    if curl -q --noproxy '*' --connect-timeout 10 --max-time 30 -fL \
      "https://raw.githubusercontent.com/Haofei-Hu/Demo_HKU_open-day_SUPER/main/$script_name" \
      -o "$check_download_dir/$script_name" \
      && python3 -m py_compile "$check_download_dir/$script_name"
    then
      cp "$check_download_dir/$script_name" "$HOME/Downloads/$script_name"
    else
      break
    fi
  fi
done
```

完整首次部署所需的现有定位脚本，见 [README 的安装步骤](../README.md#首次获取这次更新安装脚本)。不要在锁定会话后覆盖正在使用的对齐文件。

## 终端 1–8：恢复定位

### 终端 1：MID360

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

本流程保留 `publish_freq=10.0`。50 Hz 调整按用户要求暂缓，不通过改时间戳或重复发布旧测量伪造高频定位。

### 终端 2：FAST_LIO

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file "$(ros2 pkg prefix fast_lio)/share/fast_lio/config/mid360.yaml" \
  -p use_sim_time:=false \
  -p publish.map_en:=false \
  -p publish.path_en:=false \
  -p publish.scan_publish_en:=true \
  -p publish.scan_bodyframe_pub_en:=false
```

正常窗口已记录 `pub_age` 约 0.02–0.03 秒、`irx≈0`、`lidar_q=0`。如果重新出现 1 秒以上积压，先排查数据流，不放宽新鲜度门限来起飞。

### 终端 3：MAVROS

```bash
source /opt/ros/humble/setup.zsh
ros2 launch mavros px4.launch \
  fcu_url:="serial:///dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00:2000000" \
  gcs_url:="udp://127.0.0.1:14555@127.0.0.1:14550"
```

串口路径按本机 `/dev/serial/by-id/` 中的实际设备核对。MAVROS 使用 USB，QGC 使用本机 UDP 转发。

### 终端 4：飞控中心姿态诊断

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/fastlio_body_debug.py
```

### 终端 5：参数缓存、时间检查和新会话对齐

确认终端 6／7 尚未运行，飞机未解锁：

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
ros2 service call /mavros/param/pull \
  mavros_msgs/srv/ParamPull '{force_pull: true}'
timeout 10s ros2 topic echo /mavros/state --once
```

确认 `connected: true`、`armed: false` 后：

```bash
ros2 param set /mavros/param EKF2_EV_CTRL 0
ros2 param get /mavros/param EKF2_EV_CTRL
ros2 param get /mavros/param EKF2_HGT_REF
python3 ~/Downloads/odom_age_check.py
python3 ~/Downloads/frame_alignment_check.py
```

配对运行 15 秒；保持飞机静止。使用本次输出的四元数，下面是占位格式，不能原样粘贴：

```text
python3 ~/Downloads/update_preview_alignment.py QX QY QZ QW
```

每次 FAST_LIO 重新初始化后重新配对，不复用历史四元数。10 月 9 日记录曾测到 `[0.001259, -0.007881, 0.964143, 0.265265]`，仅作为那一次会话证据。

### 终端 6：对齐位姿

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/aligned_pose_preview.py
```

终端 5 检查原始时间戳和位姿：

```bash
timeout 10s ros2 topic echo /super_debug/aligned_pose_cov \
  geometry_msgs/msg/PoseWithCovarianceStamped --once
```

预览零点是该节点首个接受的 FC 中心位置，不能单独当作实际地面高度。终端 11 会另行捕获起飞地面位姿。

### 终端 7：持续视觉转发

确认停止旧 raw／aligned／ground 视觉转发进程。在终端 5 恢复本机已经使用过的定位配置：

```bash
ros2 param set /mavros/param EKF2_HGT_REF 3
ros2 param set /mavros/param EKF2_EV_POS_X 0.0
ros2 param set /mavros/param EKF2_EV_POS_Y 0.0
ros2 param set /mavros/param EKF2_EV_POS_Z 0.0
ros2 param set /mavros/param EKF2_EV_QMIN 0
ros2 param set /mavros/param EKF2_EV_CTRL 11
```

这些参数修改只在地面、未解锁状态进行。`11` 对应外部水平位置、垂直位置和 yaw，输入没有外部速度。

随后终端 7：

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 ~/Downloads/flight_pose_cov_transport.py \
  --ros-args -p publish_to_mavros:=true
```

等待 `Session locked; first pose forwarded with original stamp/covariance`。在终端 5 确认：

```bash
ros2 topic info /mavros/vision_pose/pose_cov
timeout 10s ros2 topic hz /mavros/vision_pose/pose_cov
timeout 10s ros2 topic echo /mavros/estimator_status --once
timeout 10s ros2 topic echo /mavros/local_position/pose --once
```

原生视觉话题应只有一个发布者、有 MAVROS 订阅者，约 10 Hz；水平相对位置和水平／垂直速度状态应有效。没有 GPS 时，绝对水平位置标志为 false 不能单独说明外部相对定位失败。

如果终端 7 报 `Active alignment file changed`，应在未解锁地面状态停止 6／7，完成对齐更新后同时恢复两者。如果报参数等待，先 force pull 和逐项回读，不反复只启动进程。

查看具体拦截原因：

```bash
timeout 12s ros2 topic echo /rosout rcl_interfaces/msg/Log \
  --qos-durability volatile --qos-reliability best_effort \
  --filter "m.name == 'flight_pose_cov_transport'"
```

Humble 的此命令不与 `--field msg` 同时使用，否则过滤器中的 `m` 会变成字符串，出现 `AttributeError`。

### 终端 8：QGroundControl

```bash
~/Downloads/QGroundControl-x86_64.AppImage
```

Application Settings → Comm Links：关闭 Pixhawk 自动连接，开启 UDP。USB 线仍插着，由 MAVROS 使用；QGC 从本机 UDP 14550 接收转发。

MAVLink Console：

```text
listener vehicle_visual_odometry 1
listener vehicle_local_position 1
listener actuator_armed 1
commander check
```

检查是当前会话的结果。不要用之前的 OK 或地面融合输出替代此次起飞前检查。

## 终端 9／10：可选规划地面测试

定点悬停前停止这两个进程。需要单独检查 SUPER 时才运行以下步骤，飞机保持未解锁。

终端 5 准备已有源配置的诊断副本：

```bash
if [ ! -f "$HOME/super_ws/src/SUPER/super_planner/config/mid360_ground_diagnostic_ros2.yaml" ]; then
  python3 ~/Downloads/create_super_ground_diagnostic.py
fi
mkdir -p ~/super_ws/src/SUPER/super_planner/log
```

终端 9：

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
ros2 run super_planner fsm_node --ros-args \
  -p use_sim_time:=false \
  -p config_name:=mid360_ground_diagnostic_ros2.yaml
```

终端 10：

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 ~/Downloads/control_bridge_preview.py
```

未给目标时等待 `/planning/pos_cmd` 是正常现象。需要小目标验证时，在终端 5 运行：

```bash
python3 ~/Downloads/send_control_preview_goal.py
```

这是地面规划目标，不是实际起飞指令。检查地图和规划输出：

```bash
timeout 10s ros2 topic echo /rog_map/occ \
  sensor_msgs/msg/PointCloud2 --once --field width
timeout 10s ros2 topic hz /planning/pos_cmd
```

即使规划显示 FOLLOW_TRAJ 或 Traj finish，地面未移动飞机也会重新规划，不能当作飞机已完成轨迹。原来的 `ground_setpoint_transport.py` 会在解锁／OFFBOARD 时锁止，不能用于此次自动悬停。

## 保留的定位会话

保持终端 1–4、6–8 运行，尤其终端 7 的 `flight_pose_cov_transport.py --ros-args -p publish_to_mavros:=true`。它会继续转发解锁后的视觉定位。停止终端 9 的 SUPER，以及以前终端 11 的 `ground_setpoint_transport.py` 或其他 MAVROS 控制发送器；首悬停不用规划目标。已锁定会话后不改对齐文件、不重启 FAST_LIO／MAVROS／位姿节点。

飞机放在真正起飞地面、水平静止。无外接供电／显示器线；机载电脑和飞控随飞机一起飞。遥控器保持连接、解锁开关关闭，模式为 POSCTL。地面飞控中心离地 8.5 cm 是本次使用的物理尺寸。

## 控制行为

1. 验证新鲜 FC 状态、RC、估计器、落地状态、视觉与本地位姿、参数及唯一控制来源。要求未解锁 POSCTL，地面稳定 3 秒后固定本地 XYZ、yaw 和定位发布者身份。
2. 显式启动后，在原地面位置先发送 2 秒、20 Hz 的 ENU 位置／航向指令，再请求 OFFBOARD；确认模式后请求解锁。两次请求都不强制、不重试，拒绝或超时即停止。
3. 确认解锁后约 3.89 秒平滑上升，目标垂直峰值速度 0.2 m/s；固定起飞 XY 和 yaw。目标本地 Z 为地面 Z 加 **0.415 m**，对应飞控中心离地 **0.5 m**。
4. 高度进入目标 ±6 cm 后保持 2 秒，再请求 AUTO.LAND。若解锁后 8 秒还未完成保持，也请求降落。实际总飞行时间取决于飞控起飞、落地检测和降落过程，不能保证恰好 10 秒。
5. PX4 执行降落；只有收到落地且未解锁的状态才报告 DONE。程序不发送强制解锁、空中停机或定时解锁解除请求，不会再次起飞。

以上是默认 0.5 m／2 秒配置。`--hover-height 1.0 --hold-seconds 6.0` 使用用户要求的 1 m／6 秒配置：飞控中心地面高度仍为 8.5 cm，目标本地 Z 为 `z_ground + 0.915 m`，上升参考约 8.578 秒，垂直峰值速度仍为 0.2 m/s。达到目标 ±6 cm 后开始计时保持 6 秒，再请求 AUTO.LAND。解锁到请求降落的备用总时限随参考时长计算，当前为 17 秒；正常上升加保持约 14.58 秒，完整飞行还包括降落。高度上界随目标调整为目标加 0.235 m，其余定位、遥控、接管和异常检查保留。控制程序支持离地高度大于 0.085 m、至多 1 m，保持时间 0–6 秒。高度与保持参数在启动时固定，空中不修改。

接管优先：遥控器切到 POSCTL 等其他模式，程序立即停止位置发送，不再改模式、不请求降落覆盖接管。输入过期、估计无效、来源变化、定时器中断、水平偏离 0.3 m 或航向偏离 15°，会停止发送；若仍由本程序控制且处于已解锁 OFFBOARD，则请求 AUTO.LAND。命令服务失联或进程退出时依赖 PX4 已回读的 OFFBOARD 丢失动作：`COM_OF_LOSS_T=0.5`、`COM_OBL_RC_ACT=4`。这不是已验证的物理降落保证，也不形成正式场地围栏。

## 终端 11：只读检查与实际启动

在终端 5 回读限速与 OFFBOARD 丢失动作。下面数值已在 10 月 8 日机载设置并回读，重启后仍需检查实际值：

```bash
for ros_param_name in MPC_XY_VEL_ALL MPC_Z_VEL_ALL MPC_XY_VEL_MAX MPC_Z_VEL_MAX_UP MPC_Z_VEL_MAX_DN COM_OF_LOSS_T COM_OBL_RC_ACT; do
  printf '%s: ' "$ros_param_name"
  ros2 param get /mavros/param "$ros_param_name"
done
```

预期为 `-1.0、-1.0、0.5、0.5、0.5、0.5、4`。`configure_first_hover.py` 默认只备份／读取，显式 `--apply` 才修改限速和失联参数。控制脚本本身不修改参数。

终端 11：

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/first_hover_terminal.py
```

等 `Phase: READY` 和 `GROUND LOCK`。此方式不会向 MAVROS 发布 setpoint，不会切模式、不会解锁。

QGC 的 MAVLink Console，解锁开关保持关闭，释放 Kill 后检查：

```text
listener actuator_armed 1
commander check
```

应为 `armed: False`、`kill: False`、`termination: False`，且在 POSCTL 下 `Preflight check: OK`。检查未通过时不以自稳／角速度模式解锁绕过检查。

终端 5 可补充当前状态：

```bash
timeout 10s ros2 topic echo /mavros/state --once
timeout 10s ros2 topic echo /mavros/extended_state --once
```

## 实际启动（完成上述机载检查之后）

终端 11 Ctrl+C 停止只读实例，然后：

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/first_hover_terminal.py --execute
```

用户要求的 **飞控中心离地 1 m、保持 6 秒** 则使用以下命令，代替上面的默认命令。先在落地、未解锁状态停止旧终端 11 实例，下载最新版脚本，再启动一个实例：

```bash
python3 ~/Downloads/first_hover_terminal.py --execute --hover-height 1.0 --hold-seconds 6.0
```

只读检查同一配置：

```bash
python3 ~/Downloads/first_hover_terminal.py --hover-height 1.0 --hold-seconds 6.0
```

启动日志应显示 `hover FC=1 m, hold=6 s` 和 `OFFBOARD budget=17 s`，GROUND LOCK 应显示 `FC height above floor=1.000 m`。日志不同或提示参数不认识，说明正在使用旧文件／旧实例。

程序再次稳定检查和捕获实际地面位置。只有见到 READY 且飞行空间清空、遥控接管人员就位，才在终端 5 发起以下命令。**它会自动切 OFFBOARD、解锁并起飞。**

```bash
ros2 service call /first_hover/start std_srvs/srv/Trigger '{}'
```

正常日志为 PRESTREAM → MODE_REQUEST → ARM_REQUEST → RISE → HOLD → LAND_REQUEST → LANDING → DONE。

主动提前降落：

```bash
ros2 service call /first_hover/land std_srvs/srv/Trigger '{}'
```

遥控器切 POSCTL 是人工接管。Kill 会直接停止电机，不是平稳降落指令。重新开始必须先确认飞机落地、未解锁，再新启动程序；不能在空中重启任何初始化节点。

若只看到 `waiting for service to become available...`，先检查终端 11 是否还在运行：

```bash
ros2 node info /first_hover
```

启动请求返回 `success: true` 只说明本次请求被程序接受，随后仍需看到 OFFBOARD、解锁确认、RISE 和 HOLD。服务返回不等于飞机已起飞成功。

## 开发验证

测试入口 `tests/test_first_hover_control.py` 使用 ROS2 Humble DDS 和 MAVROS 2.14.0 的官方消息／服务定义，不连接飞机。测试遥测频率 10 Hz，控制目标约 20 Hz；验证默认 0.5 m／2 秒与新增 1 m／6 秒配置、随配置调整的高度上界和总时限、一键启动、固定 XYZ／yaw、高度偏移、模式／解锁拒绝、输入失效、重复定位来源、偏离、定时器中断、接管和落地／未解锁双重确认。参考曲线另有有限差分检查，包含 1 m 配置的速度上限与保持端点。软件检查不能验证真实悬停与降落。

`first_hover_terminal.py` 由 `build_first_hover_terminal.py` 生成，来源是 `first_hover_control.py`、`ground_setpoint_transport.py` 中的位姿校验、`hover_reference.py` 和 `first_hover_preview.py` 中的参数表。发布前对两个版本运行同一套隔离测试，避免单文件版本与主实现分叉。

发布复核时，并行运行两套测试曾触发一次超过 0.2 秒的定时器中断保护，程序正确锁止；随后主版本单独重跑通过，门限没有放宽。机载实际负载下的时序稳定性仍需地面与飞行验证，特别是不要在首悬停时同时运行可选的规划任务。

1 m／6 秒更新验证：主版本与自包含版本分别顺序运行完整 DDS／服务检查通过，参考曲线四项单元测试通过，两个入口均在 ROS 初始化之前拒绝 NaN、超过 1 m 的高度和超过 6 秒的保持时间。自包含版本与 CLI 边界检查并行时也曾触发 0.2 秒定时器中断保护；改为顺序运行后完整检查通过，没有修改保护门限。模拟遥测的任务时钟在测试中加速，不能把测试日志中的墙钟时长当作真实上升或保持时间。

仅在与飞机隔离的开发环境运行：

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD:$PWD/scripts/localization:$PWD/scripts/flight:$PWD/scripts/planning:$PWD/scripts/analysis:$PWD/scripts/maintenance${PYTHONPATH:+:$PYTHONPATH}"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=194
python3 scripts/maintenance/build_first_hover_terminal.py --check
python3 tests/test_hover_reference.py
python3 tests/test_first_hover_control.py
FIRST_HOVER_TEST_MODULE=first_hover_terminal python3 tests/test_first_hover_control.py
```

用户随后反馈“成功了”，具体完成范围尚待明确；尚未收到本次 READY、RISE／HOLD／DONE、高度或落地日志，不能补写实测结论。正式 3×6 m 场地围栏、2 m/s 轨迹飞行和实际定位／遥控失联降落验证仍未完成。

发布检查还验证了自包含文件与生成器一致、三份主要文档中 67 个 Bash 命令块的语法和本地链接、AX900 脚本语法，以及把归档 helper 安放在临时根目录后的 systemd unit 解析。系统配置检查没有启动驱动／网络服务，不能替代机载冷启动验证。
