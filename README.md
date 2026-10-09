# Demo_HKU_open-day_SUPER

HKU Open Day 无人机演示部署记录：Livox MID360 → FAST_LIO → SUPER → MAVROS → PX4。

**更新日期：2026-10-09（香港时间）**。已恢复 10 Hz 外部视觉 XYZ／yaw 地面融合，Mac 的 NoMachine 桌面已可操作。终端 11 的固定位置、飞控中心离地 0.5 m 悬停程序已实现并通过隔离测试；尚未收到这段程序在真实飞机上的 READY、当前预检 OK 或实际起飞结果。

直接按 [完整测试工作流：终端 1–11、重启恢复、起飞、降落和接管](docs/FIRST_HOVER_2026-10-09.md) 操作。终端 9／10 用于规划地面调试，定点悬停时停止；终端 11 沿用现有 10 Hz 定位，发送 20 Hz 位置／航向目标，等待显式起飞命令。

## 快速开始：电脑重启后恢复

以下命令运行在**机载 Ubuntu 22.04 电脑**，使用 zsh。重启只需恢复进程；已编译的 FAST_LIO 修复保留，**不用每次重新编译**。飞机保持未解锁，解锁开关关闭。

### 先恢复远程桌面

机载电脑上的 USB 线连接飞控，飞行时这条线仍留在飞机上。Mac／Windows 通过 Wi-Fi 操作机载桌面，ROS、QGC 和 RViz 都在机载电脑运行。

10 月 9 日最后确认的网络记录：

| 用途 | 网卡／网络 | 地址 |
| --- | --- | --- |
| MID360 专用以太网 | `enp4s0`，`livox-mid360` | 电脑 `192.168.50.125/24`，雷达 `192.168.50.163` |
| 远程操作无线网 | AX900，`wlx6c1ff7880bdf`，`Mars_Office 1` | 电脑 `192.168.6.146/24` |
| 临时外部有线网 | `enp1s0` | 排查期间曾为 `192.168.1.101/24`，最后记录 unavailable |

无线 IP 可能在重启后变化，先在 Ubuntu 查看：

```bash
nmcli device status
ip -br addr
```

两台电脑都安装 [NoMachine](https://www.nomachine.com/download)，连接同一个 Wi-Fi。在 Mac／Windows 创建**手动连接**，协议 `NX`、Host 填当前 Ubuntu 无线 IP、端口 `4000`，使用 Ubuntu 用户 `mars` 和其本机登录密码。

**NoMachine 已恢复可操作。** 原安装版本为 Personal Edition 10.2.3，报订阅缺失；改用 NoMachine 9.8.2 后服务已启用。无显示器画面问题已通过本机 DP-1／EDID 方案恢复；开机初始化与冷启动待验证部分见 [远程桌面和 AX900 恢复记录](docs/REMOTE_RECOVERY_2026-10-09.md)。重启后先检查无线地址和服务：

```bash
# Ubuntu
sudo /usr/NX/bin/nxserver --status
sudo /usr/NX/bin/nxserver --version
sudo ss -ltnp 'sport = :4000'
systemctl is-enabled ugreen-ax900.service
ip -4 -br addr show dev wlx6c1ff7880bdf
```

```bash
# Mac：将地址换成当前 Ubuntu 无线 IP
ping -c 3 192.168.6.146
nc -vz -G 3 192.168.6.146 4000
```

已保存无线自动连接设置和 AX900 systemd 初始化服务；最终延迟切换方案尚未收到完整冷启动成功回传。SSH／tmux 尚未配置。飞机已移到地面，用户确认无外接线；实际起飞前仍需当前会话的定位／预检结果。

### 首次获取这次更新：安装脚本

已有完整 ROS 工作区时，在本仓库根目录执行下面的复制步骤。**先停止终端 6／7 和 9–11 中的预览、转发进程**，再更新文件；复制会覆盖历史会话四元数，因此随后必须重新做姿态配对。

```bash
mkdir -p ~/Downloads
scripts_backup_dir=$(mktemp -d ~/Downloads/super_scripts_backup_XXXXXX)
for script in *.py; do
  if [ -f "$HOME/Downloads/$script" ]; then
    cp "$HOME/Downloads/$script" "$scripts_backup_dir/"
  fi
  cp "$script" "$HOME/Downloads/"
done
```

这一步只复制文件。完整的 Livox／FAST_LIO／SUPER 工作区及依赖仍使用机载电脑现有安装，未随本仓库打包。下面的 1–8 恢复地面定位，9–10 是另行开启的规划调试。

| 终端 | 当前用途 |
| --- | --- |
| 1 | MID360 驱动 |
| 2 | FAST_LIO，发布世界点云 |
| 3 | MAVROS，独占飞控 USB 并转发 QGC UDP |
| 4 | 飞控中心姿态诊断 |
| 5 | 检查、参数回读、姿态配对和安装辅助 |
| 6 | 当前会话对齐位姿预览 |
| 7 | 当前会话视觉定位转发到 MAVROS |
| 8 | QGroundControl；RViz 为可选独立窗口 |
| 9 | SUPER 地面诊断规划器 |
| 10 | 规划指令坐标预览，仅输出 debug topic |
| 11 | `first_hover_terminal.py`：默认只读检查；`--execute` 后等待显式起飞请求 |

### 启动前检查

```bash
timedatectl status
ip -br addr
ls -l /dev/serial/by-id/
pgrep -af '[f]astlio_mapping|[m]avros_node|[a]ligned_pose_preview.py|[f]light_pose_cov_transport.py'
```

系统时间应同步，雷达以太网应 UP，串口应存在。若进程已运行，继续检查现有会话，避免重复启动。

### 终端 1：MID360

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

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

初始化时放稳。已记录的正常窗口约为 `pub_age=0.02–0.035` 秒、`irx≈0`、`lidar_q=0`。`scan_publish_en=true` 给 SUPER 提供 `/cloud_registered`；单次启动阶段的 `No point, skip this scan!` 后继续初始化曾正常出现。

### 终端 3：MAVROS

```bash
source /opt/ros/humble/setup.zsh
ros2 launch mavros px4.launch \
  fcu_url:="serial:///dev/serial/by-id/usb-Matek_HKUST_UAV_NxtPX4_0-if00:2000000" \
  gcs_url:="udp://127.0.0.1:14555@127.0.0.1:14550"
```

QGC 的 **Application Settings → Comm Links** 中关闭 Pixhawk 自动连接、开启 UDP 自动连接。USB 由 MAVROS 使用；QGC 通过本机 UDP 接收转发，USB 线保持插着。

### 终端 4：姿态诊断

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/fastlio_body_debug.py
```

### 终端 5：检查并重新测量会话对齐

```bash
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
source ~/fastlio_ws/install/setup.zsh
source ~/super_ws/install/setup.zsh

ros2 service call /mavros/param/pull \
  mavros_msgs/srv/ParamPull "{force_pull: true}"
timeout 10s ros2 topic echo /mavros/state --once
```

确认 `connected: true`、`armed: false`，然后在未启动视觉转发的条件下暂时关闭融合，测量新会话：

```bash
ros2 param set /mavros/param EKF2_EV_CTRL 0
ros2 param get /mavros/param EKF2_EV_CTRL
ros2 param get /mavros/param EKF2_HGT_REF
python3 ~/Downloads/odom_age_check.py
python3 ~/Downloads/frame_alignment_check.py
```

保持飞机水平、静止，姿态配对运行 15 秒。用本次输出的 `Candidate rotation quaternion xyzw` 更新，替换下面四个占位符：

```text
python3 ~/Downloads/update_preview_alignment.py QX QY QZ QW
```

**每次重启 FAST_LIO，都重新测量；历史四元数不能直接复用。** 这个值对齐本次 LIO 世界和 PX4 世界，不是固定安装角。归档预览保留的是 10 月 7 日测量值，不能直接用于新会话。

### 终端 6：对齐位姿预览

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/aligned_pose_preview.py
```

在终端 5 查看：

```bash
timeout 10s ros2 topic echo \
  /super_debug/aligned_pose_cov \
  geometry_msgs/msg/PoseWithCovarianceStamped --once
```

第一份接受的飞控中心位姿作为固定零点。**零点不代表地面**；用户目前已把飞机放到地上，终端 11 仍需捕获实际起飞地面位姿。预览保留原时间戳，过期输入会报告 `fresh=False`，后续转发另有 0.25 秒门限。

### 终端 7：恢复已验证的地面视觉定位融合

先在终端 5 回读已确认的定位配置：

```bash
for px4_param_name in EKF2_HGT_REF EKF2_EV_POS_X EKF2_EV_POS_Y EKF2_EV_POS_Z EKF2_EV_QMIN; do
  printf '%s: ' "$px4_param_name"
  ros2 param get /mavros/param "$px4_param_name"
done
```

预期依次为 `3、0、0、0、0`。确认未解锁、新对齐预览时间新鲜，停止其他 raw／aligned／ground 视觉转发脚本，再恢复已验证的 XYZ／yaw 地面融合：

```bash
ros2 param set /mavros/param EKF2_EV_CTRL 11
ros2 param get /mavros/param EKF2_EV_CTRL
```

随后在终端 7 启动：

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 ~/Downloads/flight_pose_cov_transport.py \
  --ros-args -p publish_to_mavros:=true
```

不加此参数时仅发布 `/super_debug/flight_vision_pose_cov`。该脚本先在未解锁状态固定来源、几何和原点；之后允许继续传定位，**没有解锁、模式切换或控制指令**。解锁后的连续定位只通过模拟测试，尚未在真实飞行验证。

在终端 5 检查：

```bash
ros2 topic info /mavros/vision_pose/pose_cov
timeout 10s ros2 topic hz /mavros/vision_pose/pose_cov
timeout 10s ros2 topic echo /mavros/estimator_status --once
timeout 10s ros2 topic echo /mavros/local_position/pose --once
```

应有一个发布者、一个 MAVROS 订阅者，约 10 Hz，并检查当前水平／垂直位置和速度有效性。过去的地面确认不替代新会话检查。来源重启、对齐文件改变或原点不一致触发锁止后，先在未解锁状态排查，再恢复整个会话。

### 终端 8：QGroundControl

```bash
~/Downloads/QGroundControl-x86_64.AppImage
```

在飞行器的 **Analyze Tools → MAVLink Console** 运行以下只读检查：

```text
listener vehicle_visual_odometry 1
listener vehicle_local_position 1
listener manual_control_switches 1
listener actuator_armed 1
commander check
```

10 月 8 日在 POSCTL、解锁开关 OFF、Kill 已释放时，`kill=false`、`termination=false`，`commander check` 返回 OK。Kill ON 会阻止检查；QGC 的 “Flight termination active” 也可能由 Kill 引起。仅检查开关和状态不等于实际接管／停机验证。

### 终端 9–10：另行恢复地面规划调试

本次固定点悬停时保持这两个调试进程停止。单独进行规划地面验证时，在终端 5 为缺失的诊断配置生成一个副本：

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

未给目标时 `Wait for goal`／`WAIT: fresh /planning/pos_cmd` 正常。给目标后检查：

```bash
timeout 10s ros2 topic echo /rog_map/occ sensor_msgs/msg/PointCloud2 --once --field width
timeout 10s ros2 topic hz /planning/pos_cmd
```

如果需要 RViz，另开窗口加载现有配置：

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
rviz2 -d ~/fastlio_ws/mid360_super.rviz
```

`world` 与 `camera_init` 的单位变换仅用于当前规划／显示约定；缺失该 TF 时应核对帧再添加。RViz 的 `2D Goal Pose` 使用 `/goal_pose`，目标高度受配置 `click_height` 覆盖，最终目标也可能被规划器调整。始终以 `Get goal at` 日志为准。

### 终端 11：定点、离地 0.5 m 悬停

`first_hover_terminal.py` 是自包含文件，获取命令见 [完整工作流](docs/FIRST_HOVER_2026-10-09.md#获取脚本不使用安装包)。停止旧的 `ground_setpoint_transport.py` 和其他控制发送器，保留终端 1–8 的定位会话。

先只读检查：

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/first_hover_terminal.py
```

等待 READY 和 GROUND LOCK，确认地面 FC 高度 8.5 cm、本次目标离地 0.5 m。QGC Console 中，解锁开关保持 OFF，释放 Kill，在 POSCTL 下确认 `armed=false`、`kill=false`、`termination=false`，且 `commander check: OK`。

终端 11 Ctrl+C 停止只读实例，再启动执行实例；此时仍不自动起飞：

```bash
python3 ~/Downloads/first_hover_terminal.py --execute
```

再次 READY 后，在终端 5 才执行起飞触发。**下面这条命令会自动请求 OFFBOARD、解锁并起飞**：

```bash
ros2 service call /first_hover/start std_srvs/srv/Trigger '{}'
```

固定 XY／yaw，上升 0.415 m 后保持约 2 秒，再请求 AUTO.LAND。实际总时长取决于飞控起飞与落地检测。主动提前降落：

```bash
ros2 service call /first_hover/land std_srvs/srv/Trigger '{}'
```

遥控器切 POSCTL 接管后，程序停止位置发送，不覆盖人工模式。Kill 会直接停电机，不是平稳降落指令。定位／来源／时序异常会锁止，不自动重新解锁；只能在落地、未解锁后检查并开启新会话。实现细节、门限和当前验证范围见完整工作流。

## 10 月 8 日完成到哪里

完整过程见 [10 月 8 日部署记录](docs/DEPLOYMENT_CHECKPOINT_2026-10-08.md)。

| 项目 | 已有证据 | 尚待验证 |
| --- | --- | --- |
| FAST_LIO 时间修复 | Best Effort IMU 订阅保留；新会话姿态年龄通常 20–35 ms | 飞行及完整负载长期稳定性 |
| PX4 外部视觉接收 | `vehicle_visual_odometry` 有新鲜样本 | 飞行时持续接收 |
| XYZ／yaw 融合 | `EKF2_EV_CTRL=11`、`EKF2_HGT_REF=3`；地面两套位置接近，抬高／放回方向一致 | 实测距离标定、振动及空中性能 |
| SUPER 规划 | `/rog_map/occ` 宽度 5098；`FOLLOW_TRAJ`、重规划成功 | 实际轨迹跟踪 |
| 控制坐标预览 | 转换后的 ENU 位置／yaw debug 输出 | 完成保持、实际控制和边界限制 |
| 地面指令传输 | 未解锁约 20 Hz；PX4 `offboard_control_mode.position=true` | OFFBOARD 执行、退出／失联处理 |
| Position 检查 | 释放 Kill 后未解锁 `commander check: OK` | 首飞前新会话复查 |
| 首悬停参数 | 7 个参数已实际写入并回读一致 | 飞行中的限速和失联降落响应 |
| 首悬停预览 | 云端测试通过；debug 输出限定 | 尚未安装或运行于飞机 |
| Mac／Windows 远程桌面 | USB Wi-Fi 已联网 | NoMachine 连接未成功 |

历史 QGC 日志出现过 RC 解锁、位置失效转 Altitude 和起降检测；这些事件不能据此认定为已完成受控试飞。10 月 8 日尚无完整飞行控制状态机；10 月 9 日已实现本次单次悬停流程，仍待实际飞机验证。

## 参数记录与首悬停准备

### 当前定位及限速配置

| 参数 | 10 月 8 日确认的值 | 含义 |
| --- | --- | --- |
| `EKF2_EV_CTRL` | `11` | 水平位置＋高度＋yaw，未融合视觉速度 |
| `EKF2_HGT_REF` | `3` | Vision 高度参考；`1` 在此固件表示 GPS |
| `EKF2_BARO_CTRL` | `1` | 保留气压计融合配置 |
| `EKF2_EV_DELAY` | `0.0` | 当前记录值，未拿它补偿旧队列延迟 |
| `EKF2_EV_POS_X/Y/Z` | `0.0` | 已在预览中转换至飞控中心 |
| `EKF2_EV_QMIN` | `0` | 当前视觉质量门限 |
| `MPC_XY_VEL_ALL`、`MPC_Z_VEL_ALL` | `-1.0` | 使用单独的速度限制 |
| `MPC_XY_VEL_MAX` | `0.5 m/s` | 初期低速参数 |
| `MPC_Z_VEL_MAX_UP`、`MPC_Z_VEL_MAX_DN` | `0.5 m/s` | 初期升降速度参数 |
| `COM_OF_LOSS_T` | `0.5 s` | Offboard 输入丢失超时 |
| `COM_OBL_RC_ACT` | `4` | 此固件为 Land；实际响应尚未测试 |

保留的读值：`MPC_ACC_HOR=3`、`MPC_ACC_HOR_MAX=5`、`MPC_THR_HOVER=0.5`。正式 2 m/s 演示参数尚未设置。`configure_first_hover.py` 默认只读取并备份，显式 `--apply` 才修改上述七个限速／失联参数；这次物理写入使用的是逐条 ROS 命令，未确认参数备份文件已生成。

```bash
# 已融合且未解锁 POSCTL 时，只读核对并保存 JSON 备份
python3 ~/Downloads/configure_first_hover.py
```

### 起飞地面与悬停预览

飞控中心放在实际起飞地面时约高 **8.5 cm**。飞机已移到地面；仍不能把任意视觉零点直接当成实际地面。

保持同一个定位会话移到地面，放稳后记录 FC 地面位置 `z_ground`。预览首悬停目标为离实际地面 **0.5 m**，所以目标 ENU 高度是 `z_ground + 0.415 m`，即使地面在桌面初始化坐标中为负值也一样。若换供电导致重启，先执行上面的恢复／新姿态配对。

新脚本 `first_hover_preview.py` 在实际起飞地面初始化，固定 XY 和 yaw，发布 `/super_debug/first_hover_target`。参考曲线上升／保持／下降约 **9.781 秒**，参考垂直峰值速度 **0.2 m/s**；这不是实际飞行时长或降落检测。完成后持续输出原地面参考，状态／来源异常及观察到解锁／OFFBOARD 会锁止。

该脚本及参数辅助已通过云端模拟检查，但**尚未在飞机安装运行**。单独在终端 11 做未解锁预览时：

```bash
source /opt/ros/humble/setup.zsh
python3 ~/Downloads/first_hover_preview.py
```

它只输出 debug topic。10 月 9 日已新增完整单次悬停控制状态流程，见 `first_hover_control.py`／`first_hover_terminal.py`；控制实现通过隔离测试，机载启动与物理效果仍待回传。

## 代码和归档文件

| 文件 | 用途 |
| --- | --- |
| [flight_pose_cov_transport.py](flight_pose_cov_transport.py) | 本次会话定位转发，默认 debug；原时间戳／协方差，显式启用 MAVROS |
| [create_flight_pose_transport.py](create_flight_pose_transport.py) | 可读的纯文本安装器；校验源内容并备份旧文件 |
| [control_bridge_preview.py](control_bridge_preview.py) | 规划世界到 FC ENU 的未解锁坐标预览，AST 加载几何以避免重复 ROS 初始化 |
| [repair_control_preview_loader.py](repair_control_preview_loader.py) | 修复旧版预览的导入问题 |
| [ground_setpoint_transport.py](ground_setpoint_transport.py) | 未解锁 POSCTL／AUTO.LOITER 地面指令传输；20 Hz，解锁／OFFBOARD 锁止 |
| [ground_xy_fusion_transport.py](ground_xy_fusion_transport.py)、[ground_height_fusion_transport.py](ground_height_fusion_transport.py) | 10 月 8 日分步验证融合用的地面转发；观察到解锁后锁止 |
| [create_ground_xy_fusion_transport.py](create_ground_xy_fusion_transport.py)、[create_ground_height_fusion_transport.py](create_ground_height_fusion_transport.py)、[enable_ground_vision_yaw.py](enable_ground_vision_yaw.py) | 严格匹配、备份式安装／更新辅助；旧目标不匹配时停止 |
| [configure_first_hover.py](configure_first_hover.py)、[hover_reference.py](hover_reference.py)、[first_hover_preview.py](first_hover_preview.py) | 参数备份及低速参考／地面预览，尚未物理部署 |
| [first_hover_control.py](first_hover_control.py)、[first_hover_terminal.py](first_hover_terminal.py) | 终端 11 单次固定位置悬停；默认只读，显式请求启动，AUTO.LAND／接管／异常锁止，已做隔离测试 |
| [build_first_hover_terminal.py](build_first_hover_terminal.py) | 从主控制实现生成自包含版本；`--check` 验证没有分叉 |
| [control_chain_age_check.py](control_chain_age_check.py)、[ground_pose_snapshot.py](ground_pose_snapshot.py) | 同时检查消息新鲜度／接收间隔及地面定位快照 |
| [send_control_preview_goal.py](send_control_preview_goal.py) | 未解锁时发送沿规划 X 的小目标；不是沿机头的首飞指令 |
| [create_super_ground_diagnostic.py](create_super_ground_diagnostic.py) | 生成独立地面诊断 YAML，保留原始参数文件 |
| [当天提供的 SUPER YAML](configs/observed_2026-10-08/mid360_real_ros2.yaml) | 用户实际粘贴的配置归档；不会自动覆盖机载配置 |
| [fastlio_body_debug.py](fastlio_body_debug.py)、[frame_alignment_check.py](frame_alignment_check.py)、[update_preview_alignment.py](update_preview_alignment.py) | 名义安装变换、时间配对姿态测量、新会话四元数更新 |
| [aligned_pose_preview.py](aligned_pose_preview.py) | 名义杆臂／旋转／完整协方差变换，固定初始零点 |
| [odom_age_check.py](odom_age_check.py)、[stationary_position_check.py](stationary_position_check.py) | 深度 1 新鲜度及静止／原点一致性检查 |
| [raw_pose_cov_transport.py](raw_pose_cov_transport.py)、[aligned_pose_cov_transport.py](aligned_pose_cov_transport.py) | 早期仅 `EV_CTRL=0` 的传输实验，保留用于历史复现 |
| [patches/](patches/) 和 `apply_fastlio_*.py` | FAST_LIO 协方差、回调诊断及 IMU QoS 修复 |

旧式生成脚本可能依赖特定旧源块；获取仓库后优先复制独立的最新脚本。纯文本安装方式替代了之前粘贴受损的压缩 payload，不需要在聊天中重新复制 base64 数据。

发布前代码检查补齐了地面转发／控制预览的状态时间戳检查，并让参数辅助在等待服务时继续处理状态、写入前再次确认未解锁。这些最新修订及对应回归测试在云端通过，尚未替换飞机上的运行进程。

### 名义安装及坐标约定

雷达相对机头 yaw 约 +45°，雷达高于 FC 约 10 cm。FAST_LIO IMU–LiDAR 参数记录为 `extrinsic_T=[-0.011,-0.02329,0.04412]`、旋转单位阵，在线估计开启。当前预览采用这些**名义配置**，未完成动态外参／独立安装标定。

FAST_LIO 输出 `camera_init → body`，其位姿为 LIO IMU 原点。脚本先移至 FC 中心，再用本次配对的世界旋转及固定原点生成 `px4_enu_preview`。MAVROS 执行 ENU／NED 转换。四元数 `q` 和 `-q` 表示同一姿态。

SUPER 的 `world` 在当前上游代码数值上沿用 LIO 世界。轨迹 flag `1` 为常规轨迹，`2` 为备用轨迹；不要只根据消息常量名称把 `2` 当成停止。普通规划结束且飞机没移动时会再次生成轨迹，地面反复 `FOLLOW_TRAJ` 不代表飞机执行。

### FAST_LIO 修复与重新编译

10 月 7 日实际完成编译，重启后保留：

- 协方差在发布前取当前 `P`；使用 position-first 顺序，将 IMU 局部旋转误差转换至世界轴，保留交叉项。
- IMU 订阅改为 Best Effort、深度 10；当时约 1.275 秒的回调积压在观察窗口内消失。
- timing probe 分别显示输入到达年龄、队列和处理耗时。处理仅数毫秒时也可存在输入队列积压；平均 10 Hz 不代表每条位姿新鲜。

未改写传感器时间戳，未通过放宽过期门限掩盖延迟。没有将 DDS 具体重传机制断言为已经证实的唯一根因。

仅在新源安装或尚未包含修复时，停止相关进程、保留工作区备份，再从仓库根目录运行：

```bash
python3 apply_fastlio_covariance_fix.py
python3 apply_fastlio_timing_probe.py
python3 apply_fastlio_imu_qos.py
source /opt/ros/humble/setup.zsh
source ~/ws_livox/install/setup.zsh
cd ~/fastlio_ws
tar -czf ~/fastlio_install_before_fixes_$(date +%Y%m%d_%H%M%S).tar.gz install
MAKEFLAGS="-j2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select fast_lio --executor sequential
```

辅助脚本检查源块、备份并支持已经修复的状态。等价 patch 与辅助脚本选一种。完整 FAST_LIO 编译成功记录来自用户电脑，云端检查不代替完整机载工作区编译。

### SUPER 地面诊断配置

`mid360_ground_diagnostic_ros2.yaml` 是原文件的副本，只改变 `traj_opt/exp_traj/opt_accuracy` 为 `1e-4`、`traj_opt/switch/save_log_en` 为 `true`。地面测试随后恢复了规划输出，但这不足以证明优化失败的唯一原因或飞行参数已经合格。原配置包含最大速度 5 m/s，不是最终 2 m/s 展示配置。

## 常见问题

| 现象 | 当前解释／处理 |
| --- | --- |
| `ros2 interface show` 崩溃或 `.msg` 不存在 | 本次 CLI 名称／消息安装路径检查出错；生成的 Python `PositionCommand` 可构造。检查实际 `get_fields_and_field_types()`，不要因此重启整套节点 |
| `Context.init() must only be called once` | 旧预览在模块顶层初始化 ROS；新 loader 用 AST 只载入几何，保留运行中的定位节点 |
| `/planning/pos_cmd` 没有消息 | 先看是否收到目标和轨迹是否成功；INIT 等待目标时没有指令正常 |
| `MINCO ... line search ... failed` | 规划未生成成功；地面传输缺目标是后果。保留原 YAML 和日志，使用独立诊断配置复查 |
| 地面转发解锁后停止 | ground 脚本的锁止设计；用于连续定位的是终端 7 session relay，实际控制尚未实现 |
| `No valid position estimate` | 查看当前源新鲜度、融合参数及 FC 有效性；早期 ground relay 在解锁后停发也会造成定位丢失，历史日志不能单独定因 |
| QGC Kill／termination 报错 | 同时看 `manual_control_switches` 和 `actuator_armed`；本次释放 Kill 后 POSCTL 检查通过 |
| `timeout` 结束时 context／wait-set 错误 | 若之前已有连续正常数据，通常是 CLI 退出过程；另行检查实际节点是否仍运行 |
| `Input stamp age ... rejected` | 定位已过期，查看 timing probe；不能把长期 1 秒以上延迟视为短飞无影响 |
| NoMachine subscription 报错 | 原因尚待确认；先核对局域网手动 IP 入口、版本和服务状态 |

## 场地和剩余工作

实验室先测试：以起飞机头方向定义前向范围约 **0–5 m**，左右各 **2 m**，最高离地 **2 m（FC 中心）**。这些是用户给出的操作范围，尚未测量净空，也没有部署程序边界限制。

正式演示：**3 m 宽 × 6 m 长 × 3 m 高**，目标速度 **2 m/s**，FC 中心最高离地 **2 m**，不能碰到边界。飞机轴距 225 mm、5 寸桨；实际含桨外包尺寸和起飞点到各边距离待测。仅凭轴距不能确定所有姿态下的边界余量。

下一步完成当前地面 READY／POSCTL 预检 → 实际低速短时悬停 → 物理接管和定位／遥控失联降落验证 → 正式场地约束与规划控制接入 → 2 m/s 分阶段验证。软件流程已实现，实际飞行与场地边界约束仍待验证。

## 开发验证与历史记录

10 月 8 日发布前八个相关测试程序全部通过，其中纯参考曲线程序运行了三个 unittest；发现问题后的四个相关程序已重跑并通过。10 月 9 日新增单次悬停控制 DDS／服务测试，分别验证主实现与自包含版本。测试使用隔离的 ROS Humble Docker 环境、独立 ROS domain 和只读源码挂载；完整证据见当天工作流。

ROS 测试会创建模拟 MAVROS 服务及话题，不能和实际飞机使用同一 ROS domain。已安装 ROS Humble、MAVROS 消息／服务（含 State、ExtendedState、EstimatorStatus、RCIn、SetMode、CommandBool）及 SUPER PositionCommand 消息后，可在仓库根目录运行：

```bash
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=180
export ROS_LOCALHOST_ONLY=1
python3 tests/test_ground_xy_fusion_transport.py
python3 tests/test_ground_height_fusion_transport.py
python3 tests/test_control_bridge_preview.py
python3 tests/test_ground_setpoint_transport.py
python3 tests/test_flight_pose_cov_transport.py
python3 tests/test_configure_first_hover.py
python3 tests/test_first_hover_preview.py
python3 tests/test_hover_reference.py
python3 build_first_hover_terminal.py --check
python3 tests/test_first_hover_control.py
FIRST_HOVER_TEST_MODULE=first_hover_terminal python3 tests/test_first_hover_control.py
```

另保留以前的 ROS 传输、协方差和 C++ timing／IMU QoS 检查。当前模拟测试不验证实际飞行、停机或失联降落。

- [10 月 8 日：融合、规划、地面指令、首悬停参数、远程桌面进度](docs/DEPLOYMENT_CHECKPOINT_2026-10-08.md)
- [10 月 9 日：完整测试工作流与终端 11](docs/FIRST_HOVER_2026-10-09.md)
- [10 月 9 日：AX900、NoMachine 与重启恢复](docs/REMOTE_RECOVERY_2026-10-09.md)
- [10 月 7 日：时间问题及 QoS 修复](docs/DEPLOYMENT_CHECKPOINT_2026-10-07.md)
- [10 月 7 日完整历史 README](https://github.com/Haofei-Hu/Demo_HKU_open-day_SUPER/blob/c312c13432d6d2712b100e796cd13e603175537b/README.md)

## 固定版本与上游

| 项目 | 记录版本 |
| --- | --- |
| Ubuntu／ROS | 22.04／Humble，zsh；8 逻辑 CPU、约 15 GiB RAM |
| Livox driver | `livox_ros_driver2` 1.2.7 |
| FAST_LIO | ROS2，`a4743b095409588842a5b30ddfa27e29d2f99164` |
| SUPER | `2ad3419c127a617c6d7df6925e81a14175a9c096` |
| MAVROS | 2.14.0 |
| PX4 | HKUST_NXT_DUAL，Release 1.17.0，`d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |

[SUPER](https://github.com/hku-mars/SUPER) · [FAST_LIO ROS2](https://github.com/hku-mars/FAST_LIO/tree/ROS2) · [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) · [MAVROS](https://github.com/mavlink/mavros/tree/2.14.0) · [PX4 外部定位](https://docs.px4.io/main/en/ros/external_position_estimation) · [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard)
