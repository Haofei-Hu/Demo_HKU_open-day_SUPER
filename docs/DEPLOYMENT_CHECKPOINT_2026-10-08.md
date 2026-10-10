# Ground integration checkpoint — October 8, 2026

This checkpoint summarizes the ground integration stage. Subsequent work completed remote desktop operation and a successful automatic takeoff, hover, and landing; see [current progress](PROGRESS_2026-10-10.md) and the [flight workflow](FIRST_HOVER_2026-10-09.md).

## Stage results

| Area | October 8 result |
| --- | --- |
| Localization timing | Compiled Best Effort IMU subscription retained across reboot; approximately 20–35 ms pose ages and no sustained LiDAR queue |
| External vision | Actual PX4 reception, XY/yaw fusion, then height fusion and vision height reference |
| Position preflight | Successful disarmed POSCTL check after releasing Kill |
| Low-speed settings | Seven speed/Offboard-loss settings written and read back |
| SUPER mapping | Registered-cloud input and occupancy output active |
| SUPER planning | Trajectory generation/replanning and converted ground command preview |
| Native setpoint wire | Position/yaw messages reached PX4 during disarmed tests |
| Continuous vision relay | Native opt-in deployed; fixed-session relay replaces arming-latched bench relays |
| Flight control | Smooth reference and ground preview prepared; complete flight milestone followed later |

## Firmware and external-vision transport

QGC used MAVROS UDP forwarding while MAVROS owned the USB serial port. The aircraft firmware was HKUST_NXT_DUAL, PX4 1.17.0, hash `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`.

A PX4 `vehicle_visual_odometry` snapshot had NED pose frame, finite position/quaternion, 24.248 ms sample-to-publication delay, and approximately 35.080 ms publication age. Position/orientation variances were approximately 1e-5; velocity was NaN, as expected for the pose-only transport.

Exact-firmware parameter interpretation:

| Parameter | Meaning |
| --- | --- |
| `EV_CTRL` bit 0 | Horizontal position |
| bit 1 | Height |
| bit 2 | Velocity |
| bit 3 | Yaw |
| `HGT_REF` 0 / 1 / 2 / 3 | Barometer / GPS / range / vision |

The no-GPS deployment uses external local position rather than a global position reference. Absence of valid global/home position alone is not the POSCTL blocker. The recorded firmware's EV interval setting is 200 ms; a nominal stream frequency alone does not establish timing quality.

## Heading initialization and XY/yaw fusion

The first XY-only trial did not fuse: tilt was aligned but yaw was not, EV observations remained NaN, and `time_last_fuse=0`. In the exact NED EV branch, yaw alignment is required before processing position observations. The NaN aid-source observation therefore did not establish NaN FAST_LIO output.

No usable `sensor_mag` stream was published during the check. Enabling EV yaw with `EV_CTRL=9` established local heading and XY fusion. Both EKF instances then showed tilt/yaw alignment, EV position/yaw aiding, no inertial dead reckoning, finite fused observations, and no rejection. XY innovations ranged approximately −5.74 mm to +3.51 mm; yaw innovations were approximately 0.040/0.049°. These are measurement/estimator agreement at rest, not independent accuracy measurements.

Ground translation comparison:

| Stream | XY displacement | Magnitude |
| --- | --- | --- |
| Aligned preview | `[-0.050251, +0.199521] m` | 0.205752 m |
| MAVROS local pose | `[-0.049161, +0.197788] m` | 0.203807 m |

The displacement-vector difference was 2.046 mm and the direction difference approximately 0.178°. Physical movement was approximately 20 cm, without precision measurement.

A left-turn comparison yielded preview yaw 100.601→158.126° and MAVROS yaw 100.643→158.209°. Both tracked approximately +57.5°, with endpoint disagreement approximately 0.084°. The physical angle was estimated, so this established consumer agreement rather than independently calibrated angular scale. Quaternion sign reversal is orientation-equivalent.

## Height fusion and reference

A coarse upward motion changed preview Z by +0.140709 m, consistent with an approximately 14 cm lift. With `EV_CTRL=11`, both estimator instances showed EV height/position/yaw aiding, fused height observations without rejection, and height innovations approximately −1.78/−1.58 mm.

The original local Z retained a roughly −3 m barometric-origin offset. Switching the configured height reference to vision (`HGT_REF=3`) and reinitializing the FC established close preview/local Z agreement. The barometer remained enabled (`BARO_CTRL=1`) and EV velocity aiding remained disabled.

A raise-and-return sequence:

| Pose | Preview Z | MAVROS local Z |
| --- | --- | --- |
| Initial | 0.061121 m | 0.063964 m |
| Raised | 0.221985 m | 0.211268 m |
| Returned | 0.055652 m | 0.061004 m |

The aligned session origin still did not define floor height. The physical FC-centre ground clearance was approximately 0.085 m; subsequent hover control captures the actual floor-rest pose independently.

## Mapping and planner integration

The deployed SUPER revision was `2ad3419c127a617c6d7df6925e81a14175a9c096`. Generated Python `PositionCommand` introspection/construction succeeded despite a failed `ros2 interface show` invocation and a missing assumed `msg/` path. The package generates messages from `ros2_msg/`.

The publisher labels its numerical LIO coordinates `world`, uses flag 1 for regular trajectories and 2 for backup trajectories, and emits commands in FOLLOW_TRAJ/EMER_STOP. WAIT_GOAL has no command stream. A flight bridge must explicitly handle frames, completion hold, and loss of commands.

Registered-cloud input had one subscriber and `/rog_map/occ` width was 5098. Restored cloud publication retained a recent 18–31 ms odometry-age window with no LiDAR backlog. This was an observed load window, not a full flight-load benchmark.

The ground coordinate preview applies session/world and nominal mounting/lever geometry and publishes only `/super_debug/control_target`. An AST-based geometry loader fixed repeated ROS context initialization caused by importing an executable preview module. Same-stamp pairing and fixed-origin checks then initialized correctly.

Successful planning reached FOLLOW_TRAJ, produced converted flag-1 commands, replanned, and finished trajectory time. With a stationary aircraft away from its goal, the FSM generated another trajectory; this was expected ground behavior.

An accepted goal `[0.350, -0.050, 0.150]` differed from the configured click height −0.05 m. The planner searched for a nearby unoccupied inflated-map cell and moved the goal upward by 0.20 m. Accepted goal logs, rather than clicked coordinates alone, are needed when assessing paths.

Repeated MINCO line-search failures interrupted the command stream during another ground trial. A separate diagnostic copy changed `exp_traj/opt_accuracy` to `1e-4` and enabled planner logging; planning subsequently recovered. This experiment did not establish a unique solver root cause. The [archived YAML](../configs/observed_2026-10-08/mid360_real_ros2.yaml) retains the original snapshot, including the 5 m/s planning limit.

## Native ground wire and continuous vision relay

The ground setpoint helper sent approximately 20 Hz position/yaw targets through MAVROS. PX4 reported `offboard_control_mode.position=true`, with other control fields false. In the pinned receiver, this can be published outside OFFBOARD; native trajectory-setpoint consumption requires OFFBOARD. The wire test was therefore distinct from flight execution.

The original ground vision relays latch off after observing arming. Repeated RC arming and position-loss events motivated a separate continuous session relay. `flight_pose_cov_transport.py` initializes disarmed, preserves original pose/stamp/covariance, freezes origin/geometry/source identities, and continues after arming. Changes in those identities latch the relay off.

Native relay deployment produced one publisher and one MAVROS subscriber at 9.974–10.039 Hz, with displayed intervals 0.082–0.115 seconds. Connected/disarmed POSCTL and valid attitude, horizontal/vertical velocity, relative horizontal position, and vertical position were observed. Absolute horizontal position remained invalid without global aiding.

## RC and POSCTL preflight

Recorded channel mapping was throttle 3, flight mode 6, arm 5, and Kill 7. Mode slot 1 selected Stabilized and slot 4 selected POSCTL. RC input was present and not in failsafe.

A failed Position preflight coincided with QGC `Kill switch engaged` and `Flight termination active`, despite valid local position and velocity. In this firmware, the termination message can also be raised by the Kill state. Releasing Kill with the arm switch OFF yielded:

```text
mode_slot=4, arm_switch=3, kill_switch=3
armed=false, ready_to_arm=true, kill=false, termination=false
commander check: OK
```

No arming checks or accuracy limits were disabled to obtain this result.

## Low-speed configuration

| Setting | Confirmed value |
| --- | --- |
| `MPC_XY_VEL_ALL`, `MPC_Z_VEL_ALL` | −1.0 |
| `MPC_XY_VEL_MAX` | 0.5 m/s |
| `MPC_Z_VEL_MAX_UP`, `MPC_Z_VEL_MAX_DN` | 0.5 m/s |
| `COM_OF_LOSS_T` | 0.5 s |
| `COM_OBL_RC_ACT` | 4: Land |

Acceleration and hover-thrust readbacks were retained: `MPC_ACC_HOR=3`, `MPC_ACC_HOR_MAX=5`, and `MPC_THR_HOVER=0.5`. The parameter helper supports read-only backup and explicit application with fresh connected/disarmed POSCTL checks and partial-write journaling.

The initial reference used a 0.5 m FC-centre height, a 2-second hold, and a 0.2 m/s peak reference speed. Floor capture prevents treating a table-initialized visual origin as the takeoff floor. The later 1 m / 6-second controller and successful flight are covered by the current runbook.

## Verification and subsequent work

Eight isolated ROS Humble programs passed during this stage: ground XY/height relays, command preview, ground setpoint wire, continuous vision relay, parameter helper, hover preview, and reference. Review also fixed cumulative State-header-age checks and rechecked disarmed state after delayed parameter-service availability.

By this checkpoint, remote desktop setup and flight execution were the next stage. Their later outcomes are recorded in [remote recovery](REMOTE_RECOVERY_2026-10-09.md) and [current progress](PROGRESS_2026-10-10.md). Planned laboratory limits are 0–5 m forward, ±2 m lateral, and FC-centre height ≤2 m. Formal surveyed boundaries and planner-driven flight remain unfinished.
