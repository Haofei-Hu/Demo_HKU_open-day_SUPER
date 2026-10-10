# Ground localization continuation — 2026-10-08

## Latest confirmed status

This checkpoint records the October 8 work in chronological order. Earlier paragraphs marked “pending” or “next” describe the state at that point; the summary here and the final sections take precedence for resuming work.

| Area | Latest evidence and remaining work |
|---|---|
| Localization timing | Compiled FAST_LIO IMU Best Effort change retained across reboot; recent stationary surveys report roughly 20–35 ms body-pose age and no LiDAR queue buildup. |
| External vision | Native session relay in terminal 7 confirmed one publisher, one MAVROS subscriber and approximately 10 Hz. XYZ/yaw ground fusion and close preview/PX4 position agreement were observed. Physical continuation after arming remains untested. |
| Position preflight | Releasing the Kill switch with the arm switch OFF cleared the current blocker: `actuator_armed.kill=false`, `termination=false`, `ready_to_arm=true`, and `commander check: OK`. This is a disarmed preflight result. |
| First-hover parameters | Physical readback confirmed XY/up/down speed limits 0.5 m/s, overall speed overrides −1, Offboard-loss timeout 0.5 s and action 4 (Land). Actual speed limiting and loss/landing behavior have not been measured. |
| Planner/control wire | Ground diagnostic profile recovered planning; disarmed native position messages reached PX4. The planner and existing control preview are ground tools, with no completed flight controller. |
| Hover preparation | New parameter helper, smooth hover reference and disarmed hover preview passed cloud tests. These helpers have not been installed/run on the aircraft; the preview emits a debug topic only. |
| Floor origin | Aircraft is on a table/elevated surface. Actual takeoff-floor pose has not been captured; FC-centre ground height is approximately 0.085 m. |
| Remote desktop | USB Wi-Fi is connected to `Mars_Office` at `192.168.6.177/24`. Mac full-desktop access was selected; a Windows computer is also available. NoMachine connection remains unresolved. |
| Flight readiness | Startup/completion hold, flight-control integration, source-loss handling, laboratory bounds and formal venue limits remain unfinished. No verified controlled flight is recorded. |

The user has now requested uploading all today's code and documentation to GitHub. Publication itself must be verified after the commit/push; this checkpoint does not assert that a push has already succeeded.

## Current terminal numbering

The operator requested consecutive numbers. Current assignments are **1 MID360, 2 FAST_LIO, 3 MAVROS, 4 body debug, 5 checks, 6 aligned preview, 7 native session vision relay, 8 QGroundControl, 9 SUPER ground diagnostic, 10 disarmed control preview, 11 empty**. The earlier October 7 numbering is historical. Before the deferred move to the floor, stopping 9/10 was instructed but not confirmed by new process output; recheck actual processes rather than assuming those terminals are empty.

## Observed after reboot

FAST_LIO retained the compiled IMU Best Effort change. The reported window had odometry publication age 14–27 ms, IMU receive age rounded to 0.000 s, and no queued LiDAR scans. No package rebuild was needed for the reboot.

The new stationary alignment survey returned 150 body/704 IMU messages, 140 matches:

```text
quaternion xyzw: [-0.013679, 0.010126, 0.924049, 0.381896]
RPY degrees: [0.474, 1.892, 135.099]
spread mean/max: 0.046/0.095 degrees
pairing gap max: 0.008 s
body age mean/max: 0.023/0.031 s
```

The operator was instructed to back up and update the preview with this session's normalized quaternion. Aligned ROS transport then reported approximately 10 Hz, with observed intervals 0.078–0.115 s.

## QGroundControl and actual PX4 reception

In this QGroundControl version, AutoConnect is under **Application Settings → Comm Links**, not General. The screenshot confirmed **Pixhawk off, UDP on**. Restart QGroundControl to release an earlier USB link; MAVROS owns USB and forwards GCS data through:

```text
serial:///dev/ttyACM0:2000000
udp://127.0.0.1:14555@127.0.0.1:14550
```

The user successfully ran QGroundControl's MAVLink Console. `ver all` reconfirmed `HKUST_NXT_DUAL`, Release 1.17.0, git hash `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, build May 13 2026.

`listener vehicle_visual_odometry 5` produced a fresh FC-internal sample:

```text
sample before publication: 24248 us
publication age when printed: 0.035080 s
position: [0.00478, 0.00508, 0.00534]
pose_frame: 1 (NED)
velocity/angular velocity and velocity variance: NaN
position/orientation variances: approximately 0.00001
reset_counter: 0; quality: 0
```

This confirms PX4 receipt of at least the printed sample, not continuous fusion. Its total age at printing was approximately 59.3 ms. Missing velocity is expected for VISION_POSITION_ESTIMATE and must not be fused as valid velocity.

## Parameters and state confirmed

- MAVROS connected, disarmed, `AUTO.LOITER`.
- `EKF2_EV_CTRL=0`; horizontal velocity/position validity false, constant-position mode true.
- `EKF2_HGT_REF=1` is **GPS**, not barometer or vision.
- `EKF2_EV_POS_X/Y/Z=0.0`.
- `EKF2_EV_QMIN=0`, `EKF2_EV_NOISE_MD=0`, `EKF2_EVP_NOISE≈0.1 m`.
- Exactly one ROS pose-covariance publisher and one MAVROS subscriber.

The exact displayed firmware hash was successfully fetched read-only from PX4 upstream. Its [external vision parameters](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/params_external_vision.yaml) define bit 0 as horizontal position, bit 1 height, bit 2 velocity, bit 3 yaw; **EV_CTRL=1 enables XY only**. Its [height parameter definition](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/module.yaml) maps HGT_REF 0/1/2/3 to barometer/GPS/range/vision. EV_MAX_INTERVAL is 200 ms in this source; this does not constitute flight acceptance of the present 10 Hz stream.

## Prepared next step: disarmed XY fusion relay

[ground_xy_fusion_transport.py](../scripts/localization/ground_xy_fusion_transport.py) is a new, separate **bench-only** relay. The original zero-fusion transport remains available. [create_ground_xy_fusion_transport.py](../scripts/maintenance/create_ground_xy_fusion_transport.py) reproduces the new relay from the existing aligned relay with strict source checks, without modifying the original file.

The new relay preserves frame, stamp, pose and covariance; default maximum input age is 0.25 s. It requires a recent integer EV_CTRL in {0,1,9}, all three EV_POS offsets exactly zero (within 1e-9), QMIN=0, and a fresh connected/disarmed FC state. It blocks stale states/parameter confirmations and invalid inputs. Observing arming latches it off. It writes no PX4 parameters, sets no modes and sends no control setpoints. Polling/state callbacks are not an atomic hardware interlock; this relay is never a flight bridge.

Cloud ROS Humble tests passed using the exact MAVROS 2.14 State.msg generated in an isolated fixture: field preservation; invalid/repeated/stale input rejection; supply at EV_CTRL 0, 1 and 9; rejection of masks 2/3/4/8/15/-1; offset/quality mismatch; stale confirmations/state, disconnect; arming latch. These are synthetic transport tests, not evidence of EKF fusion.

## XY-only trial and heading prerequisite

The operator created the standalone relay after an exact-text conversion failed due to a different local source block. Its actual output reached about 10 Hz (intervals 0.082–0.118 s) with one publisher/subscriber. Startup state/parameter waits cleared before forwarding. The source-conversion helper is optional; use the standalone relay to avoid reliance on older local versions.

After instructions to set EV_CTRL=1, the reported ROS horizontal flags remained false. The uploaded aid-source log showed both EKF instances had `time_last_fuse=0`, `fused=false`, `observation=[nan,nan]`; both status instances had **tilt aligned but yaw not aligned**, with EV position/yaw inactive. In this exact firmware's [NED EV position branch](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/aid_sources/external_vision/ev_pos_control.cpp), observations remain NaN while yaw alignment is false. This is a processing prerequisite failure, not proof that FAST_LIO supplied NaN. The approximately 225 ms aid-source sample delay also includes the EKF's delayed fusion horizon; it is not the callback age diagnosed earlier.

The operator then confirmed EV_CTRL restored to **0**. `EKF2_MAG_TYPE=0` (automatic), `CAL_MAG0_ID=CAL_MAG1_ID=0`, and `listener sensor_mag 1` reported **never published**. This establishes no usable published magnetometer stream at this check; it does not prove that the board has no magnetometer hardware. A contemporaneous visual-odometry sample had finite near-zero position and unit attitude, 34.7 ms sample-to-publication delay plus 31.684 ms publication age when printed (about 66.4 ms total), pose_frame NED and no velocity.

## Prepared next step: XY plus vision yaw

The relay now permits **0, 1 or 9** only. Mask 9 enables horizontal position and vision yaw without vertical-position or velocity fusion. [enable_ground_vision_yaw.py](../scripts/maintenance/enable_ground_vision_yaw.py) updates the previous relay's unique parameter guard with a backup, retaining other gates. Updated isolated ROS tests exercise forwarding at 9 and rejection of unrelated masks, with the previous state and input checks retained.

The exact firmware's [EV yaw initialization code](https://github.com/PX4/PX4-Autopilot/blob/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/ekf2/EKF/aid_sources/external_vision/ev_yaw_control.cpp) can initialize yaw from valid NED EV attitude when tilt is aligned. This establishes the vision's local heading reference; the earlier attitude agreement with uninitialized FC yaw was not evidence of true-north calibration. Venue alignment and dynamic transform checks remain unfinished.

**At preparation time the 9-capable relay and fusion were unverified; the subsequent reported outcome is recorded below.** With EV_CTRL=0, stop relay 7, back up/update it, restart it and verify a single fresh approximately 10 Hz publisher. Then test EV_CTRL=9 while disarmed/stationary, checking both `estimator_aid_src_ev_pos` and `estimator_aid_src_ev_yaw` plus status flags. Keep FAST_LIO 2 and preview 6 running without recalibrating their world rotation during fusion. Before ending this bench experiment or restarting localization/preview, restore EV_CTRL=0 while connected and verify the readback. No flight-capable bridge or arming authorization is provided by these checks.

## Reported XY/yaw fusion success

The next uploaded console log shows **both EKF instances** with `cs_tilt_align=true`, `cs_yaw_align=true`, `cs_ev_pos=true`, `cs_ev_yaw=true`, `cs_inertial_dead_reckoning=false`. Height remains barometer-aided (`cs_baro_hgt=true`); external height/velocity aiding remain off (`cs_ev_hgt=false`, `cs_ev_vel=false`). No reported innovation faults or EV yaw fault.

Both position aid-source instances have finite observations, `fused=true`, `innovation_rejected=false`, and nonzero recent `time_last_fuse`. XY innovations range from approximately −5.74 mm to +3.51 mm in the reported snapshots, with small test ratios. Position observation variance is 0.01 m², consistent with the configured 0.1 m noise lower bound. Both yaw aid-source instances likewise report fusion without rejection; innovations 0.00069/0.00086 rad (approximately 0.040/0.049 degrees). These innovations describe estimator/measurement agreement at rest, not independent absolute localization accuracy.

This establishes actual ground XY and vision-yaw fusion in the printed samples. The approximately 230 ms aid-source timestamp gap includes delayed EKF fusion timing and does not contradict the earlier fresh input measurements. Sustained operation, active-selector ROS validity, translation/rotation scale and direction, height reference, venue limits and flight-control integration still require validation. Do not replace measurement stamps to reduce the displayed EKF delay.

Next ground check: obtain fresh `/mavros/estimator_status`, then record stationary preview and MAVROS local **XY** positions before and after a slow approximately 20 cm translation without changing heading/height. Allow settling at both endpoints and compare displacement rather than absolute origins. EV height is off, so preview and barometric local Z are not expected to track as an externally fused height pair. Leave FAST_LIO and preview initialized throughout the comparison.

## Reported approximately 20 cm translation

The next before/after ROS snapshots show `velocity_horiz_status_flag=true`, `pos_horiz_rel_status_flag=true`, `pred_pos_horiz_rel_status_flag=true`. Absolute horizontal position remains false, as expected without global aiding. `const_pos_mode_status_flag=true` alone does not negate the confirmed fusion: in this exact firmware, `get_ekf_soln_status()` also sets this flag while `vehicle_at_rest` is true.

Preview XY displacement was **[−0.050251, +0.199521] m**, magnitude **0.205752 m**; MAVROS local XY displacement was **[−0.049161, +0.197788] m**, magnitude **0.203807 m**. The displacement-vector difference was **0.002046 m**, direction difference about **0.178 degrees**. This is close agreement during the coarse manual translation, not a 2 mm absolute localization accuracy claim.

Preview Z changed **−0.008818 m**, while barometric local Z changed **+0.088645 m**, and the latter remained around −3 m in its existing origin. Neither is established venue floor height. External height fusion remains off; height origin and drift must be addressed separately before flight-height enforcement.

Next ground check is a settled approximately 30-degree left yaw rotation without source/preview restart or world recalibration. [ground_pose_snapshot.py](../scripts/analysis/ground_pose_snapshot.py) provides read-only five-second XY and circular-mean yaw snapshots with freshness checks and normalized quaternions. Compare before/after yaw changes and signs, then recheck position/yaw aid-source fusion. This validates tracking between preview and the fused output; independent mounting/extrinsic accuracy still requires dedicated checks.

## Reported left-turn attitude snapshots

The operator provided settled before/after quaternion snapshots. Normalized ROS ENU yaw was **100.601 → 158.126 degrees** in the preview and **100.643 → 158.209 degrees** in MAVROS local pose. Wrapped changes were **+57.525** and **+57.567 degrees**, respectively; endpoint yaw disagreement was approximately **0.084 degrees**. The opposite overall signs of the reported quaternions represent the same orientation and are not a frame error.

This demonstrates closely matching left-turn heading changes between the external preview and its fused consumer. The requested manual turn was approximately 30 degrees, while these snapshots describe approximately 58 degrees. The operator clarified that the physical turn was estimated without measuring the angle. Do not claim independently validated angular scale. Height configuration and ground height fusion remain pending.

## Height prerequisites reconfirmed

The operator read back **EV_CTRL=9, HGT_REF=1, BARO_CTRL=1, EV_DELAY=0.0**. External XY/yaw aiding is enabled, external vertical/velocity aiding is disabled, barometer height aiding is enabled, and the configured height reference remains GPS. The reference parameter alone does not establish actual GPS height aiding.

The exact firmware marks HGT_REF as reboot-required. EV_CTRL=11 would enable external XYZ/yaw but would be rejected by the current relay's {0,1,9} configuration gate; do not set it before preparing the relay. First verify upward external Z scale/sign through a measured, disarmed approximately 20 cm vertical translation without resetting the source/preview. This does not yet calibrate venue floor height or change height fusion/reference.

The operator's upward-translation snapshots changed preview position from [0.018377, 0.029878, 0.027990] m to [0.004330, 0.046201, 0.168699] m. Reported displacement is **[−0.014047, +0.016323, +0.140709] m**, horizontal magnitude 0.021533 m. Upward Z sign is correct. The operator clarified that the physical lift was estimated without measurement; the requested approximately 20 cm lift and reported approximately 14.1 cm rise therefore do not establish a scale error or scale agreement. A measured vertical lift with settled endpoints and a return-to-start check is still required. No height parameters or relay gates changed in this step.

The operator subsequently asserted the actual lift was approximately **14 cm**, consistent with the reported rise. This supports coarse vertical scale/sign agreement; it remains an estimated physical distance without a reported measured return-to-start check.

## Prepared disarmed external-height fusion test

[ground_height_fusion_transport.py](../scripts/localization/ground_height_fusion_transport.py) is a separate ground-only relay derived from the existing XY/yaw relay by [create_ground_height_fusion_transport.py](../scripts/maintenance/create_ground_height_fusion_transport.py). The creation helper identifies the unique integer {0,1,9} membership guard using Python AST, adds **11**, updates the node/class/startup labels, compiles before writing and preserves the original XY relay. This avoids exact multiline-source/indentation matching on the operator's local version. An existing differing target is backed up.

Isolated ROS Humble DDS tests passed for both relays: the height relay forwards unchanged pose/covariance/frame/stamp at 0/1/9/**11**, including a nonzero Z sample at 11; it rejects other masks including 15, bad inputs, offsets/quality mismatch, stale confirmations/state and disconnected states, and latches off upon arming. The original relay still rejects 11. These are transport tests, not physical height-fusion evidence.

Proposed operator sequence: while connected/disarmed, set/read EV_CTRL=0; stop only relay 7; create/run the separate height relay; confirm one approximately 10 Hz publisher; set/read EV_CTRL=11 and allow settling; inspect `estimator_status_flags` and `estimator_aid_src_ev_hgt` in QGC. Mask 11 enables XYZ/yaw without external velocity. Keep BARO_CTRL=1, HGT_REF=1 and EV_DELAY=0 for this initial aiding test. EV height bias estimation can retain the previous local-height origin, so successful fusion need not make MAVROS Z equal preview Z. Reference switching to vision and required FC reboot remain a subsequent step. Source/preview world alignment stays fixed. No physical 11 trial has been reported yet.

## Reported external-height fusion success

The operator confirmed setting/readback **EV_CTRL=11**. The uploaded console log shows both estimator status instances with `cs_ev_pos=true`, `cs_ev_yaw=true`, **`cs_ev_hgt=true`**, `cs_baro_hgt=true`, `cs_gps_hgt=false`, `cs_ev_vel=false`, `cs_in_air=false`, `cs_vehicle_at_rest=true`. No printed position/yaw rejections, innovation fault changes, barometer fault or dead reckoning.

Both external-height aid-source instances show **`fused=true`, `innovation_rejected=false`**, recent nonzero `time_last_fuse`, innovations **−0.00178 / −0.00158 m**, and test ratios approximately 0.00001. Observation variances were 0.01547/0.01549 m². The observations 2.85562/2.87033 m include the EV height bias in the existing local NED origin; they are not venue floor heights. Status instance 0 was printed about 0.91 s old, instance 1 about 0.003 s old; aid-source publication ages were about 0.031/0.063 s. These snapshots establish ground height fusion, not sustained/dynamic or flight readiness.

The next reference-switch sequence is prepared: EV_CTRL=0 and read back; stop relay 7; set/read HGT_REF=3 while connected/disarmed, retain BARO_CTRL=1 and EV_DELAY=0; reboot **only the flight controller** through its connected QGC MAVLink Console as required by parameter metadata; restart MAVROS 3 after USB returns, verify connected/disarmed and retained 0/3/1 parameters; restart height relay 7, verify one fresh publisher, then re-enable EV_CTRL=11. Keep LiDAR 1, FAST_LIO 2, body debug 4 and preview 6 running so the external world/origin remain unchanged. Do not re-estimate world rotation against uninitialized post-reboot FC yaw. Verify EV height/yaw/position fusion and ROS height comparison after reinitialization. The reference switch/reboot has not yet been reported as executed. Venue floor calibration remains pending.

## Reference-switch follow-up: parameter readback incomplete

The next requested post-restart check returned `connected=true`, `armed=false`, mode AUTO.LOITER, **HGT_REF=3**, **BARO_CTRL=1**, but the MAVROS ROS read for EV_CTRL returned **`Parameter not set.`** This does not establish the actual FC EV_CTRL value or a firmware parameter removal. Do not restore relay/fusion until the value is confirmed.

MAVROS 2.14's `~/pull` service uses `mavros_msgs/srv/ParamPull` (`bool force_pull` request; `bool success`, `uint32 param_received` response). A forced pull clears/refetches the local parameter cache without writing FC parameters. Refresh through `/mavros/param/pull`, then read EV_CTRL/HGT_REF/BARO_CTRL. If EV_CTRL is still absent, read it directly with QGC MAVLink Console `param show EKF2_EV_CTRL` and inspect the pull result before any next change. Actual post-reboot vision-reference fusion remains unverified.

The operator's next readback successfully returned **EV_CTRL=0, HGT_REF=3, BARO_CTRL=1**. The earlier missing EV_CTRL read was therefore transient at the ROS parameter interface; the pull response itself was not supplied. Resume the separate height relay, confirm one fresh publisher, enable/read back EV_CTRL=11, then inspect EV height aid/status and paired ROS positions. Actual post-restart fusion/height tracking awaits those outputs; floor calibration and flight-capable transport remain unfinished.

## Reported fusion after switching to vision height reference

The next uploaded FC log, with a reset uptime, reports both EKF status instances with EV position/yaw/height enabled, barometer enabled, GPS/EV velocity disabled, ground/rest true, and no printed innovation faults or position/yaw rejections. Both height aid-source instances report **fused=true, innovation_rejected=false**, fresh nonzero fusion timestamps. Observations are **−0.06165 m** in NED, consistent with positive ROS ENU height, with observation variance **0.01 m²**, innovations **−0.00425 / −0.00509 m**, and small test ratios. Status snapshots were approximately 0.67/0.77 s old when printed; height-aid publications approximately 0.058/0.097 s old. These are finite ground snapshots, not sustained flight validation.

Paired settled ROS positions were preview **[0.038566, 0.054202, 0.062505] m** and MAVROS **[0.039102, 0.055121, 0.058206] m**. MAVROS-minus-preview difference was **[+0.536, +0.919, −4.299] mm**, 3D norm **4.428 mm**. The former approximately −3 m local Z offset has disappeared in these samples, consistent with the confirmed HGT_REF=3 configuration and post-restart external-height initialization. Agreement measures the input/fused-output consistency, not absolute floor-height accuracy. Next test: settled snapshots before/after a gentle approximately 15 cm vertical lift and return, comparing both ROS Z changes while source/preview remain initialized. No arming, control bridge or venue floor calibration is provided by this outcome.

## Reported post-reference lift and return

Three settled preview/PX4 Z pairs were **0.061121 / 0.063964 m** before the lift, **0.221985 / 0.211268 m** at the upper position, and **0.055652 / 0.061004 m** after returning. Preview rise was **0.160864 m**, PX4 rise **0.147304 m**, a rise disagreement of **0.013560 m**. Returned-minus-initial Z was **−0.005469 m** in the preview and **−0.002960 m** in PX4. PX4-minus-preview Z disagreements at the three settled snapshots were **+0.002843, −0.010717, +0.005352 m**.

Both reported streams moved upward and returned close to their respective starting heights. These sequential endpoint samples support ground height-following consistency; they do not measure dynamic tracking latency, independent absolute height accuracy or motor-driven behavior. Physical lift magnitude was approximate. The operator was asked for takeoff FC-centre distances to the left/right long sides and rear short side, since takeoff is not necessarily centred. Pending control work requires the actual installed `mars_quadrotor_msgs/msg/PositionCommand` definition and workspace source inventory before selecting the planner-to-PX4 integration. Existing relay 7 remains ground-only and latches off upon arming; no flight-capable external-pose bridge, SUPER execution or fixed venue protection is implemented by these tests.

## Laboratory phase and message-inspection failure

The operator clarified that the formal venue takeoff location is **undetermined**: laboratory testing will come first, followed by the formal venue bounds. Continue laboratory control integration and disarmed checks; first-flight limits must reflect the actual laboratory space. Formal 3×6 m venue coordinates remain pending.

Ubuntu's crash dialog identified executable `/opt/ros/humble/bin/ros2`, command **`ros2 interface show mars_quadrotor_msgs/msg/PositionCommand`**, and exception **`rosidl_adapter.parser.InvalidResourceName`** with displayed offending text `//`. The operator's process/node checks still showed FAST_LIO, MAVROS, preview and height relay running, plus LiDAR/body-debug nodes. This establishes failure of the interface-inspection CLI, not a demonstrated core-node crash. The actual `.msg` contents and traceback have not yet been supplied, so malformed comment syntax is a hypothesis rather than a confirmed cause.

Next inspection bypasses `ros2 interface show`: read the installed `msg/PositionCommand.msg` using the ament package-share index, inspect the generated Python message's field/type metadata and instantiation separately, and list the workspace source directories. Preserve the running localization nodes and existing message files while collecting this evidence.

The operator's installed `.../share/mars_quadrotor_msgs/msg/PositionCommand.msg` path was absent. However, generated Python `PositionCommand` field introspection and construction **succeeded**. Actual fields include header, position/velocity/acceleration/jerk, angular velocity/attitude/thrust, yaw/yaw_dot, vel_norm/acc_norm, kx/kv, trajectory_id and trajectory_flag. The workspace source inventory contains only **SUPER**. The absent assumed `.msg` path does not establish absent compiled message support.

Read-only upstream inspection fetched hku-mars/SUPER master **2ad3419c127a617c6d7df6925e81a14175a9c096** to `/tmp/super-controller-review` (not the user repository). Its ROS2 CMake generates from **`ros2_msg/*.msg`**, a nonstandard source subdirectory. Its PositionCommand definitions match the reported generated fields. Its `fsm_ros2.hpp` stamps planner commands in **world**, sets flag **1 for regular / 2 for backup trajectory**, uses Best Effort depth-1 command QoS, and publishes position commands only in FOLLOW_TRAJ/EMER_STOP. Reaching WAIT_GOAL stops that command publication. These are upstream findings; the actual aircraft SUPER revision and corresponding publisher functions must be checked before relying on them for control. A bridge must handle planner/frame transforms, backup semantics, and planner-completion hold explicitly rather than blindly forwarding message rate or treating flag 2 as necessarily invalid.

## First control-coordinate preview prepared

The operator confirmed the aircraft SUPER commit is exactly **2ad3419c127a617c6d7df6925e81a14175a9c096**, supplied matching publisher functions and generated constants (including READY=1, EMER=2, COMPLETED=3, the misspelled TRAJECTROY_STATUS_ABORT=4). Its ROGMap ROS2 odometry callback copies FAST_LIO position/orientation numerically without a frame transform; the planner's `world` header is therefore a numerical alias of this session's `camera_init`, not PX4 ENU.

[control_bridge_preview.py](../scripts/planning/control_bridge_preview.py) implements a **disarmed coordinate preview**. It extracts the operator's current `~/Downloads/aligned_pose_preview.py` geometry conversion, obtains its fixed translation origin from same-stamp raw/aligned pose pairs, verifies orientation agreement and origin consistency, and converts planner flat-output attitude/IMU-origin position to nominal FC-centre position/orientation. It emits only `/super_debug/control_target` PoseStamped with the original planner timestamp, in `px4_enu_preview`. Flags 1/2 are accepted; inactive/invalid flags, wrong frames, nonfinite input, repeated/stale/future commands, stale/disconnected/armed FC state and stale localization are rejected. Origin/rotation mismatch and observed arming latch it off. It does not implement native PX4 setpoints, completion hold, floor/bounds, or flight-capable vision output. The original session/mounting assumptions remain preliminary.

Isolated ROS Humble tests **passed** using generated PositionCommand from the exact pinned upstream message and the existing exact MAVROS State fixture. The tests independently check analytic 90-degree world / 45-degree mounting geometry, yaw-dependent lever rotation, q/−q equivalence, accepted flags 1/2, original stamp/input preservation, rejected other flags/invalid inputs, stale state/localization, disconnect, origin mismatch and arming latch. Graph inspection confirms no `/mavros/` publishers. These are synthetic control-coordinate tests, not hardware execution.

Physical next step: restore FAST_LIO's registered-world scan output to feed SUPER while keeping full map/path/body-scan outputs disabled. First set/read EV_CTRL=0, stop relay 7 and aligned preview 6, then restart only FAST_LIO 2 with `publish.scan_publish_en=true`. Keep LiDAR 1, MAVROS 3 and body-debug 4 running. Confirm a single `/cloud_registered` and `/Odometry` publisher and rerun the stationary 15-second frame-alignment survey. Update the preview's session quaternion and restart its origin/fusion before bringing up control preview/SUPER. Full-cloud/planner load freshness, new calibration, physical control preview, native setpoint/hold behavior, flight-capable vision relay and laboratory limits are still pending.

## Registered scan restored and new session survey

The operator reported **one `/cloud_registered` publisher** (no subscribers yet) and **one `/Odometry` publisher**, with body-debug as the single current odometry subscriber. The 15-second stationary survey returned **150 body / 743 IMU / 148 matched** samples:

```text
new quaternion xyzw: [-0.010946, 0.009194, 0.978206, 0.207142]
RPY degrees: [0.771, 1.445, 156.097]
rotation spread mean/max: 0.040/0.082 degrees
stamp pairing gap max: 0.009 s
body age mean/max: 0.025/0.044 s
```

Reported FAST_LIO timing with registered-world scan publishing enabled remained fresh: pub_age approximately **18–31 ms**, irx rounded **0.000 s**, **lidar_q=0**, scan interval **0.100 s**, processing approximately **4.7–17 ms**. This is restored-cloud load with SUPER still stopped, not proof of freshness under full planner/control load.

Next: back up/update only the physical preview's top-level Q_WC assignment with the normalized new quaternion; restart preview 6 and height relay 7 while EV_CTRL=0; confirm a single fresh approximately 10 Hz vision publisher; set/read EV_CTRL=11 and compare preview/PX4 positions. The canonical archived October 7 quaternion in the repository is not this newly measured session value. No floor calibration or native controller is introduced by this restart.

## Reported paired positions after the new calibration

After the requested preview/relay/fusion restoration, the operator supplied preview **[−0.002944, +0.001184, +0.001545] m** and MAVROS **[+0.001950, +0.000504, +0.006072] m**. MAVROS-minus-preview difference was **[+4.894, −0.680, +4.527] mm**, 3D norm **6.701 mm**. This supports close settled output agreement in the new session; no new aid-source status log was supplied in this step.

Keep current localization/fusion terminals running. The next terminal assignment is **9 SUPER** (8 remains QGC). Start standalone `ros2 run super_planner fsm_node --ros-args -p config_name:=mid360_real_ros2.yaml`, create log directories if needed, and expect INIT/Wait for goal before a goal. Check `/cloud_registered` gains a subscriber, `/rog_map/occ` has nonzero width, and FAST_LIO timing stays fresh with empty scan queue under map-update load. Planner trajectory execution, physical control preview, flight-capable external-pose transport, and laboratory limits remain pending. Do not run the simulation launch against the real-data topics.

The operator then confirmed `/cloud_registered` has **one publisher / one subscriber** and `/rog_map/occ` width is **5098**. This establishes live SUPER map input/output; no new FAST_LIO timing lines were included. Next assignment is **10 control-coordinate preview**, loading the active Downloads alignment file and emitting only the debug target topic. Because the operator previously had trouble downloading generated files, provide an offline copy/paste installer carrying the tested source with SHA256 verification, compilation before writing and backup of an existing target. The payload roundtrip was verified locally. Expect `READY coordinates; WAIT: fresh /planning/pos_cmd` before a goal; confirm physical mapping startup/freshness before triggering a planner goal. The exact installed source remains reviewable in the repository. No native PX4 setpoints, flight relay or laboratory bounds are enabled.

## Control-preview loader repair

Physical terminal 10 exited with `Context.init() must only be called once`: its import executed the older deployed aligned preview's unguarded module-level `rclpy.init()` after control preview had already initialized its own ROS context. Other running localization/fusion nodes are separate processes and need no restart. The loader now extracts only the geometry function/constant dependencies using AST, skipping ROS startup, node construction and main execution. It also supports older previews that inline conversion in their callback. `repair_control_preview_loader.py` backs up and replaces only the deployed control script's top-level loader, checks the embedded payload SHA256 and compiles before writing.

Isolated ROS Humble regression tests passed, including an unguarded-startup fixture in an already initialized ROS context and an inline-conversion fixture, as well as the existing independent geometry/rejection/latch checks and graph confirmation of no MAVROS publishers. Repair payload and replacement roundtrips passed. Physical terminal 10 retry is pending; no aircraft control has been enabled.

The operator retried terminal 10 and reported `DISARMED CONTROL PREVIEW` followed by repeated `READY coordinates; WAIT: fresh /planning/pos_cmd`. This confirms the repaired loader starts on the deployed unguarded preview and the same-stamp localization mapping is accepted. No planner command has yet been observed. The next ground-only check is a single goal 0.30 m along planner-world X from a fresh raw odometry pose, preserving its heading. Exact upstream `setGoalPosiAndYaw` overrides incoming goal Z with configured `click_height` when above −5 (currently −0.05), and may relocate an occupied goal to a nearby cell within a 3 m search; inspect the planner's actual `Get goal at` output rather than assuming the requested target was unchanged. Keep the aircraft disarmed and all outputs in the debug preview; collect planner acceptance and converted `PREVIEW` output.

## First physical planner/control-preview stream

The operator reported `GenerateExpTrajectory SUCCESS`, state transition `GENERATE_TRAJ -> FOLLOW_TRAJ`, repeated successful replans, then `Traj finish` and `FOLLOW_TRAJ -> GENERATE_TRAJ`. With the actual aircraft stationary and no actuator/setpoint bridge, finishing trajectory time does not satisfy the FSM's actual-odometry goal-distance check, so repeated planning is expected. Terminal 10 reported regular flag=1 converted targets, yaw approximately 111.0–111.2 degrees, positions ranging in the supplied snapshots from about [-0.019, 0.044, 0.032] to [-0.299, 0.171, 0.219] m. This confirms physical planner commands reach the disarmed coordinate preview. It does not establish that the requested 0.30 m goal was accepted unchanged: `Get goal at` was omitted, and the target Z excursions require checking the planner's actual accepted target and configured click height before judging the path. No native FC control or flight readiness is established by this stream.

## Actual accepted goal and disarmed native-setpoint wire test

The operator supplied `Get goal at [0.350, -0.050, 0.150]`, goal yaw 0.147 degrees, and YAML `click_height: -0.05`, `click_yaw_en: true`. Exact upstream code first overrides requested Z with click_height, then calls `getNearestInfCellNot(OCCUPIED, ..., 3.0)`; the accepted target was moved upward by 0.20 m to a nonoccupied inflated-map cell. This explains the vertical target excursion and confirms the requested goal was not accepted unchanged. Do not disable occupancy inflation or alter mounting/fusion to remove this expected goal adjustment.

`ground_setpoint_transport.py` prepares a **disarmed AUTO.LOITER-only** wire check from `/super_debug/control_target` to `/mavros/setpoint_position/local`. At 20 Hz it emits only position and yaw, retains the input timestamp, labels the already-aligned numerical ENU pose `map`, and delegates ENU/NED conversion to MAVROS. It validates finite/unit-quaternion inputs, fresh FC/local/target timestamps and receipts, a 1 m radius and 30 degree heading difference for this ground test, one target source/one native writer, and a recent native-plugin `mav_frame=LOCAL_NED` confirmation. Bad inputs, lost services, parameter changes, expired inputs and incorrect mode block output; observed arming or OFFBOARD mode latch it off until process restart. These test gates are not laboratory/venue limits and the script is not flight capable. There are no mode/arming/parameter-write requests.

Isolated ROS Humble tests passed: approximately 20 Hz output, correct ENU XYZ and yaw with roll/pitch removed, original stamp preservation, invalid/stale/future/repeated inputs, duplicate writer, plugin frame changes/service loss, OFFBOARD and arming latch. Fixture traffic only; physical wire result is pending.

Exact MAVROS 2.14 source sends LOCAL_NED SET_POSITION_TARGET_LOCAL_NED with mask ignoring velocity/acceleration/yaw rate. Exact aircraft PX4 mavlink receiver publishes `offboard_control_mode` for accepted position requests while in AUTO.LOITER, but publishes `trajectory_setpoint` only in OFFBOARD. Therefore the ground wire check should inspect recent `listener offboard_control_mode 1` with position=true, velocity/acceleration=false; do not switch modes merely to obtain a trajectory_setpoint listener result. Operator guidance assigns this new helper terminal 11, keeps 1–10 running and uses QGC MAVLink Console in terminal 8 for the read-only listener. Experimental laboratory clear-space dimensions have been requested asynchronously; actual flight controls/vision output, hold/loss behavior and laboratory bounds remain pending.

The operator replied that the laboratory test should be limited to 5 m forward and at most 2 m laterally. Clarification is pending whether the lateral value means each side (4 m total) or total width (1 m each side). The previous FC-centre maximum floor height 2 m remains the stated height requirement. These are proposed operation limits, not measured laboratory wall dimensions. They must be expressed in the takeoff heading frame and calibrated floor origin; they are not implemented in the current 1 m-radius disarmed wire helper.

The operator clarified lateral limits as **2 m each side (4 m total)**. Proposed laboratory operation envelope: 0–5 m forward in the takeoff-heading frame, ±2 m lateral, FC-centre floor height at most 2 m. Calibration, enforcement and measured laboratory clearance remain pending; current ground wire helper is unchanged.

## Physical native-wire helper waits and brief output

The operator started terminal 11: recent plugin LOCAL_NED confirmation succeeded, but the local/target freshness gate blocked output. Subsequent pasted log shows the first SENT at 1791461792.306 after startup 1791461674.584 (~117.7 s), repeated SENT through 1791461796.256 (~4 s), then freshness warnings resumed for at least ~58 s. Positions during the brief burst ranged approximately [-0.019, 0.044, 0.027] to [-0.156, 0.000, 0.326] m, yaw 111.2–111.3 degrees. This is intermittent ROS native-setpoint publication, not sustained 20 Hz hardware transport, not a PX4 listener confirmation and not flight readiness. No freshness threshold has been relaxed. `control_chain_age_check.py` is a read-only depth-1 check of raw odometry, planning commands, converted targets and MAVROS local pose; request its 10-second results plus current planner/control-preview tail to identify which input is absent/old.

The operator's 10-second chain measurement received 100 raw odometry messages with age 0.011/0.025/0.041 s and maximum receive gap 0.117 s; 282 MAVROS local poses with age 0.001/0.008/0.017 s and maximum receive gap 0.041 s. It received **no planning commands and no converted control targets**. During this sample, neither localization latency nor MAVROS local-pose rate explains the native helper wait: the upstream planner is not sending commands. The current terminal 9 state/log tail was requested but not yet provided; do not assume WAIT_GOAL, a crashed process, invalid goal, or a specific PlanFromRest failure. Exact confirmed publisher runs only in FOLLOW_TRAJ/EMER_STOP. Keep the freshness gates unchanged and inspect current FSM state before changing goals/configuration or introducing hold behavior.

## Planner failure identified from physical screenshot

Terminal 9 screenshot shows repeated `MINCO TrajOpt failed, Line search reaches the maximum try number, assumptions not satisfied or precision not achievable`, `Minco exp_traj opt failed`, `OptimizationExpTrajInPolytopes for new path failed`, `PlanFromRest GenerateExpTrajectory failed`, and retry messages. Reported last cost-evaluation log repeats Energy ~2164.8579, time ~1.262834 s and displayed zero sampled penalty logs. `Opt finish` is printed regardless of solver return code; zero reported penalties do not establish convergence or continuous-path safety. Exact source prints the failure on a negative L-BFGS status, clears the trajectory and returns infinity. Its g_epsilon is disabled (0), past=3 and relative-cost delta comes from exp_traj.opt_accuracy; default max_linesearch is 64. Physical configuration/corridor data are not yet available, so the underlying reason for line-search failure is unresolved; do not blindly raise limits, accept negative solver results, remove collision penalties or claim a firmware defect. Ask the operator to stop only planner 9 to end repeated failed retries and supply current complete mid360_real_ros2.yaml before selecting a reversible diagnostic/tuning change. Remaining ground-only receivers continue waiting.

## Supplied full YAML and reversible convergence experiment

The operator supplied the complete configuration, now recorded verbatim (shell prompt removed) in `configs/observed_2026-10-08/mid360_real_ros2.yaml`. It retains upstream main convergence delta 5e-6, backup 1e-6, max_vel 5 m/s, main rho 12000, mass 1.64 kg and substantial constraint penalties; real inputs are /Odometry and /cloud_registered, raycasting enabled, click_height −0.05, virtual ground −1.0. This is an observed ground configuration, not a final 2 m/s bounded flight configuration.

In the supplied one-segment failed case, E≈2164.857918 and T≈1.262833990 imply 7E/(rho*T)≈0.999999838. For a rest-to-rest single minimum-snap segment with no active penalties, this is consistent with an almost stationary duration objective E(T)+rho*T. The full failed boundary/corridor case is not captured yet; this supports a numerical stopping hypothesis without proving root cause. Exact upstream disables gradient termination and uses past-cost delta, so test only the primary exp_traj.opt_accuracy at 1e-4 and enable failure logging. Create a **separate** `mid360_ground_diagnostic_ros2.yaml`, preserve original YAML and existing fixed-name logs before restarting planner 9. Do not change cost weights, speed/acceleration bounds, obstacle inflation, backup thresholds or solver-return handling in this experiment.

`create_super_ground_diagnostic.py` uses YAML scalar source marks to preserve comments, verifies exactly the two intended data changes, backs up source/previous diagnostic/existing exp/back/time CSV logs and creates the new profile. Tested against the actual uploaded configuration, including a repeated creation and previous-log preservation. A shorter literal two-value copy/paste variant was also checked against that exact input. Restarting only the YAML-reading planner requires no compilation. Keep aircraft disarmed, restart 9 with diagnostic config_name and send one fresh +0.30 m ground goal while all existing localization/preview/native ground receivers continue. Physical recovery is pending; if negative line-search failures recur, stop the retry and collect the newly enabled failed-case log for faithful reproduction instead of further blind tuning. Laboratory limits and actual 2 m/s trajectory/execution enforcement remain pending.

## Physical diagnostic-profile recovery observed

After creating/restarting with the diagnostic profile and submitting the new ground goal, the operator supplied screenshot logs showing repeated `PlanFromRest GenerateExpTrajectory SUCCESS`, `GENERATE_TRAJ -> FOLLOW_TRAJ`, successful replans and time-finished/replanning cycles. Reported single-segment time was ~1.289481408 s, energy ~2210.539557. Terminal 11 supplied eight consecutive SENT records at roughly one-second reporting intervals, from 1791462878.856 through 1791462885.856, with nominal transformed positions about [-0.018,0.044,0.030] through [-0.298,0.171,0.219] m and yaw 111.0–111.2 degrees. This demonstrates current ROS control-chain recovery and at least ~8 seconds of observed native-topic transmission under the diagnostic profile. It does not establish indefinite numerical stability or isolate causality from the planner/map restart and new goal. No new long-duration timing or PX4 receiver listener was supplied. The next read-only physical check is QGC `listener offboard_control_mode 1`: recent timestamp and position=true, velocity/acceleration=false, while aircraft remains disarmed AUTO.LOITER. No arming/mode switch is needed or requested.

## PX4 receiver confirmation

The operator ran QGC `listener offboard_control_mode 1`. PX4 reported timestamp 8814983579, ~0.019455 s old, position=True and velocity/acceleration/attitude/body_rate/thrust_and_torque/direct_actuator=False. This confirms current native position-control-message reception in PX4. The 19 ms value is elapsed time since the last PX4 uORB update, not measured end-to-end latency, not a trajectory coordinate echo, and not evidence of armed control execution. Existing ground vision/setpoint helpers still latch off on arming (setpoint helper also on OFFBOARD) and must not be used for flight.

Next work: obtain remote-control connectivity and actual mode-switch/emergency-stop verification (asked asynchronously); read preflight status and current RC/Offboard-loss/controller-speed parameters before preparing flight-capable relays, completion/loss handling and calibrated operating bounds. Exact aircraft source confirms `commander check` sends only RUN_PREARM_CHECKS and reports cached preflight pass status; it is not an arming request. COM_OF_LOSS_T is Offboard-loss timeout; COM_OBL_RC_ACT enumerates Position0/Altitude1/Stabilized2/Return3/Land4/Hold5/Terminate6/Disarm7, so read values rather than assuming safe default behavior. No flight authorization or mode/arming command has been requested.

Operator reports RC connected but mode switching/emergency stop **not yet verified**. Read the current RC mode/kill-switch mappings before instructing physical switch testing; do not infer that RC connectivity alone establishes takeover capability.

Operator corrected the RC status: the transmitter/aircraft were successfully connected previously, but **RC is not connected now**. Previous answer must not be treated as evidence of present connectivity or verified takeover. Next step is to restore the prior transmitter/model link with arming switch locked and inspect fresh PX4 input_rc (rc_lost/rc_failsafe/timestamp_last_signal), then proceed to switch mapping/preflight checks. No arming or OFFBOARD switch is requested.

The operator says restoring RC will require power-off/restart. Clarification is pending whether this affects only aircraft/FC or also the computer. Stop ground native output/planner/preview/vision processes before an aircraft power cycle or receiver rewiring; retain stored programs and the separate diagnostic profile. Actual switch mapping/takeover verification and current preflight/failsafe parameter outputs are still pending. If FAST_LIO/alignment are restarted, re-establish the measured session rotation and initial FC origin before restoring ground fusion/control outputs; do not blindly reuse archived calibration after a new LIO session. No aircraft reboot or software shutdown has been executed by the cloud assistant.

## Subsequent RC verification and latest restart

The operator restored RC reception. Fresh `input_rc` messages reported 16 channels, `rc_lost=false`, `rc_failsafe=false`, and changing stick values. Parameters confirmed throttle=channel 3, flight-mode selector=6, arm switch=5, kill switch=7; COM_RC_IN_MODE=3, COM_OF_LOSS_T=1 s, COM_OBL_RC_ACT=0 (Position). Flight-mode slots 1/4/6 contain 8/2/6: Stabilized/Position/Acro in the exact firmware, **not Offboard**. Slots 2/3/5 are unassigned. No mode parameter was changed.

The operator demonstrated slot 4 selecting POSCTL while disarmed. Switch messages showed arm_switch=3 and kill_switch=1, then kill_switch=3 after releasing kill. Preflight passed in Stabilized but initially failed in POSCTL. QGC identified `Navigation error: No valid position estimate`. Its message history also showed repeated actual `Armed by RC switch` and `Disarmed by RC switch` events, despite the intended disarmed check. The ground height relay permanently stops publishing after observing arming. This is a plausible contributor to position loss; the old relay's actual latch warning was not supplied, so causation is not conclusively established. Restarting relay 7 restored approximately 10 Hz publication. After setting EV_CTRL=11 the operator reported connected/disarmed POSCTL, horizontal velocity and relative position valid, and vertical position valid. Successful POSCTL preflight after this recovery has not been reported. None of these observations establish flight takeover or motor emergency-stop performance.

After another operator shutdown/restart, parameter pull succeeded with 922 parameters, EV_CTRL was staged at 0, and HGT_REF remained 3. The latest stationary survey is:

```text
Received body/imu: 150 703; matched: 141
quaternion xyzw: [-0.013266, 0.009775, 0.923592, 0.383021]
RPY degrees: [0.453, 1.833, 134.959]
spread mean/max: 0.043/0.115 deg
pairing gap max: 0.050 s
body age mean/max: 0.021/0.034 s
```

Instructions were supplied to back up/update only the active preview's Q_WC, restore preview/height relay, verify one approximately 10 Hz vision publisher, restore EV_CTRL=11, and start SUPER with `mid360_ground_diagnostic_ros2.yaml` and the control-coordinate preview. The operator answered that these were ready, without posting the new fusion/preview logs. Terminal 11 remained stopped.

To restore native **disarmed ground** testing in the verified POSCTL mode, `ground_setpoint_transport.py` now accepts POSCTL as well as its existing AUTO.LOITER mode. Freshness, frame confirmation, one-source/writer checks, 1 m/30-degree test bounds, and the permanent arming/OFFBOARD latch are retained. An isolated ROS Humble graph test passed in POSCTL and AUTO.LOITER, including 20 Hz ENU position/yaw output with source stamps, invalid/stale inputs, unsupported modes, duplicate writer, service/frame failures and both latches. The offline patch was checked for exact source reproduction, repeatability and unchanged latch/freshness/frame methods. This is cloud validation; physical patched output is still pending. It remains unsuitable for flight; laboratory/venue limits and a flight-capable localization/control chain remain unfinished.

The physical operator subsequently supplied eleven consecutive `SENT ENU` reports from patched relay 11, timestamps 1791467010 through 1791467020. Positions varied from approximately [0.020, 0.031, -0.015] to [-0.198, 0.251, 0.172], with heading 89.7–90.0 degrees. These establish native ROS publication in the latest session and successful relay gating at send time. The logger reports every twentieth publication at roughly one-second intervals; an independent physical rate measurement and new PX4 `offboard_control_mode`/visual-odometry snapshots are still pending. Repeated short trajectories while the stationary aircraft remains away from its goal are consistent with the previously observed planner replan behavior. No arming, OFFBOARD transition, flight or measured trajectory tracking is established by these setpoint reports.

The operator then supplied current-session PX4 snapshots. `offboard_control_mode` was 46.513 ms old with position=true and all other control fields=false. `vehicle_visual_odometry` was 51.544 ms old, with its sample another 32.515 ms earlier (approximately 84.059 ms total age at printing). NED position was [0.00062, -0.00552, -0.00391], yaw approximately 0 degrees, pose_frame=1 and reset_counter=0. Position/orientation variances were approximately 1e-5; velocity fields remained NaN as expected. These establish receipt of the displayed samples, not a flight or continuous timing acceptance.

The operator reconfirmed takeoff-floor FC-centre height approximately 0.085 m and that the USB-connected ROS computer **is onboard and flies with the aircraft**. Thus the stated 2 m maximum centre height corresponds to 1.915 m above a floor-initialized origin; neither an enforceable flight limit nor the final venue geometry has been installed yet.

## Prepared session vision relay, preview first

`flight_pose_cov_transport.py` initializes while disarmed against at least three identical-stamp raw/aligned samples using the active geometry, then continues pose forwarding through normal arming and mode changes. It preserves the aligned covariance/frame/stamp, freezes inferred origin, alignment-file digest and raw/aligned publisher identities, and latches off on a detected session reset/mismatch or arming before initialization. Fresh FC state, original pose age, finite symmetric positive-semidefinite covariance and recent CTRL=11/HGT_REF=3/zero EV_POS/QMIN=0 are required. Default output is `/super_debug/flight_vision_pose_cov`. Native output is an explicit immutable `publish_to_mavros:=true` option and requires one vision writer and a MAVROS subscriber. There are no arming/mode requests or actuator/setpoint publishers. Loss of source/state/parameter validity blocks vision; a flight-controller response to such loss still must be implemented and verified.

An isolated ROS Humble graph test passed debug-only default, native opt-in, exact data preservation, unchanged origin and continued output across simulated arming/OFFBOARD, covariance/age/state/configuration gates, duplicate native writer, startup-armed refusal and latched origin/file/source changes. No physical arming was requested or performed by the assistant. The checked offline installer `create_flight_pose_transport.py` verifies its compressed source SHA256, compiles before writing, is repeatable and backs up a differing existing file; its roundtrip/repeat/backup checks passed. The next physical action is only a parallel debug-output comparison, with the existing ground vision relay 7 retained and native setpoint sender 11 stopped. Physical replacement of relay 7, flight control, completion/source-loss behavior, speed/range/floor enforcement and final POSCTL preflight remain pending.

The assistant's chat rendering of the compressed payload was corrupted; the operator reported gzip CRC failure before any source file was written. The assistant acknowledged the error. The reviewed relay itself is unchanged. The installer was replaced with transparent plain source, retaining SHA256/compile/backup/repeat protections, and the revised installer passed exact-source, repeatability and backup checks. Use this plain-source installer for the physical retry rather than the earlier compressed chat block. Physical new-relay startup remains unverified.

The operator subsequently reported successful physical DEBUG startup: at timestamp 1791469242 the session node began and briefly awaited fresh FC state; at 1791469243.353614731 it logged `Session locked; first pose forwarded with original stamp/covariance`. The independent debug-topic rate check rose from 9.871 to 9.979 Hz, with observed intervals 0.078–0.117 s. This confirms the installed relay's ground initialization and debug publication; physical armed behavior is not established. Next, stop this temporary DEBUG process in terminal 11 and the old ground height relay in 7, then start the new session relay in 7 with `publish_to_mavros:=true`, retaining the active LIO/aligned-preview processes. Verify one native vision publisher, approximately 10 Hz and disarmed fusion recovery before proceeding to flight control. Terminal 11 remains free afterward; the lab flight controller and enforced range/speed/height limits are still unfinished.

## Native session vision and valid ground fusion

The operator subsequently supplied a native `/mavros/vision_pose/pose_cov` graph with **one publisher and one MAVROS subscriber**, and a measured rate of **9.974–10.039 Hz**, with displayed intervals **0.082–0.115 s**. This confirms the native relay replacement in terminal 7. Its command is:

```zsh
source /opt/ros/humble/setup.zsh
source ~/super_ws/install/setup.zsh
python3 ~/Downloads/flight_pose_cov_transport.py \
  --ros-args -p publish_to_mavros:=true
```

The matching state sample at ROS time 1791469601 reported connected=true, **armed=false**, **POSCTL**. At 1791469603 the estimator reported valid attitude, horizontal/vertical velocity, relative horizontal position, absolute vertical position and predicted relative horizontal position, with no acceleration-error flag. Absolute horizontal position remained false without global aiding. Constant-position mode was true at rest and does not independently negate the preceding fusion evidence. These are ground observations; no physical armed test of the new relay or controlled flight is established.

Current terminal assignments are **1 MID360, 2 FAST_LIO, 3 MAVROS, 4 body debug, 5 checks, 6 aligned preview, 7 native session vision relay, 8 QGC, 9 SUPER ground diagnostic, 10 disarmed control-coordinate preview, 11 empty**. Old ground vision relays must not run alongside terminal 7. Terminal 10 and the optional historical ground setpoint sender still latch off on arming; they are not flight controllers.

At this point the operator requested publication of the day's code and reboot runbook, then explicitly postponed the GitHub update until the end of the day to continue deployment. No October 8 commit or push had been performed at that earlier checkpoint; publication was subsequently requested again after the remote-access attempt. Flight-control integration, startup/completion hold, loss response and calibrated laboratory speed/range/height enforcement remained pending. Readback of MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP/DN, MPC_ACC_HOR/MAX, MPC_THR_HOVER, COM_OF_LOSS_T and COM_OBL_RC_ACT, plus a current disarmed POSCTL `commander check`, had been requested. The reply repeated the earlier state/estimator output rather than providing those new values. Later readbacks and the recovered preflight result are recorded below.

## Position preflight failure investigated

The operator then explicitly reported `commander check: FAILED` in Position, while Stabilized and angular-rate modes pass. No new detailed QGC reason or concurrent local-position/failsafe snapshot was supplied. The earlier ROS estimator-valid flags are insufficient to identify the current failing Commander check. Preserve existing fusion parameters while obtaining current QGC health details, `listener failsafe_flags 1` and `listener vehicle_local_position 1` with Position selected, stationary and disarmed.

Read-only inspection of the exact firmware's `HealthAndArmingChecks/checks/estimatorCheck.cpp` and `modeCheck.cpp` confirms that Commander considers local-position validity, freshness, accuracy and preflight consistency, rather than only MAVROS's estimator-status booleans. Position/velocity validity includes one-second freshness and recovery hysteresis. `commander check` requests checks and prints the cached vehicle-status result immediately, so allow the mode/check results to update before relying on a repeat. Mode-specific failure does not establish absent GPS as the cause, and no arming checks or accuracy thresholds have been disabled or relaxed.

The operator supplied concurrent flight-controller evidence after another failed check. `failsafe_flags` was 38.924 ms old and reported attitude, angular velocity, local position (strict and relaxed), local velocity and local altitude all valid, manual input present and position_accuracy_low=false. Global position and home were invalid and Offboard signal absent, but the displayed requirement masks 56/32/16384 do not require any of these for POSCTL (navigation state 2). These flags alone do not identify a Position-specific arming blocker.

`vehicle_local_position` was 7.589 ms old with sample 1.732 ms before publication: XYZ [0.01849, -0.01286, -0.03280] m NED, xy/z/v_xy/v_z valid, heading_good_for_control=true, dead_reckoning=false. EPH/EPV were 0.05987/0.03837 m and EVH/EVV 0.07503/0.04150 m/s; these are estimator uncertainty measures, not independent accuracy measurements. Local global-reference fields were unset, consistent with the no-GPS local deployment. Reset counters 32/6/28/2/2 are cumulative, not proof of ongoing resets without a time comparison. Next obtain fresh `health_report`, `vehicle_status` (actual and intended nav state, arming state and preflight result) and current QGC health-error text before modifying fusion, GPS/USB checks or any arming threshold.

The next `health_report` showed can_arm_mode_flags=0, can_run_mode_flags=8372167 (POSCTL bit set), health_error_flags=0 and arming_check_error_flags=17825920. Exact firmware enums decode the latter as remote-control (128), system (1048576) and global-position-estimate (16777216) categories. The report was 6.739 s old; it records the last published health result rather than an instantaneous measurement. A fresh `vehicle_status` confirmed actual and intended nav_state=2 (POSCTL), disarmed, failsafe=false, safety_off=true, power valid, USB connected and pre_flight_checks_pass=false.

QGC's **current Overall Status** explicitly showed **Kill switch engaged** and **Flight termination active**; the latest listed switch event was Kill engaged at 22:48:20 HKT. In the exact source, ManualControlChecks blocks all modes when kill_switch is ON, while SystemChecks emits "Flight termination active" when either actuator_armed.kill or actuator_armed.termination is true. Thus the second message can be caused by the same Kill switch; it does not by itself establish irreversible termination or a defective estimator. Release only Kill for the disarmed check, retain arm-switch OFF/throttle low, verify manual_control_switches.kill_switch=3 and arm_switch=3, and inspect actuator_armed.kill/termination plus a repeated check after settling. Do not arm, force-arm, remove the Kill mapping or disable checks to diagnose this.

The QGC history also contains actual earlier RC arming/disarming, older position-loss fallback to Altitude, and takeoff/landing detector events at 21:35 HKT. These are historical events and do not establish a verified controlled flight or current localization failure. Keep the distinction between arming-switch OFF and Kill ON explicit during subsequent checks.

## Position preflight recovered after releasing Kill

The operator subsequently supplied `manual_control_switches` with mode_slot=4, arm_switch=3 (OFF), kill_switch=3 (OFF) and termination_switch=0. `actuator_armed` reported armed=false, prearmed=false, **ready_to_arm=true**, lockdown=false, kill=false and termination=false. The repeated `commander check` returned **OK**. In combination with the preceding nav_state=2 and valid local-position snapshots, this establishes a successful disarmed Position preflight after the Kill blocker was removed; it does not establish flight execution, takeover performance or a completed flight controller.

The next step is the still-missing readback of speed/acceleration/hover-thrust and Offboard-loss settings. No limits, failover actions or mode-slot mappings have yet been changed for first flight. The native vision relay remains terminal 7, while terminal 10 is still the disarmed-only control preview and terminal 11 remains empty.

## Reported current limits and first-hover configuration prepared

The operator supplied MPC_XY_VEL_MAX=12.0 m/s, MPC_Z_VEL_MAX_UP=3.0 m/s, MPC_Z_VEL_MAX_DN=1.5 m/s, MPC_ACC_HOR=3.0 m/s², MPC_ACC_HOR_MAX=5.0 m/s², MPC_THR_HOVER=0.5, COM_OF_LOSS_T=1.0 s and COM_OBL_RC_ACT=0 (Position). No configuration write has been reported yet.

Exact firmware inspection confirms horizontal speed supports a 0.5 m/s cap and vertical limits have a documented minimum 0.5 m/s. MPC_ACC_HOR has documented minimum 2 m/s²; keep the reported acceleration and hover-thrust values while preparing a slowly varying hover target. SYS_VEHICLE_RESP can couple acceleration/jerk, MPC_XY_VEL_ALL can overwrite XY caps and MPC_Z_VEL_ALL can overwrite vertical limits. Deactivate only the two overall speed overrides with -1.0 while retaining their old values in the backup; set individual XY/up/down limits to 0.5 m/s. PX4 can also clamp manual/cruise/auto ascent/descent speeds to these caps, so capture those values before writing. Intended Offboard-loss settings are timeout 0.5 s and action 4 (Land). This is a configured fallback, not a verified hardware loss/landing response or a geofence; no mode change, arming request or flight setpoint is part of this step.

`configure_first_hover.py` provides a read-only default and explicit `--apply` path. It saves original affected/coupled parameters before the first write, requires fresh connected/disarmed POSCTL and current vision fusion CTRL=11/HGT_REF=3, latches any observed arming, applies only seven listed limits/fallback settings, verifies readback and preserves a journal of partial changes on rejection/timeout. Isolated ROS Humble service tests passed exact writes and untouched acceleration/thrust, read-only behavior, backup fidelity, state gates, missing parameters, rejected-write stop/journal and absence of MAVROS flight publishers. These tests do not establish actual aircraft parameter write success, physical landing behavior or flight readiness. Physical commands/readbacks and subsequent low-speed control testing remain pending.

## Physical low-speed settings confirmed; takeoff-floor reference pending

The operator executed all seven settings and reported successful parameter responses. Subsequent readback confirmed **MPC_XY_VEL_ALL=-1.0, MPC_Z_VEL_ALL=-1.0, MPC_XY_VEL_MAX=0.5, MPC_Z_VEL_MAX_UP=0.5, MPC_Z_VEL_MAX_DN=0.5, COM_OF_LOSS_T=0.5, COM_OBL_RC_ACT=4**. These are confirmed configured values, not a measured flight-speed cap or tested loss/landing response. No acceleration, hover-thrust, mode-slot or fusion change was requested in this step. A parameter-dump backup was instructed, but its file output was not supplied; do not claim confirmed physical backup creation.

The operator clarified that the aircraft is **on a table or another elevated surface**. The aligned session origin must therefore not be treated as the takeoff floor. Keep the current localization session if moving the powered disarmed aircraft to the floor; capture its settled FC position there and calculate a target relative to that position. For a 0.5 m FC-centre floor height and ground FC height 0.085 m, the target is the captured ground ENU Z plus **0.415 m**, even if the ground position has negative Z in the existing table-initialized frame. Do not rebase a running fused vision stream just to make the ground coordinate zero.

`hover_reference.py` prepares a fixed-XY/yaw smooth up/hold/down reference (peak reference vertical speed 0.2 m/s, target FC floor height 0.5 m, hold 2 s, default reference duration 9.781 s). Three offline tests passed floor/offset calculations, fixed endpoints/yaw, finite-difference speed and stationary joins, and invalid-input rejection. This reference duration is not a measured flight/landing duration.

`first_hover_preview.py` consumes the fresh FC pose and native session vision while disarmed POSCTL, checks the confirmed limits/fusion/fallback parameters, freezes source identities and the initial ground pose, and publishes only 20 Hz `/super_debug/first_hover_target`. Completion holds the original ground reference. Observed arming/OFFBOARD or post-initialization source/state/parameter/frame/movement failures latch it off. Its isolated ROS DDS test passed an old-table-origin negative ground-Z example, target conversion, fixed XY/yaw, output frequency, completion hold, failure latches and absence of native flight publishers. It has **not been installed or run on the aircraft** and remains a ground preview, not a flight controller.

## Remote access before switching power and moving to the floor

The operator explained that the computer is physically aboard the aircraft, available external cables are too short, and requested connecting another computer to control it remotely before switching to aircraft power. The remote client is **macOS**, and the requested experience is the **existing complete Ubuntu desktop, including terminals, QGC and RViz**. A Windows client is also available. NoMachine was recommended for the desktop connection; SSH/tmux was suggested later for persistent terminal work, but neither SSH nor tmux setup has been verified.

The operator inserted a USB wireless adapter and supplied these current interfaces:

| Interface | Reported state | Connection/address |
|---|---|---|
| `wlx90de80cd9b4b` | Wi-Fi connected / UP | `Mars_Office`, **192.168.6.177/24** |
| `enp4s0` | Ethernet connected / UP | `livox-mid360`, **192.168.50.125/24** |
| `enp5s0` | Ethernet unavailable / DOWN | No current address |
| `enp1s0`, `enp2s0` | Ethernet unavailable / DOWN | No current address |

The historical wired client address **192.168.1.102 is not the current remote connection target**. Guidance was to connect the Mac to the same `Mars_Office` network, install NoMachine on both computers from <https://www.nomachine.com/download>, and create a manual LAN connection with protocol **NX**, host **192.168.6.177**, port **4000**, Ubuntu username **mars** and the Ubuntu login password entered locally in the client. The private IPv4 address is a reported DHCP/session address, not a guaranteed permanent assignment; recheck it after reconnecting or rebooting. No account password is stored in this repository.

The connection did not succeed. The user reported **`no subscription found on this service`**. A possible cause is selecting NoMachine's Network/cloud service instead of the manual LAN connection, but no screenshot, product/version output or server status was supplied to confirm that diagnosis. Manual IP connection was prescribed; **successful remote login, desktop sharing, installed product/version and any need for a paid plan have not been established**. Changing the client to Windows does not by itself resolve an unavailable server or blocked network path.

These diagnostic commands were requested but their results were not supplied:

On onboard Ubuntu, terminal 5:

```bash
sudo /usr/NX/bin/nxserver --status
sudo ss -ltnp 'sport = :4000'
sudo ufw status
sudo /usr/NX/bin/nxserver --version
```

On the Mac connected to the same network:

```bash
ping -c 3 192.168.6.177
nc -vz -G 3 192.168.6.177 4000
```

A missing `/usr/NX/bin/nxserver` would indicate that this server path is absent, and calls for checking the Ubuntu installation. The service/listener/network checks should determine the next fix before any firewall change or paid subscription recommendation. They are diagnostic commands, not reported successful results.

The move to the floor and switch to aircraft power remain deferred until remote access is established. **No floor snapshots, successful power transition or remote operation after removing the external monitor have been supplied.** Preserve the running localization processes and current power while establishing access. A later computer/FAST_LIO restart requires process restoration and a fresh measured session alignment; do not reuse a previous world's quaternion or interpret the table-initialized origin as floor zero.

## Next deployment work

1. Resolve the NoMachine LAN connection and verify the Mac can operate the existing Ubuntu desktop; then verify wireless access survives the intended power/monitor arrangement.
2. Move the disarmed aircraft to the actual takeoff floor, capture settled local/native vision poses and confirm freshness. The 0.5 m first-hover reference is the captured ground FC Z plus 0.415 m; the table origin is retained numerically if the localization session stays powered.
3. Install and validate the disarmed hover preview before preparing a flight-capable controller with startup hold, completion/landing and source-loss response. The native vision relay can support the continued session, but it does not send flight commands.
4. Implement and validate the laboratory operating envelope of 0–5 m forward, ±2 m lateral and FC-centre floor height at most 2 m. Formal 3×6×3 m venue geometry, 2 m/s operation and boundary clearance require separate calibration and testing.

## Publication review — 2026-10-09 HKT

The October 8 work was consolidated for the requested GitHub update. README now starts with the current numbered reboot recovery, Wi-Fi/NoMachine status, fresh-session alignment and native vision restoration. Earlier complete README content remains accessible through the October 7 commit; this checkpoint retains the chronological physical evidence.

Read-only code review found two issues and the final archived sources correct them:

- Ground XY/height relays and the control-coordinate preview now recheck the MAVROS State header age when publishing, as well as the time since receipt. A State message already 1.9 seconds old on arrival must not still pass after another 1.9 seconds. The source-generating installers remain consistent with these changes.
- The first-hover parameter helper processes State callbacks while waiting for a parameter service and checks fresh connected/disarmed POSCTL again immediately before sending a write. Delayed service availability must not permit a mutation after state expiry or observed arming. Read-only parameter inspection remains available.

Eight isolated ROS Humble test programs passed: ground XY fusion, ground height fusion, control-coordinate preview, ground setpoint transport, session flight-pose covariance transport, first-hover parameter configuration, first-hover preview, and hover reference. The reference program ran three unittest tests. The four affected programs were rerun successfully after the fixes, covering cumulative State age and delayed-service expiry/arming. Containers used distinct ROS domains, disabled networking and read-only repository/message-fixture mounts. Python source compilation and installer/canonical-source consistency were also checked.

These publication-review fixes have not been copied to or validated on the aircraft. The verified physical low-speed values and ground fusion remain the October 8 observations; no further physical flight, floor reference or remote-desktop success is implied by the cloud tests.
