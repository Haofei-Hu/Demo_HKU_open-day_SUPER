# Next development stage

The automatic takeoff, fixed-position hover, and landing milestone is complete. Development now moves to repeatability, localization quality, failure response, and planner-driven flight.

## 1. Quantify and repeat the successful flight

Record the controller profile, phase transitions, FC mode/arming state, local and external-vision poses, estimator status, RC, and landing state. Keep PX4 ULogs when available. Measure height and lateral motion independently where practical; target values alone are not tracking-error measurements.

Compare repeated flights for horizontal drift, height/yaw stability, source resets, input age, and landing consistency. Save controller output without changing the explicit-start workflow:

```bash
source /opt/ros/humble/setup.zsh
mkdir -p ~/Downloads/hover_logs
hover_log="$HOME/Downloads/hover_logs/hover_$(date +%Y%m%d_%H%M%S).log"
python3 ~/Downloads/first_hover_terminal.py --execute --hover-height 1.0 --hold-seconds 6.0 2>&1 | tee "$hover_log"
```

The process still waits for READY and `/first_hover/start`. Keep the complete localization chain running during flight.

## 2. Deferred: return-to-marker loop

Loop recording is deferred during the current planner integration stage. Repeated hover testing is complete; remaining hover work focuses on the approximately 10 cm lateral motion and controller/estimation diagnosis.

Use the [three-terminal bag workflow](BAG_XYZ_2026-10-09.md) for a disarmed handheld loop. Compare source/header gaps, registered-cloud artifacts, return XYZ/yaw error, and the final stationary windows. Preserve raw sensor messages for later diagnosis.

Store bags and full point clouds outside the source repository. Archive compact reports with route, configuration version, start/end pose, and environmental notes. The exporter's default 0.5-second gap threshold is an offline diagnostic setting, not a flight acceptance limit.

FAST_LIO does not apply loop-closure optimization. A stable final estimate and a small return error are useful observations, but neither establishes calibrated extrinsics or global map correction.

## 3. Measure mounting geometry and timing

The integration currently uses nominal +45° LiDAR yaw, a 10 cm vertical offset, and the configured internal IMU lever arm. Measure mounting rotation and translation, then compare known translations, yaw rotations, and small pitch/roll motions against the transformed pose.

Session quaternion pairing aligns world frames; it does not calibrate mechanical extrinsics. Investigate cloud spikes using raw timing, deskewing, mounting, and dynamic-scene evidence before changing parameters.

## 4. Validate takeover and loss response

Normal automatic landing is complete. Emergency and degraded-input responses remain a separate validation campaign: manual POSCTL takeover, early landing, localization loss, controller exit, and RC loss.

Begin with ground switch/state checks, then use planned tests with flight logs and an experienced pilot. Do not improvise airborne cable removal or computer/localization restarts. Wi-Fi/NoMachine loss and onboard control-process loss are different faults and must be evaluated separately.

## 5. Repeat cold-boot recovery

Verify automatic AX900 initialization, saved Wi-Fi connection, headless desktop, and the dedicated MID360 Ethernet link across multiple cold boots. The compact tmux manager has restored aircraft localization and passed a disarmed ground planning check; use the [session runbook](SESSION_STARTUP.md) for subsequent recovery. Full-stack ROS autostart and SSH access remain optional follow-up work.

Initialization must remain a grounded, disarmed operation. Process restoration must never automatically arm or start a flight.

## 6. Integrate planning and surveyed boundaries

| Requirement | Remaining work |
| --- | --- |
| Laboratory envelope | Implement −0.10–5 m forward, ±2 m lateral, and FC-centre height ≤2 m using measured axes and origin |
| Formal venue | Survey the 3×6×3 m space, takeoff position, vehicle envelope, and clearance margins |
| Planner-to-flight bridge | Consistent coordinates/lever arm, valid targets, endpoint hold, planner-stop behavior, limits, takeover, and landing |
| Target speed | Final candidate: 2 m/s with 1 m/s² translational acceleration; begin with 0.3 m/s and 0.5 m/s² previews, then staged flight validation |

The 225 mm wheelbase and 5-inch propellers do not by themselves define the collision envelope. The recorded SUPER YAML includes a 5 m/s planner limit and is a historical configuration, not the final demonstration profile.

`control_bridge_preview.py` and `ground_setpoint_transport.py` are ground tools; their arming/OFFBOARD latches must not be mistaken for a flight bridge. SUPER can relocate goals out of inflated occupied cells, so validate the accepted goal rather than only the clicked point.

Separate low-speed and demonstration candidates are generated from the active diagnostic YAML. The compact manager can start the low-speed ground preview after `planner-stop`, preserving localization. PX4 limits and flight geometry remain unchanged until the flight-capable bridge and current-state validation are complete.

The `low-speed-reserved` profile (internal 0.25 m/s / 0.45 m/s² targets) has passed a 10-second physical ground sample with unchanged strict 0.3 m/s / 0.5 m/s² acceptance. Observed advertised speed and acceleration maxima were 0.289609 m/s and 0.310397 m/s². Freshness, timestamp/frame/source checks and fresh connected/disarmed state also passed. This completes that sampled ground diagnostic; it does not validate actual tracking or continuous bounds.

A separate debug-only bounded reference preview now implements fixed takeoff-floor/forward coordinates, point bounds, polynomial-backed endpoint holding and moving-discontinuity rejection. Its unit, isolated ROS and tmux lifecycle checks pass. Next, validate that ground preview on the aircraft, including any initial-reference relocation by SUPER, then implement the flight-capable control interface and actual restart/terminal transition dynamics. A 0.424264 m reference restart remained present over 20.310 ms despite passing advertised-derivative checks. A flight bridge must reject or explicitly reconcile that discontinuity rather than forwarding the restart. Goal relocation, loss handling and RC takeover must be validated separately before SUPER trajectory execution. The working RViz2 ground view remains a visualization.

The first bounded-preview ground trial rejected an initial reference at forward −0.0552 m while lateral and height values were valid. The ground envelope now permits 10 cm behind the captured takeoff point, retaining original SUPER rest-start selection. Restore the optional exact-start patch if applied, then resample for TRACK → END_HOLD with the updated preview. Native flight output remains separate.
