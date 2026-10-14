# Recorded data

Raw bags default to `~/Downloads/lio_bags/` on the onboard computer. XYZ exports use the sibling `<bag_name>_xyz/` directory. Large raw recordings and point clouds remain outside the source repository.

The [recording/export workflow](../docs/BAG_XYZ_2026-10-09.md) defines the topics, outputs, timing metrics, and return error. Archive compact reports with configuration versions and test descriptions as data becomes available.

The [October 13 bounded-reference ground result](observed_2026-10-13/bounded_reference_ground.json) records requested/accepted planner goals, initial axis offsets, held coordinates and restart-reference suppression. It is a compact stationary-ground report, not a flight log or measured tracking-accuracy result.

The [October 14 vision-chain audit](observed_2026-10-14/super_handover_vision_audit.json) records a sustained disarmed terminal-reference hold and concurrent raw/aligned/native-vision/FC-pose sampling. Earlier handover trials remain in [the original audit record](observed_2026-10-14/super_handover_audit.json). Neither report validates planner-driven aircraft flight or explains the intermittent freshness faults.
