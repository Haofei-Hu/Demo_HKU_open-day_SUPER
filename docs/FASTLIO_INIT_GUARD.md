# FAST-LIO IMU initialization guard

The uploaded onboard `IMU_Processing.hpp` matches upstream revision
`a4743b095409588842a5b30ddfa27e29d2f99164` byte-for-byte by the reviewed hash.
Its constructor does not initialize `last_lidar_end_time_` or `acc_s_last`,
which are read during the first undistortion. Reset also omits those members.
`Process()` returns immediately on empty IMU input without clearing its output;
the mapping caller subsequently checks only whether the cloud is empty, so a
previous scan can remain eligible for processing under the next scan timestamp.
These are source defects; the supplied runtime logs do not prove when either
path was responsible for a particular position jump.

The small [patch](../patches/fastlio_imu_init_guard.patch) initializes and resets
both members, anchors the integration boundary at completion of IMU
initialization, and clears output before early returns. It leaves extrinsics,
noise settings, downsampling and the estimator/deskew formulas unchanged.
It does not flush startup queues or comprehensively fix missing IMU coverage.
The [helper](../scripts/maintenance/apply_fastlio_init_guard.py) accepts only
the exact reviewed source or its exact patched output, refuses a running
`fastlio_mapping` process, backs up the header/diff/checksums, and replaces only
that header. By default it checks without writing; `--apply` is explicit.

## Observed evidence and limits

During stationary B startup, the body-pose helper reports Z=-0.464 m and then
Z=2.240 m about 2.018 seconds later in the same session. This is approximately
2.704 m of estimated motion, rather than just a difference between two session
origins. The helper's fixed lever-arm conversion cannot explain this magnitude.
First published odometry age was 2.892 seconds, with 59 queued lidar frames;
it was near 10 ms two seconds later. Other startups also show approximately
2.7–2.9 seconds of initial backlog, so backlog alone does not identify the cause.
The B/A steady-state comparison is therefore confounded by startup/map state;
it does not establish that surface size 0.2 intrinsically worsens localization.

The initial `No point, skip this scan` messages can occur while IMU initialization
returns no output; they are not by themselves proof that the LiDAR emitted empty
clouds. Livox exit code -7 in the two trial logs occurs after requested Ctrl+C,
not during the stationary measurement. That shutdown problem is separate.

The guard is not a validated solution to 10 Hz attitude oscillation or airborne
hover drift. Rebuild and compare stationary startup and steady-state data using
the original 0.5 surface/map sizes and fixed extrinsics before further tuning.
Keep vision/control stopped after restarting localization; old alignment is not
valid for the new session.

## Validation and deployment

The patch helper's exact-source rejection, process guard, backup, idempotency,
race refusal and temporary-file cleanup were checked on the uploaded header.
Actual constructor, Reset and Process bodies were extracted from the patched
header and compiled with AddressSanitizer/UndefinedBehaviorSanitizer and small
dependency stubs. Checks cover initial member values, empty-input stale-output
removal, initialization boundary, normal dispatch and reset. This is control-flow
validation, not a full ROS/PCL build or ESKF/deskew numerical test.

Reproduce with an original, unmodified copy of the reviewed header:

```bash
python3 tests/check_fastlio_init_guard_tool.py /path/to/original/IMU_Processing.hpp
python3 tests/check_fastlio_init_guard_cpp.py /path/to/original/IMU_Processing.hpp
```

Deployment requires a grounded managed stop, `--apply`, and a successful onboard
`colcon build --packages-select fast_lio` before restarting. Source edits alone
do not update the executable. The full onboard build and physical regression
test have not been performed in this environment.

To roll back, keep the stack stopped, copy the printed backup header over the
source header, and rebuild the same package. Retain the backup folder and build
output with the subsequent diagnostic reports.
