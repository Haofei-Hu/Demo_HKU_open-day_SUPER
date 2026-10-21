# View raw MID360 scans

The observed `/livox/lidar` stream uses `livox_ros_driver2/msg/CustomMsg`.
`scripts/analysis/mid360_raw_viewer.py` subscribes with sensor-data QoS and
publishes `/mid360/raw_points` as `sensor_msgs/msg/PointCloud2` for RViz.
It preserves XYZ, reflectivity as float intensity, tag, line, point offset time,
header timestamp and frame. It neither filters points (including zero returns)
nor deskews, transforms, downsamples or registers them. The sensor's CustomMsg
timebase and lidar identifier are not additional fields in the display output.

Use `configs/rviz/mid360_raw_compare.rviz`: fixed frame `livox_frame`, with
`RawSingleFrame` (decay zero) or `RawAccumulated1s` (decay one second) enabled
one at a time. If the adapter prints a different header frame, use that frame
instead. Leave the aircraft physically stationary with motors stopped. Raw
accumulation on a moving sensor naturally smears because no motion compensation
is applied. A cluster at the sensor origin may contain invalid zero-distance
returns, which this adapter deliberately retains.

Keep an existing Livox driver running; the adapter never starts or reconfigures
it. If the managed stack was stopped for building, start only
`ros2 launch livox_ros_driver2 msg_MID360_launch.py` in a sourced terminal before
viewing. Do not start another driver alongside an already running one. FAST-LIO,
MAVROS, external vision and flight control are not required for this view.

The format conversion was checked for byte layout, point order, maximum field
values, zero-return preservation and empty input. The RViz configuration is a
topic/frame adaptation of the existing single-frame/accumulated comparison.
