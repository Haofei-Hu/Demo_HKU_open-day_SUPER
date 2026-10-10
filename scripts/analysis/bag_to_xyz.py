#!/usr/bin/env python3
"""Export a ROS2 bag to unfiltered world-frame XYZ and return-to-marker metrics.

Run after sourcing ROS2 Humble and the Livox workspace. No ROS publishers,
flight commands, ICP alignment, voxel filtering, or outlier removal are used.
"""
import argparse
import csv
import json
import math
from collections import deque
from pathlib import Path

import numpy as np


def cloud_xyz(msg):
    """Respect PointCloud2 byte order, field offsets, point and row padding."""
    if not msg.width or not msg.height:
        return np.empty((0, 3))
    formats = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    endian = ">" if msg.is_bigendian else "<"
    fields = {f.name: f for f in msg.fields}
    names, dtypes, offsets = [], [], []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in formats:
            raise ValueError(f"Invalid or missing scalar {name} field")
        dtype = np.dtype(endian + formats[field.datatype])
        if field.offset < 0 or field.offset + dtype.itemsize > msg.point_step:
            raise ValueError(f"{name} field exceeds point_step")
        names.append(name)
        dtypes.append(dtype)
        offsets.append(field.offset)
    if msg.row_step < msg.width * msg.point_step:
        raise ValueError("row_step is shorter than one row of points")
    required = (msg.height - 1) * msg.row_step + msg.width * msg.point_step
    if len(msg.data) < required:
        raise ValueError("PointCloud2 data is truncated")
    dtype = np.dtype({"names": names, "formats": dtypes, "offsets": offsets,
                      "itemsize": msg.point_step})
    view = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                      strides=(msg.row_step, msg.point_step))
    points = np.column_stack([view[name].ravel() for name in names]).astype(float)
    return points[np.isfinite(points).all(axis=1)]


class Timing:
    def __init__(self, threshold):
        self.threshold = threshold
        self.n = self.header_n = self.bad_stamps = 0
        self.receive_gaps = self.stamp_gaps = 0
        self.max_receive_gap = self.max_stamp_gap = 0.0
        self.first = self.last = self.previous_stamp = None
        self.frames = set()
        self.delay_sum = self.delay_max = self.delay_min = None

    def add(self, received, stamp, frame):
        self.n += 1
        if self.last is not None:
            gap = received - self.last
            self.max_receive_gap = max(self.max_receive_gap, gap)
            self.receive_gaps += gap > self.threshold
        else:
            self.first = received
        self.last = received
        if stamp is None:
            return
        self.frames.add(frame)
        self.header_n += 1
        if self.previous_stamp is not None:
            gap = stamp - self.previous_stamp
            self.bad_stamps += gap <= 0
            self.max_stamp_gap = max(self.max_stamp_gap, gap)
            self.stamp_gaps += gap > self.threshold
        self.previous_stamp = stamp
        delay = received - stamp
        self.delay_sum = (self.delay_sum or 0) + delay
        self.delay_min = delay if self.delay_min is None else min(self.delay_min, delay)
        self.delay_max = delay if self.delay_max is None else max(self.delay_max, delay)

    def report(self):
        duration = self.last - self.first if self.n > 1 else 0
        return dict(messages=self.n, observed_duration_s=duration,
                    average_hz=(self.n - 1) / duration if duration > 0 else None,
                    max_record_gap_s=self.max_receive_gap,
                    record_gaps_over_threshold=self.receive_gaps,
                    max_header_gap_s=self.max_stamp_gap,
                    header_gaps_over_threshold=self.stamp_gaps,
                    non_increasing_header_stamps=self.bad_stamps,
                    header_messages=self.header_n, frames=sorted(self.frames),
                    record_minus_header_s=dict(min=self.delay_min, max=self.delay_max,
                        mean=self.delay_sum / self.header_n if self.header_n else None))


def stamp_and_frame(msg):
    if not hasattr(msg, "header"):
        return None, ""
    header = msg.header
    nanoseconds = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
    return (nanoseconds / 1e9 if nanoseconds > 0 else None), header.frame_id


def mean_yaw(quaternions):
    quaternions = np.asarray(quaternions)
    norms = np.linalg.norm(quaternions, axis=1)
    valid = np.isfinite(quaternions).all(axis=1) & (norms > 1e-6)
    if not valid.any():
        return None
    q = quaternions[valid] / norms[valid, None]
    _, vectors = np.linalg.eigh(q.T @ q)
    x, y, z, w = vectors[:, -1]
    return math.degrees(math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))


def trajectory_report(rows, window, frames):
    # Columns: record time, header time, xyz, quaternion xyzw.
    data = np.asarray(rows, dtype=float)
    if not len(rows):
        return {"available": False}
    positions = data[:, 2:5]
    elapsed = data[-1, 0] - data[0, 0]
    report = dict(available=True, samples=len(rows), duration_s=float(elapsed),
                  frames=sorted(frames))
    tail_windows = []
    for offset in (3, 2, 1):
        lower = data[-1, 0] - offset * window
        upper = lower + window
        section = data[(data[:, 0] > lower) & (data[:, 0] <= upper)]
        if len(section):
            center = section[:, 2:5].mean(axis=0)
            tail_windows.append(dict(start_relative_s=float(lower-data[0, 0]),
                end_relative_s=float(upper-data[0, 0]), samples=len(section),
                mean_xyz_m=center.tolist(), std_xyz_m=section[:, 2:5].std(axis=0).tolist(),
                max_radius_m=float(np.linalg.norm(section[:, 2:5]-center, axis=1).max())))
    report["final_three_windows"] = tail_windows
    if len(rows) > 1:
        dt = np.diff(data[:, 1])
        distance = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        speeds = distance[dt > 0] / dt[dt > 0]
        report["max_consecutive_step_m"] = float(distance.max())
        report["max_consecutive_speed_m_s"] = float(speeds.max()) if len(speeds) else None
    if elapsed < 2 * window or len(frames) != 1:
        report["closure_available"] = False
        report["reason"] = "Recording too short for separate windows, or trajectory frame changed"
        return report
    start = data[data[:, 0] <= data[0, 0] + window]
    end = data[data[:, 0] >= data[-1, 0] - window]
    a, b = start[:, 2:5].mean(axis=0), end[:, 2:5].mean(axis=0)
    delta = b - a
    start_yaw, end_yaw = mean_yaw(start[:, 5:9]), mean_yaw(end[:, 5:9])
    yaw_delta = ((end_yaw - start_yaw + 180) % 360 - 180
                 if start_yaw is not None and end_yaw is not None else None)
    report.update(closure_available=True, window_s=window,
        start_mean_xyz_m=a.tolist(), end_mean_xyz_m=b.tolist(),
        return_delta_xyz_m=delta.tolist(), return_error_3d_m=float(np.linalg.norm(delta)),
        return_error_xy_m=float(np.linalg.norm(delta[:2])), return_yaw_delta_deg=yaw_delta,
        start_window_samples=len(start), end_window_samples=len(end),
        start_window_max_radius_m=float(np.linalg.norm(start[:, 2:5] - a, axis=1).max()),
        end_window_max_radius_m=float(np.linalg.norm(end[:, 2:5] - b, axis=1).max()),
        interpretation="Physical return error only if first/last windows are stationary at the SAME marked pose. No loop-closure correction or ICP applied.")
    return report


def export(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = args.bag.expanduser().resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise ValueError("Pass the closed ROS2 bag DIRECTORY containing metadata.yaml")
    out = (args.output or bag.with_name(bag.name + "_xyz")).expanduser().resolve()
    if out == bag or bag in out.parents:
        raise ValueError("Output must be outside the input bag")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=args.storage),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if types.get(args.cloud_topic) != "sensor_msgs/msg/PointCloud2":
        raise ValueError(f"{args.cloud_topic} is absent or is not PointCloud2. Record /cloud_registered with scan_publish_en=true.")
    if types.get(args.odom_topic) != "nav_msgs/msg/Odometry":
        raise ValueError(f"{args.odom_topic} is absent or is not Odometry")
    classes, unavailable, timing = {}, {}, {}
    for topic, typ in types.items():
        timing[topic] = Timing(args.gap)
        try:
            classes[topic] = get_message(typ)
        except (ImportError, AttributeError, ValueError) as exc:
            unavailable[topic] = str(exc)
    for topic in (args.cloud_topic, args.odom_topic):
        if topic not in classes:
            raise ValueError(f"Cannot decode required topic {topic}: {unavailable[topic]}. Source ROS2 Humble before exporting.")
    out.mkdir(parents=True, exist_ok=False)
    rows, pose_frames, end_clouds = [], set(), deque()
    cloud_frame = cloud_first_time = cloud_last_time = None
    cloud_count = total_points = nonfinite_poses = 0
    last_cloud = None
    bag_first = bag_last = None
    with (out / "cloud_all.xyz").open("w") as merged, \
         (out / "cloud_start.xyz").open("w") as start_cloud, \
         (out / "cloud_frames.csv").open("w", newline="") as index_file:
        index = csv.writer(index_file)
        index.writerow(["record_time_s", "header_time_s", "frame_id", "first_row_zero_based", "finite_points"])
        while reader.has_next():
            topic, raw, nanoseconds = reader.read_next()
            received = nanoseconds / 1e9
            bag_first = received if bag_first is None else min(bag_first, received)
            bag_last = received if bag_last is None else max(bag_last, received)
            msg = deserialize_message(raw, classes[topic]) if topic in classes else None
            stamp, frame = stamp_and_frame(msg)
            timing[topic].add(received, stamp, frame)
            if topic == args.odom_topic:
                pose = msg.pose.pose
                p, q = pose.position, pose.orientation
                row = [received, stamp if stamp is not None else float("nan"),
                       p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if not np.isfinite(row).all():
                    nonfinite_poses += 1
                    continue
                rows.append(row)
                pose_frames.add(frame)
            if topic != args.cloud_topic:
                continue
            if cloud_frame is not None and cloud_frame != frame:
                raise ValueError("Point cloud frame changed; refusing to mix coordinate frames")
            cloud_frame = frame
            points = cloud_xyz(msg)
            if not len(points):
                continue
            if cloud_first_time is None:
                cloud_first_time = received
                np.savetxt(out / "cloud_first_frame.xyz", points, fmt="%.6f")
            cloud_last_time = received
            np.savetxt(merged, points, fmt="%.6f")
            index.writerow([received, stamp, frame, total_points, len(points)])
            if received <= cloud_first_time + args.window:
                np.savetxt(start_cloud, points, fmt="%.6f")
            end_clouds.append((received, points))
            while end_clouds and end_clouds[0][0] < received - args.window:
                end_clouds.popleft()
            last_cloud = points
            cloud_count += 1
            total_points += len(points)
            if cloud_count % 100 == 0:
                print(f"已导出 {cloud_count} 帧，{total_points:,} 个点", flush=True)
    with (out / "cloud_end.xyz").open("w") as end_file:
        for _, points in end_clouds:
            np.savetxt(end_file, points, fmt="%.6f")
    if last_cloud is not None:
        np.savetxt(out / "cloud_last_frame.xyz", last_cloud, fmt="%.6f")
    with (out / "trajectory.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["record_time_s", "header_time_s", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(rows)
    if rows:
        np.savetxt(out / "trajectory.xyz", np.asarray(rows)[:, 2:5], fmt="%.6f")
    trajectory = trajectory_report(rows, args.window, pose_frames)
    topic_reports = {}
    for topic, stats in timing.items():
        item = stats.report()
        item["leading_no_message_s"] = max(0.0, stats.first - bag_first) if stats.n else None
        item["trailing_no_message_s"] = max(0.0, bag_last - stats.last) if stats.n else None
        topic_reports[topic] = item
    report = dict(bag=str(bag), cloud_topic=args.cloud_topic, odom_topic=args.odom_topic,
                  cloud_frame=cloud_frame, cloud_frames=cloud_count, points=total_points,
                  cloud_first_record_s=cloud_first_time, cloud_last_record_s=cloud_last_time,
                  omitted_nonfinite_poses=nonfinite_poses, gap_threshold_s=args.gap,
                  trajectory=trajectory, topics=topic_reports,
                  header_decode_unavailable=unavailable,
                  limitations=["XYZ strips timestamps; use CSV/JSON for timing.",
                    "Repeated points are retained; merged cloud includes repeated scans, not an optimized map.",
                    "XYZ contains no colors, filtering, alignment, extrinsic calibration, or loop correction.",
                    "Bag gaps can reflect recorder overload or missing messages, not uniquely a sensor fault.",
                    "Record minus header time is meaningful only with compatible clocks.",
                    "Small return error alone does not establish correct extrinsics or flight readiness."])
    (out / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"\n输出目录：{out}")
    print(f"点云：{cloud_count} 帧，{total_points:,} 个有限点；未做降采样或去毛刺")
    if trajectory.get("closure_available"):
        delta = trajectory["return_delta_xyz_m"]
        print("回到门口的首尾均值差 XYZ：", [round(v, 4) for v in delta], "m")
        print(f"三维误差：{trajectory['return_error_3d_m']:.4f} m；仅在同一标记姿态静止时有效")
        print("首/尾静止窗口最大波动半径：",
              round(trajectory["start_window_max_radius_m"], 4),
              round(trajectory["end_window_max_radius_m"], 4), "m")
    else:
        print("首尾误差不可计算；详见 report.json")
    for topic in ("/livox/lidar", "/livox/imu", args.odom_topic, args.cloud_topic,
                  "/mavros/vision_pose/pose_cov"):
        if topic in timing:
            s = timing[topic]
            print(f"{topic}: n={s.n}, 最大录包间隔={s.max_receive_gap:.3f}s, "
                  f"最大消息时间戳间隔={s.max_stamp_gap:.3f}s, "
                  f">{args.gap:g}s录包间隔次数={s.receive_gaps}, "
                  f"末尾静默={topic_reports[topic]['trailing_no_message_s']}s")
    if not cloud_count or not rows:
        raise ValueError("No usable cloud or odometry messages; inspect the recording")
    if unavailable:
        print("部分消息类型无法解析 header；其录包间隔仍已统计。请 source 对应工作空间。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("-o", "--output", type=Path)
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--odom-topic", default="/Odometry")
    parser.add_argument("--storage", default="sqlite3")
    parser.add_argument("--window", type=float, default=5.0)
    parser.add_argument("--gap", type=float, default=0.5)
    args = parser.parse_args()
    if not math.isfinite(args.window) or args.window <= 0 or not math.isfinite(args.gap) or args.gap <= 0:
        parser.error("window and gap must be positive and finite")
    export(args)


if __name__ == "__main__":
    main()
