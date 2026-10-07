#!/usr/bin/env python3
"""Apply the October 7 source change with a backup and exact-match checks."""
from pathlib import Path
import shutil
import sys
import tempfile

def main():
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        Path.home() / 'fastlio_ws/src/FAST_LIO/src/laserMapping.cpp')
    original = source.read_text()
    marker = '// FASTLIO_TIMING_PROBE_V1'
    if marker in original:
        print('Timing probe already present; no changes made.')
        return
    if 'covariance_axes' not in original or 'pose_covariance' not in original:
        raise SystemExit('Existing covariance repair not found; no changes made.')

    globals_code = '''
// FASTLIO_TIMING_PROBE_V1
double fastlio_probe_lrx = -1.0, fastlio_probe_irx = -1.0;
double fastlio_probe_pre_ms = -1.0, fastlio_probe_proc_ms = -1.0;
double fastlio_probe_now()
{
    static rclcpp::Clock clock(RCL_SYSTEM_TIME);
    return clock.now().seconds();
}
'''
    pub_code = '''    static double fastlio_probe_last_log = 0.0;
    const double probe_steady = omp_get_wtime();
    if (probe_steady - fastlio_probe_last_log >= 1.0)
    {
        fastlio_probe_last_log = probe_steady;
        const double now = fastlio_probe_now();
        RCLCPP_INFO(rclcpp::get_logger("fastlio_timing"),
            "pub_age=%.3f lrx=%.3f irx=%.3f latest_lidar=%.3f latest_imu=%.3f "
            "lidar_q=%zu imu_q=%zu oldest_lidar_age=%.3f pre_ms=%.1f proc_ms=%.1f scan_dt=%.3f",
            now - lidar_end_time, fastlio_probe_lrx, fastlio_probe_irx,
            now - last_timestamp_lidar, now - last_timestamp_imu,
            lidar_buffer.size(), imu_buffer.size(),
            time_buffer.empty() ? -1.0 : now - time_buffer.front(),
            fastlio_probe_pre_ms, fastlio_probe_proc_ms,
            lidar_end_time - Measures.lidar_beg_time);
    }
'''
    changes = [
        ('double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;',
         'double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;' + globals_code),
        ('    mtx_buffer.lock();\n    double cur_time = get_time_sec(msg->header.stamp);',
         '    fastlio_probe_lrx = fastlio_probe_now() - get_time_sec(msg->header.stamp);\n'
         '    mtx_buffer.lock();\n    double cur_time = get_time_sec(msg->header.stamp);'),
        ('void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)\n{',
         'void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)\n{\n'
         '    fastlio_probe_irx = fastlio_probe_now() - get_time_sec(msg_in->header.stamp);'),
        ('    time_buffer.push_back(last_timestamp_lidar);',
         '    time_buffer.push_back(last_timestamp_lidar);\n'
         '    fastlio_probe_pre_ms = 1000.0 * (omp_get_wtime() - preprocess_start_time);'),
        ('            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);',
         '            fastlio_probe_proc_ms = 1000.0 * (omp_get_wtime() - t0);\n'
         '            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);'),
        ('    pubOdomAftMapped->publish(odomAftMapped);',
         pub_code + '    pubOdomAftMapped->publish(odomAftMapped);'),
    ]
    updated = original
    for old, new in changes:
        if updated.count(old) != 1:
            raise SystemExit(f'Source mismatch: {old[:85]!r}; no changes made.')
        updated = updated.replace(old, new, 1)
    backup_dir = Path(tempfile.mkdtemp(
        prefix='fastlio_timing_backup_', dir=str(Path.home())))
    backup = backup_dir / source.name
    shutil.copy2(source, backup)
    source.write_text(updated)
    print('Timing probe applied.')
    print('Backup:', backup)


if __name__ == "__main__":
    main()
