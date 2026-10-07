#!/usr/bin/env python3
"""Apply the October 7 source change with a backup and exact-match checks."""
from pathlib import Path
import shutil
import sys
import tempfile

def main():
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        Path.home() / 'fastlio_ws/src/FAST_LIO/src/laserMapping.cpp')
    old = 'sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);'
    new = ('sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>('
           'imu_topic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), imu_cbk);')
    text = source.read_text()
    if new in text:
        print('IMU Best Effort subscription already present; no changes made.')
        return
    if text.count(old) != 1:
        raise SystemExit('IMU subscription source mismatch; no changes made.')
    backup_dir = Path(tempfile.mkdtemp(
        prefix='fastlio_imu_qos_backup_', dir=str(Path.home())))
    backup = backup_dir / source.name
    shutil.copy2(source, backup)
    source.write_text(text.replace(old, new, 1))
    print('IMU subscription changed to Best Effort; depth remains 10.')
    print('Backup:', backup)


if __name__ == "__main__":
    main()
