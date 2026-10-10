#!/usr/bin/env python3
"""Apply only the reviewed covariance fix, retaining an external source backup."""
from pathlib import Path
import shutil
import sys
import tempfile

old = b'''    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }
'''
new = b'''    // Position first; convert the IMU-local rotation error to world axes.
    const auto P = kf.get_P();
    Eigen::Matrix<double, 6, 6> covariance_axes =
        Eigen::Matrix<double, 6, 6>::Identity();
    covariance_axes.block<3, 3>(3, 3) = state_point.rot.toRotationMatrix();
    const Eigen::Matrix<double, 6, 6> pose_covariance =
        covariance_axes * P.block<6, 6>(0, 0) * covariance_axes.transpose();
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            odomAftMapped.pose.covariance[i * 6 + j] = pose_covariance(i, j);
        }
    }
    pubOdomAftMapped->publish(odomAftMapped);
'''


def fix(source, backup_root):
    original = source.read_bytes()
    # The timing probe inserts a log between covariance assignment and publish.
    if new in original or new.rsplit(b'    pubOdomAftMapped->publish(odomAftMapped);', 1)[0] in original:
        print('Covariance fix already present; no changes made.')
        return
    if original.count(old) != 1:
        raise SystemExit('Expected source block not found exactly once; no changes made.')
    backup_dir = Path(tempfile.mkdtemp(prefix='fastlio_cov_backup_', dir=str(backup_root)))
    backup = backup_dir / source.name
    shutil.copy2(source, backup)
    source.write_bytes(original.replace(old, new, 1))
    print('Source covariance fix applied.')
    print('Backup:', backup)


if __name__ == '__main__':
    source = Path(sys.argv[1]) if len(sys.argv) == 2 else (
        Path.home() / 'fastlio_ws/src/FAST_LIO/src/laserMapping.cpp')
    fix(source, Path.home())
