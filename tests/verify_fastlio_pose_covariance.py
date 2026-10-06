#!/usr/bin/env python3
"""Isolated Eigen verification of the patched publish_odometry function.

Usage: python3 verify_fastlio_pose_covariance.py PATCHED_LASERMAPPING_CPP EIGEN_INCLUDE
This compiles the exact function from that source against message/publisher stubs.
It does not build FAST_LIO or modify a checkout, and requires g++ and Eigen.
"""
from pathlib import Path
import subprocess
import sys
import tempfile


PREAMBLE = r'''
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using Matrix6 = Eigen::Matrix<double, 6, 6>;
struct Vector3 { double x=0, y=0, z=0; };
struct Quaternion { double x=0, y=0, z=0, w=1; };
struct Pose { Vector3 position; Quaternion orientation; };
struct Header { std::string frame_id; double stamp=0; };
namespace nav_msgs { namespace msg {
struct Odometry {
  Header header;
  std::string child_frame_id;
  struct { Pose pose; std::array<double,36> covariance{}; } pose;
};
}}
namespace geometry_msgs { namespace msg {
struct TransformStamped {
  Header header;
  std::string child_frame_id;
  struct { Vector3 translation; Quaternion rotation; } transform;
};
}}
namespace rclcpp {
template <typename Message> struct Publisher {
  using SharedPtr = std::shared_ptr<Publisher<Message>>;
  std::vector<Message> published;
  void publish(const Message &message) { published.push_back(message); }
};
}
namespace tf2_ros {
struct TransformBroadcaster {
  std::vector<geometry_msgs::msg::TransformStamped> published;
  void sendTransform(const geometry_msgs::msg::TransformStamped &message) {
    published.push_back(message);
  }
};
}
struct Rotation {
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d toRotationMatrix() const { return matrix; }
};
struct State { Eigen::Vector3d pos; Rotation rot; } state_point;
struct Filter {
  Eigen::Matrix<double,23,23> covariance;
  auto get_P() const { return covariance; }
} kf;
nav_msgs::msg::Odometry odomAftMapped;
Quaternion geoQuat;
double lidar_end_time = 0;
double get_ros_time(double value) { return value; }
'''


CHECKS = r'''
Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d out;
  out << 0,-v.z(),v.y(), v.z(),0,-v.x(), -v.y(),v.x(),0;
  return out;
}

void verify_snapshot(const nav_msgs::msg::Odometry &message,
                     const Matrix6 &expected) {
  for (int i=0; i<6; ++i) {
    for (int j=0; j<6; ++j) {
      assert(std::abs(message.pose.covariance[i*6+j]-expected(i,j)) < 1e-12);
    }
  }
  assert(message.header.frame_id == "camera_init");
  assert(message.child_frame_id == "body");
  assert(message.header.stamp == lidar_end_time);
  assert(message.pose.pose.position.x == state_point.pos.x());
  assert(message.pose.pose.position.y == state_point.pos.y());
  assert(message.pose.pose.position.z == state_point.pos.z());
  assert(message.pose.pose.orientation.x == geoQuat.x);
  assert(message.pose.pose.orientation.y == geoQuat.y);
  assert(message.pose.pose.orientation.z == geoQuat.z);
  assert(message.pose.pose.orientation.w == geoQuat.w);
}

int main() {
  auto publisher = std::make_shared<rclcpp::Publisher<nav_msgs::msg::Odometry>>();
  auto broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
  kf.covariance.setIdentity();
  Matrix6 input;
  input <<
    1,0,0, .01,.02,.03,
    0,4,0, -.03,.05,.04,
    0,0,9, .06,-.02,.08,
    .01,-.03,.06, .01,0,0,
    .02,.05,-.02, 0,.04,0,
    .03,.04,.08, 0,0,.09;
  assert(Eigen::SelfAdjointEigenSolver<Matrix6>(input).eigenvalues().minCoeff()>0);
  kf.covariance.block<6,6>(0,0) = input;
  state_point.pos << 1,2,3;
  state_point.rot.matrix << 0,-1,0, 1,0,0, 0,0,1;
  geoQuat = Quaternion{0,0,std::sqrt(.5),std::sqrt(.5)};
  lidar_end_time = 123.4;

  // Explicit independent expected values for a 90-degree world yaw:
  // position variances stay 1/4/9, local angular x/y become world y/-x,
  // and every position-angular correlation follows that axis mapping.
  Matrix6 expected;
  expected <<
    1,0,0, -.02,.01,.03,
    0,4,0, -.05,-.03,.04,
    0,0,9, .02,.06,.08,
    -.02,-.05,.02, .04,0,0,
    .01,-.03,.06, 0,.01,0,
    .03,.04,.08, 0,0,.09;
  publish_odometry(publisher, broadcaster);
  assert(publisher->published.size()==1);
  verify_snapshot(publisher->published.back(), expected);

  // Different covariance on the next frame detects delayed assignment:
  // the second snapshot must contain the second frame's variances/cross terms.
  kf.covariance.block<6,6>(0,0) = 2 * input;
  state_point.pos << -1,4,5;
  lidar_end_time = 123.5;
  publish_odometry(publisher, broadcaster);
  assert(publisher->published.size()==2);
  verify_snapshot(publisher->published.back(), 2 * expected);
  assert(broadcaster->published.size()==2);
  const auto &transform = broadcaster->published.back();
  assert(transform.header.frame_id=="camera_init" && transform.child_frame_id=="body");
  assert(transform.header.stamp==lidar_end_time);
  assert(transform.transform.translation.x==state_point.pos.x());
  assert(transform.transform.translation.y==state_point.pos.y());
  assert(transform.transform.translation.z==state_point.pos.z());
  assert(transform.transform.rotation.w==geoQuat.w);

  // Independently derive the right-error -> fixed-world angular Jacobian
  // using a tilted rotation and exact SO(3) composition, not covariance code.
  const Eigen::Matrix3d R =
    Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,-3).normalized()).toRotationMatrix();
  const double epsilon = 1e-7;
  for (int axis=0; axis<3; ++axis) {
    const Eigen::Matrix3d perturbed = R *
      Eigen::AngleAxisd(epsilon,Eigen::Vector3d::Unit(axis)).toRotationMatrix();
    const Eigen::AngleAxisd fixed_error(perturbed * R.transpose());
    const Eigen::Vector3d numerical = fixed_error.axis() * fixed_error.angle()/epsilon;
    assert((numerical-R.col(axis)).norm()<1e-8);
  }
  std::cout << "PASS: exact patched publisher compiles; correct block order, "
               "world-axis angular/cross covariance, same-frame first/second "
               "snapshots, preserved pose/time/TF; SO(3) numerical Jacobian.\n";
}
'''


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    source = Path(sys.argv[1]).read_text()
    begin = source.index('template<typename T>\nvoid set_posestamp')
    end = source.index('\nvoid publish_path(', begin)
    functions = source[begin:end]
    with tempfile.TemporaryDirectory(prefix='fastlio-covariance-') as folder:
        cpp = Path(folder)/'verify.cpp'
        executable = Path(folder)/'verify'
        cpp.write_text(PREAMBLE + functions + CHECKS)
        subprocess.run([
            'g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-O2',
            '-isystem', sys.argv[2], str(cpp), '-o', str(executable)
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    main()
