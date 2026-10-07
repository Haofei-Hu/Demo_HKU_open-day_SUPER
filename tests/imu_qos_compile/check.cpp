#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <thread>
#include <iostream>
#include <algorithm>
int main(int argc,char** argv) {
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("imu_qos_check");
 int count=0; double max_age=0;
 auto sub=node->create_subscription<sensor_msgs::msg::Imu>(
  "imu_probe",rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
  [&](sensor_msgs::msg::Imu::UniquePtr msg) {
   ++count; max_age=std::max(max_age,(node->now()-rclcpp::Time(msg->header.stamp)).seconds());
  });
 auto actual=sub->get_actual_qos();
 if(actual.depth()!=10 || actual.reliability()!=rclcpp::ReliabilityPolicy::BestEffort) return 2;
 auto pub=node->create_publisher<sensor_msgs::msg::Imu>("imu_probe",10);
 auto timer=node->create_wall_timer(std::chrono::milliseconds(5),[&] {
  sensor_msgs::msg::Imu msg; msg.header.stamp=node->now(); pub->publish(msg);
 });
 rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
 auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
 while(std::chrono::steady_clock::now()<end) {
  executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
 }
 std::cout<<"samples="<<count<<" max_age="<<max_age<<" actual=BEST_EFFORT depth=10"<<std::endl;
 rclcpp::shutdown();
 return count>=100 && max_age<.1 ? 0:3;
}
