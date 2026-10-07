#include <rclcpp/rclcpp.hpp>
#include <omp.h>
#include <deque>
#include <thread>
#include <chrono>
double last_timestamp_lidar, last_timestamp_imu, lidar_end_time;
std::deque<double> lidar_buffer, imu_buffer, time_buffer;
struct { double lidar_beg_time; } Measures;

// FASTLIO_TIMING_PROBE_V1
double fastlio_probe_lrx = -1.0, fastlio_probe_irx = -1.0;
double fastlio_probe_pre_ms = -1.0, fastlio_probe_proc_ms = -1.0;
double fastlio_probe_now()
{
    static rclcpp::Clock clock(RCL_SYSTEM_TIME);
    return clock.now().seconds();
}

void log_probe() {
    static double fastlio_probe_last_log = 0.0;
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
}

int main(int argc, char** argv) {
 rclcpp::init(argc,argv);
 double now=fastlio_probe_now();
 last_timestamp_lidar=now-.11; last_timestamp_imu=now-.001;
 lidar_end_time=now-1.4; Measures.lidar_beg_time=lidar_end_time-.1;
 lidar_buffer.resize(12); imu_buffer.resize(280); time_buffer.push_back(now-1.3);
 fastlio_probe_lrx=.11; fastlio_probe_irx=.001;
 fastlio_probe_pre_ms=5; fastlio_probe_proc_ms=15;
 log_probe(); log_probe();
 std::this_thread::sleep_for(std::chrono::milliseconds(1100));
 log_probe(); rclcpp::shutdown();
}
