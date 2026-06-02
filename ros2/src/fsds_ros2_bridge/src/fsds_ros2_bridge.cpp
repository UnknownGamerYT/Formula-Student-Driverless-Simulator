#include "rclcpp/rclcpp.hpp"
#include "airsim_ros_wrapper.h"
#include <algorithm>
// #include <ros/spinner.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fsds_ros2_bridge"); 

    std::string host_ip = node->declare_parameter<std::string>("host_ip", "localhost");
    int api_port = node->declare_parameter<int>("api_port", RpcLibPort);
    double timeout_sec = node->declare_parameter<double>("timeout", 10.0);
    api_port = std::max(1, std::min(65535, api_port));

    AirsimROSWrapper airsim_ros_wrapper(node, host_ip, static_cast<uint16_t>(api_port), timeout_sec);

    // if (airsim_ros_wrapper.is_used_lidar_timer_cb_queue_)
    // {
    //     airsim_ros_wrapper.lidar_async_spinner_.start();
    // }

    rclcpp::spin(node);

    return 0;
} 
