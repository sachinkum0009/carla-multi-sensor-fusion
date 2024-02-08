#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>

namespace tf2_broadcaster
{
    class Tf2Broadcaster : public rclcpp::Node {
public:
    Tf2Broadcaster();
    ~Tf2Broadcaster();
private:
    void _timerCallback();
    std::unique_ptr<tf2_ros::TransformBroadcaster> _tfBroadcaster;
    geometry_msgs::msg::TransformStamped _transformStamped;
    rclcpp::TimerBase::SharedPtr _timerBase;
    };
} // namespace tf2_broadcaster
