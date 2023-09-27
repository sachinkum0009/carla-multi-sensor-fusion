#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <iostream>
#include <cstdint>
#include <cstring>

namespace radar_stop
{
    class RadarStop : public rclcpp::Node {
public:
    RadarStop();
    ~RadarStop();
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _radarPointcloudSub;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr _egoControlPub;

    carla_msgs::msg::CarlaEgoVehicleControl _egoControlMsg;
    float safetyDistance, range;
    size_t pointStep, rangeOffset;

    bool initializeValues;

    void radarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void emergencyBrake();
    };
    
} // namespace radar_stop
