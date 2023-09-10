#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace gps_odom
{
    class GpsOdom : public rclcpp::Node {
public:
    GpsOdom();
    ~GpsOdom();
private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _gnssSub;

    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    };
} // namespace gps_odom
