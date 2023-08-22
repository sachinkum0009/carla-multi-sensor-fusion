#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


namespace scan_matching
{
    class ScanMatching : public rclcpp::Node {
public:
    ScanMatching();
    ~ScanMatching();
    
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scanSub;

    sensor_msgs::msg::LaserScan _scanMsg;
    uint32_t ranges_size;
    double inf_eplison_;
    

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) const;
    
    };
} // namespace scan_matching
