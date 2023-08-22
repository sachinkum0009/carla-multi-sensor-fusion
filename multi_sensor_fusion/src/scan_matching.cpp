#include "multi_sensor_fusion/scan_matching.hpp"

namespace scan_matching
{
    
    ScanMatching::ScanMatching() : Node("scan_matching") {
        _scanSub = this->create_subscription<sensor_msgs::msg::LaserScan>("/carla/ego_vehicle/scan", 10, std::bind(&ScanMatching::scanCallback, this, std::placeholders::_1));

    }
    ScanMatching::~ScanMatching() {}

    void ScanMatching::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) const {
        RCLCPP_INFO(this->get_logger(), "I got scan message");
        RCLCPP_INFO(this->get_logger(), "laser size is: %d", scan_msg->ranges.size());
        

    }
} // namespace scan_matching



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<scan_matching::ScanMatching>());
    rclcpp::shutdown();
    return 0;
}
