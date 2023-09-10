#include <multi_sensor_fusion/gps_odom.hpp>

namespace gps_odom
{
    GpsOdom::GpsOdom() : Node("gps_odom") {
        _gnssSub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/carla/ego_vehicle/gnss", 10, std::bind(&GpsOdom::gnssCallback, this, std::placeholders::_1));
    }
    GpsOdom::~GpsOdom() {

    }
    void GpsOdom::gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Gnns Msg");

    }

} // namespace gps_odom


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps_odom::GpsOdom>());
    rclcpp::shutdown();
    return 0;
}
