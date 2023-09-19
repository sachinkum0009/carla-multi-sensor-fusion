#include <multi_sensor_fusion/gps_odom.hpp>

namespace gps_odom
{
    GpsOdom::GpsOdom() : Node("gps_odom") {
        _gnssSub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/carla/ego_vehicle/gnss", 10, std::bind(&GpsOdom::gnssCallback, this, std::placeholders::_1));
        _gpsOdomPub = this->create_publisher<nav_msgs::msg::Odometry>("/gps_odom", 10);
        _odomMsg.header.frame_id = "odom";
        _odomMsg.child_frame_id = "base_link";
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GpsOdom::publishOdom, this));
    }
    GpsOdom::~GpsOdom() {

    }
    void GpsOdom::gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Gnns Msg");
        convertLatLng2XYCoord(msg->latitude, msg->longitude, _odomMsg.pose.pose.position.x, _odomMsg.pose.pose.position.y);
    }

    void GpsOdom::convertLatLng2XYCoord(const double lat, const double lng, double& x, double& y) {
        // Convert latitude and longitude to meters using Mercator projection
        x = lng * METERS_PER_DEGREE;
        y = log(tan((90.0 + lat) * M_PI / 360.0)) * EARTH_RADIUS;
    }

    void GpsOdom::publishOdom() {
        _odomMsg.header.stamp = this->get_clock()->now();
        _gpsOdomPub->publish(_odomMsg);
    }


} // namespace gps_odom


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps_odom::GpsOdom>());
    rclcpp::shutdown();
    return 0;
}
