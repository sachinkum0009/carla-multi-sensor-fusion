#include <multi_sensor_fusion/gps_odom.hpp>

namespace gps_odom
{
    GpsOdom::GpsOdom() : Node("gps_odom"), _odomInitialized(false) {
        _gnssSub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/carla/ego_vehicle/gnss", 10, std::bind(&GpsOdom::gnssCallback, this, std::placeholders::_1));
        _gpsOdomPub = this->create_publisher<nav_msgs::msg::Odometry>("/gps_odom", 10);
        _odomMsg.header.frame_id = "odom";
        _odomMsg.child_frame_id = "base_link";
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GpsOdom::publishOdom, this));
        std::srand(std::time(nullptr));
    }
    GpsOdom::~GpsOdom() {

    }
    void GpsOdom::gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        convertLatLng2XYCoord(msg->latitude, msg->longitude, _odomMsg.pose.pose.position.x, _odomMsg.pose.pose.position.y);
        if (!_odomInitialized) {
            _initial_x = _odomMsg.pose.pose.position.x;
            _initial_y = _odomMsg.pose.pose.position.y;
            _odomInitialized = true;
        }
        RCLCPP_INFO(this->get_logger(), "Converted coordinates are x: %lf, y: %lf", _odomMsg.pose.pose.position.x, _odomMsg.pose.pose.position.y);
    }

    void GpsOdom::convertLatLng2XYCoord(const double lat, const double lng, double& x, double& y) {
        // Convert latitude and longitude to meters using Mercator projection
        x = (lng * METERS_PER_DEGREE) / 10.0f;
        y = (log(tan((90.0 + lat) * M_PI / 360.0)) * EARTH_RADIUS) / 10.0f;
        if (_odomInitialized) {
            x-=_initial_x;
            y-=_initial_y;
        }
        double noise_x = (std::rand() / (double)RAND_MAX - 0.5) * MAX_NOISE;
        double noise_y = (std::rand() / (double)RAND_MAX - 0.5) * MAX_NOISE;

        x += noise_x;
        y += noise_y;
    }

    void GpsOdom::publishOdom() {
        if (_odomInitialized) {
            _odomMsg.header.stamp = this->get_clock()->now();
            _gpsOdomPub->publish(_odomMsg);
        }
    }


} // namespace gps_odom


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps_odom::GpsOdom>());
    rclcpp::shutdown();
    return 0;
}
