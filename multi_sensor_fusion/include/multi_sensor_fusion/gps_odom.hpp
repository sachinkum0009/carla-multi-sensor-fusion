#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <cstdlib>
#include <ctime>

#define MAX_NOISE 0.1


namespace gps_odom
{
    
    class GpsOdom : public rclcpp::Node {
    // Define constants for Mercator projection
    const double EARTH_RADIUS = 6371000.0; // Earth's radius in meters
    const double METERS_PER_DEGREE = EARTH_RADIUS * M_PI / 180.0;
public:
    GpsOdom();
    ~GpsOdom();
private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _gnssSub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _gpsOdomPub;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry _odomMsg;
    double _initial_x, _initial_y;
    bool _odomInitialized;

    /**
     * Function used to publish the odometry
    */
    void publishOdom();

    void gnssCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    /**
     * code to conver the latitude and longitude
     * 
     * to x y coordinates
    */
    void convertLatLng2XYCoord(const double lat, const double lng, double& x, double& y);
    };
} // namespace gps_odom
