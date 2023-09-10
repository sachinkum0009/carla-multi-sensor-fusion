#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>



namespace odometry_fusion
{
    class OdometryFusion : public rclcpp::Node {
    public:
        OdometryFusion();
        ~OdometryFusion();
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_, laser_odom_sub_, gps_odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
        nav_msgs::msg::Odometry fused_odom;

        Eigen::Matrix<double, 3, 1> x_;  // State vector [x, y, theta]
        Eigen::Matrix<double, 3, 3> P_;  // State covariance matrix
        Eigen::Matrix<double, 3, 3> Q_;  // Process noise covariance
        Eigen::Matrix<double, 3, 3> R_;  // Measurement noise covariance

        rclcpp::TimerBase::SharedPtr timer_;

        void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void laserOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
        /**
         * Extended Kalman Filter
        */
        void ekfUpdate();
        
    };
} // namespace odometry_fusion
