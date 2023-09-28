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

        Eigen::Matrix<double, 3, 1> x_; // State Vector [x, y, theta]
        Eigen::Matrix<double, 3, 3> P_;
        Eigen::Matrix<double, 3, 3> Q_;
        Eigen::Matrix<double, 3, 3> R1_, R2_, R3_; // R1 -> wheel // R2 -> lidar // R3 -> gps
         
        Eigen::Matrix<double, 3, 1> wheelMeasurement, laserMeasurement, gpsMeasurement;
        Eigen::Matrix<double, 3, 1> xPred;
        Eigen::Matrix<double, 3, 3> pPred; 
        Eigen::Matrix<double, 3, 1> wheelMeasurementResidual, laserMeasurementResidual, gpsMeasurementResidual;
        Eigen::Matrix<double, 3, 3> S1_, S2_, S3_, K1_, K2_, K3_; 
        Eigen::Matrix<double, 3, 3> H_;
        

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
