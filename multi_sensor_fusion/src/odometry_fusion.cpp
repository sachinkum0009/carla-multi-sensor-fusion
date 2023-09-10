#include "multi_sensor_fusion/odometry_fusion.hpp"

namespace odometry_fusion
{
    OdometryFusion::OdometryFusion() : Node("odometry_fusion") {
        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/wheel_odom_topic", 10, std::bind(&OdometryFusion::wheelOdomCallback, this, std::placeholders::_1));
        laser_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/laser_odom", 10, std::bind(&OdometryFusion::wheelOdomCallback, this, std::placeholders::_1));
        gps_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/gps_odom", 10, std::bind(&OdometryFusion::wheelOdomCallback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu_topic", 10, std::bind(&OdometryFusion::imuCallback, this, std::placeholders::_1));

        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/fused_odom", 10);

        fused_odom.header.frame_id = "odom";
        fused_odom.child_frame_id = "base_link";

        // Initialize EKF variables
        x_ = Eigen::Matrix<double, 3, 1>::Zero();
        P_ = Eigen::Matrix<double, 3, 3>::Identity();
        Q_ = Eigen::Matrix<double, 3, 3>::Identity() * 0.01;  // Process noise covariance
        R_ = Eigen::Matrix<double, 3, 3>::Identity() * 0.1;   // Measurement noise covariance

        // Set up a timer for EKF fusion update
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdometryFusion::ekfUpdate, this));
    }

    OdometryFusion::~OdometryFusion() {

    }

    void OdometryFusion::ekfUpdate()
    {
        // Implement Extended Kalman Filter update step here
        // Combine information from wheel odometry and IMU
        // Publish the fused odometry message

        // Example code to publish a fused odometry message:
        
        fused_odom.header.stamp = this->now();

        // Fill in the fused_odom.pose.pose and fused_odom.twist.twist fields from the EKF state
        fused_odom.pose.pose.position.x = x_(0);
        fused_odom.pose.pose.position.y = x_(1);
        fused_odom.pose.pose.position.z = 0.0;  // Assuming 2D odometry
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x_(2));
         fused_odom.pose.pose.orientation = tf2::toMsg(q);

        // Assuming no linear velocity for simplicity
        fused_odom.twist.twist.linear.x = 0.0;
        fused_odom.twist.twist.linear.y = 0.0;
        fused_odom.twist.twist.linear.z = 0.0;
        fused_odom.twist.twist.angular.x = 0.0;
        fused_odom.twist.twist.angular.y = 0.0;
        fused_odom.twist.twist.angular.z = 0.0;

        // Publish the fused odometry message
        fused_odom_pub_->publish(fused_odom);
    }

    void OdometryFusion::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Implement fusion logic for IMU data here
        // Update state x_ and covariance P_ using the IMU data
        // For example, you can update x_ and P_ based on the incoming IMU message.
    }

    void OdometryFusion::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Implement fusion logic for wheel odometry here
        // Update state x_ and covariance P_ using the wheel odometry data
        // For example, you can update x_ and P_ based on the incoming wheel odometry message.
    }


    
} // namespace odometry_fusion


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odometry_fusion::OdometryFusion>());
    rclcpp::shutdown();
    return 0;
}