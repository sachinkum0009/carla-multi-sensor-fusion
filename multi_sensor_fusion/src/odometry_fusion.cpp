#include "multi_sensor_fusion/odometry_fusion.hpp"

namespace odometry_fusion
{
    OdometryFusion::OdometryFusion() : Node("odometry_fusion") {

        fused_odom.header.frame_id = "odom";
        fused_odom.child_frame_id = "base_link";

        // Initialize EKF variables
        x_ = Eigen::Matrix<double, 3, 1>::Zero();
        P_ = Eigen::Matrix<double, 3, 3>::Identity();
        P_(2,2) = 0.1;
        Q_ = Eigen::Matrix<double, 3, 3>::Identity() * 0.01;  // Process noise covariance
        R_ = Eigen::Matrix<double, 3, 3>::Identity() * 0.1;   // Measurement noise covariance

        H_ = Eigen::Matrix<double, 3, 3>::Identity();

        wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/relative_odom", 10, std::bind(&OdometryFusion::wheelOdomCallback, this, std::placeholders::_1));
        laser_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/lidar_odom", 10, std::bind(&OdometryFusion::laserOdomCallback, this, std::placeholders::_1));
        gps_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/gps_odom", 10, std::bind(&OdometryFusion::gpsOdomCallback, this, std::placeholders::_1));
        // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/carla/ego_vehicle/imu", 10, std::bind(&OdometryFusion::imuCallback, this, std::placeholders::_1));

        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/fused_odom", 10);
        // timer to update EKF 
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdometryFusion::ekfUpdate, this));
    }

    OdometryFusion::~OdometryFusion() {

    }

    void OdometryFusion::ekfUpdate()
    {
        // Implement Extended Kalman Filter update step here
        // Prediction Step
        xPred = x_;
        pPred = P_ + Q_;

        // Correction Step
        wheelMeasurementResidual = wheelMeasurement - xPred;
        laserMeasurementResidual = laserMeasurement - xPred;
        gpsMeasurementResidual = gpsMeasurement - xPred;

        S_ = H_ * pPred * H_.transpose() + R_;
        K_ = (pPred * H_.transpose()) * S_.inverse();

        // RCLCPP_INFO(this->get_logger(), "%f , %f", x_(0), x_(1));

        x_ = xPred + K_ * wheelMeasurementResidual + K_ * laserMeasurementResidual + K_ * gpsMeasurementResidual;
        P_ = (H_ - (K_* H_)) * pPred;
        

        fused_odom.header.stamp = this->now();

        fused_odom.pose.pose.position.x = x_(0);
        fused_odom.pose.pose.position.y = x_(1);
        fused_odom.pose.pose.position.z = 0.0;  
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x_(2));
        fused_odom.pose.pose.orientation = tf2::toMsg(q);


        // Publish the fused odometry message
        fused_odom_pub_->publish(fused_odom);
    }

    void OdometryFusion::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
    }

    void OdometryFusion::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        wheelMeasurement[0] = msg->pose.pose.position.x;
        wheelMeasurement[1] = msg->pose.pose.position.y;
    }



    void OdometryFusion::laserOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        laserMeasurement[0] = msg->pose.pose.position.x;
        laserMeasurement[1] = msg->pose.pose.position.y;
    }
    
    void OdometryFusion::gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        gpsMeasurement[0] = msg->pose.pose.position.x;
        gpsMeasurement[1] = msg->pose.pose.position.y;
    }
} // namespace odometry_fusion


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odometry_fusion::OdometryFusion>());
    rclcpp::shutdown();
    return 0;
}