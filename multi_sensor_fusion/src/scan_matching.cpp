#include "multi_sensor_fusion/scan_matching.hpp"

namespace scan_matching
{
    
    ScanMatching::ScanMatching() : Node("scan_matching"), _cloud(new pcl::PointCloud<pcl::PointXYZ>), _prevCloud(new pcl::PointCloud<pcl::PointXYZ>), _initializePointCloud(false), setOdomOffset(false) {
        // Initialize the ICP registration
        _icp.setMaxCorrespondenceDistance(50); // Maximum distance for point correspondences
        _icp.setMaximumIterations(200);           // Maximum number of iterations

        _icp.setTransformationEpsilon(1e-2);
        _icp.setEuclideanFitnessEpsilon(1e-6);
        _icp.setRANSACIterations(40);

        
        
        _odomPub = this->create_publisher<nav_msgs::msg::Odometry>("/lidar_odom", 10);
        _alignedPointCloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_pc", 10);
        _scanSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/carla/ego_vehicle/lidar", 10, std::bind(&ScanMatching::scanCallback, this, std::placeholders::_1));
        _odomSub = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 10, std::bind(&ScanMatching::odomCallback, this, std::placeholders::_1));
        _relativeOdomPub = this->create_publisher<nav_msgs::msg::Odometry>("/relative_odom", 10);
        
        _odometryMsg.header.frame_id = "odom";
        _odometryMsg.child_frame_id = "base_link";

        _relativeOdomMsg.header.frame_id = "odom";
        _relativeOdomMsg.child_frame_id = "base_link";

        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ScanMatching::publishOdom, this));
    }
    ScanMatching::~ScanMatching() {}

    void ScanMatching::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!setOdomOffset) {
            x_offset = msg->pose.pose.position.x;
            y_offset = msg->pose.pose.position.y;
            setOdomOffset = true;
        }
        else {
            _prevOdometryMsg.pose = msg->pose;
        }
        
    }


    void ScanMatching::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "I got scan message");
        // RCLCPP_INFO(this->get_logger(), "laser size is: %d", scan_msg->ranges.size());
        // if (!_initializePointCloud) {
        //     // Convert ROS PointCloud2 to PCL PointCloud
        //     pcl::fromROSMsg(*msg, *_prevCloud);
        // }
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::fromROSMsg(*msg, *_cloud);
        // If it's the first point cloud received, store it as the previous cloud
        if (!_prevCloud->size()) {
            *_prevCloud = *_cloud;
            return;
        }


        _icp.setInputSource(_cloud);             // Set the source point cloud (current frame)
        _icp.setInputTarget(_prevCloud);        // Set the target point cloud (previous frame)
        
        // Perform ICP registration to estimate transformation (motion)
        _icp.align(_alignedCloud);
        pcl::toROSMsg(_alignedCloud, _pointCloudMsg);
        _alignedPointCloudPub->publish(_pointCloudMsg);

        // Get the estimated transformation
        _transformation = _icp.getFinalTransformation();

        // Extract translation and rotation components from the transformation matrix
        _translation += _transformation.block<3, 1>(0, 3);
        _rotation += _transformation.block<3, 3>(0, 0);

        RCLCPP_INFO(this->get_logger(), "translation x: %f, original x: %f", _translation.x(), (_prevOdometryMsg.pose.pose.position.x - x_offset)/10.0f);
        RCLCPP_INFO(this->get_logger(), "translation y: %f, orignal y: %f", _translation.y(), (_prevOdometryMsg.pose.pose.position.y - y_offset)/10.0f);

        // Add odometry message
        // _odometryMsg.header = msg->header;
        _odometryMsg.pose.pose.position.x = _translation.x();
        _odometryMsg.pose.pose.position.y = _translation.y();
        // _odometryMsg.pose.pose.position.z = _translation.z();

        // Convert the rotation matrix to a quaternion and set it in the odometry message
        Eigen::Quaternionf quaternion(_rotation);
        _odometryMsg.pose.pose.orientation.x = quaternion.x();
        _odometryMsg.pose.pose.orientation.y = quaternion.y();
        _odometryMsg.pose.pose.orientation.z = quaternion.z();
        _odometryMsg.pose.pose.orientation.w = quaternion.w();

        // Update the previous point cloud for the next iteration
        *_prevCloud = *_cloud;


        

    }

    void ScanMatching::publishOdom() {
        // Publish the odometry message
        _odometryMsg.header.stamp = this->get_clock()->now();
        _relativeOdomMsg.header.stamp = _odometryMsg.header.stamp;
        _relativeOdomMsg.pose.pose.position.x = (_prevOdometryMsg.pose.pose.position.x - x_offset)/10.0f;
        _relativeOdomMsg.pose.pose.position.y = (_prevOdometryMsg.pose.pose.position.y - y_offset)/10.0f;

        _relativeOdomPub->publish(_relativeOdomMsg);
        _odomPub->publish(_odometryMsg);

    }
} // namespace scan_matching



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<scan_matching::ScanMatching>());
    rclcpp::shutdown();
    return 0;
}
