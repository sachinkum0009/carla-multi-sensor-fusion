#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>


namespace scan_matching
{
    class ScanMatching : public rclcpp::Node {
public:
    ScanMatching();
    ~ScanMatching();
    
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _scanSub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odomPub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSub;

    sensor_msgs::msg::PointCloud2 _scanMsg;
    uint32_t ranges_size;
    double inf_eplison_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, _prevCloud;
    bool _initializePointCloud, setOdomOffset;
    float x_offset,y_offset;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;
    pcl::PointCloud<pcl::PointXYZ> _alignedCloud;
    Eigen::Matrix4f _transformation;
    Eigen::Vector3f _translation;
    Eigen::Matrix3f _rotation;
    nav_msgs::msg::Odometry _odometryMsg, _prevOdometryMsg;
    

    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    };
} // namespace scan_matching
