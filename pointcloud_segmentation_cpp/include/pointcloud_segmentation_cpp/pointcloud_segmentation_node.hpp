#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "dbscan.hpp"
#include <iostream>
#include <string>
#include <system_error>
#include <vector>
#include <utility>
#include <fstream>
#include <charconv>
#include <cassert>
#include <tuple>
#include <cstring>

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// noise will be labelled as 0
auto label(const std::vector<std::vector<size_t>>& clusters, size_t n)
{
    auto flat_clusters = std::vector<size_t>(n);

    for(size_t i = 0; i < clusters.size(); i++)
    {
        for(auto p: clusters[i])
        {
            flat_clusters[p] = i + 1;
        }
    }

    return flat_clusters;
}

namespace pointcloud_segmentation_node
{
    class PointCloudSegmentation : public rclcpp::Node 
    {
public:
    PointCloudSegmentation();
    ~PointCloudSegmentation();
private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg);
    /*
    dbscan3d
    */
    void dbscan3d(const std::span<const float>& data, float eps, int min_pts);

    float epsilon;
    int min_pts;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    };
} // namespace pointcloud_segmentation_node
