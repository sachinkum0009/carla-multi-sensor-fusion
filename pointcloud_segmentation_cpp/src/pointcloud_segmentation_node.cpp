#include "pointcloud_segmentation_cpp/pointcloud_segmentation_node.hpp"


namespace pointcloud_segmentation_node
{
    PointCloudSegmentation::PointCloudSegmentation() : Node("pointcloud_segmentation")
    {
        epsilon = 0.4;
        min_pts = 20;
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, std::bind(&PointCloudSegmentation::topic_callback, this, std::placeholders::_1));
    }

    PointCloudSegmentation::~PointCloudSegmentation() {}

    void PointCloudSegmentation::dbscan3d(const std::span<const float>& data, float eps, int min_pts)
    {
        RCLCPP_INFO(this->get_logger(), "dbscan3d called");
        auto points = std::vector<point3>(data.size() / 3);

        std::memcpy(points.data(), data.data(), sizeof(float) * data.size());

        auto clusters = dbscan(points, eps, min_pts);
        auto flat     = label(clusters, points.size());

        // for(size_t i = 0; i < points.size(); i++)
        // {
        //     std::cout << points[i].x << ',' << points[i].y << ',' << points[i].z << ',' << flat[i] << '\n';
        // }
    }
    void PointCloudSegmentation::topic_callback(const sensor_msgs::msg::PointCloud2 & msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: pointcloud");
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(msg, *cloud);
      std::cout << cloud->points[0] << std::endl;

      std::vector<float> data;
        for (const auto& point : cloud->points) {
            data.push_back(point.x);
            data.push_back(point.y);
            data.push_back(point.z);
        }

        std::span<const float> data_span(data);
        dbscan3d(data_span, epsilon, min_pts);

    }
} // namespace pointcloud_segmentation_node



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_segmentation_node::PointCloudSegmentation>());
  rclcpp::shutdown();
  return 0;
}