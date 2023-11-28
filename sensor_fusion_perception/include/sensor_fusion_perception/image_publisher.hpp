#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace image_publisher
{
    class ImagePublisher : public rclcpp::Node {
        public:
        ImagePublisher();
        ~ImagePublisher();

        private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub_;
        rclcpp::TimerBase::SharedPtr timer_;

        sensor_msgs::msg::Image::SharedPtr imageMsg_;
        cv::Mat frame_;
        
        // cv::VideoCapture cap_;
        


        void publishImages();
    };
} // namespace image_publisher
