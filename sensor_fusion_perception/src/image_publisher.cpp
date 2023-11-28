#include "sensor_fusion_perception/image_publisher.hpp"

namespace image_publisher
{
    ImagePublisher::ImagePublisher() : Node("image_publisher") {
        frame_ = cv::imread("/home/asus/zzzzz/ros2/acceleration_robotics/research_thesis/colcon_ws/src/carla-multi-sensor-fusion/YOLOP/inference/images/ce480c21-65967e6b.jpg");
        imageMsg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_).toImageMsg();

        imagePub_ = this->create_publisher<sensor_msgs::msg::Image>("raw_image", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ImagePublisher::publishImages, this));
        
        // publishImages();

    }
    ImagePublisher::~ImagePublisher() {}

    void ImagePublisher::publishImages() {
        // if(!cap_.isOpened()) {
        //     RCLCPP_ERROR(this->get_logger(), "Error opening file");
        //     exit(1);
        // }


        // while (true) {
        //     cap_ >> frame_;
        //     if (frame_.empty())
        //         break;
            

        //     imagePub_->publish(*imageMsg_);
        // }

        imagePub_->publish(*imageMsg_);

    }
    
} // namespace image_publisher


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // std::string packageShareDirectory = ament_index_cpp::get_package_share_directory("sensor_fusion_perception");

    rclcpp::spin(std::make_shared<image_publisher::ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
