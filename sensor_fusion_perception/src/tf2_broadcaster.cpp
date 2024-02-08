#include "sensor_fusion_perception/tf2_broadcaster.hpp"

namespace tf2_broadcaster
{
    Tf2Broadcaster::Tf2Broadcaster() : Node("tf2_broadcaster") {
        _transformStamped.child_frame_id = "front_radar";
        _transformStamped.header.frame_id = "base_link";
        _transformStamped.transform.translation.x = 0.0f;
        _transformStamped.transform.translation.y = 0.0f;

        // Initialize the transform broadcaster
        _tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf2::Quaternion _q;
        _q.setRPY(1.571f, 0.0f, 0.0f);
        _transformStamped.transform.rotation =  tf2::toMsg(_q);
        // _transformStamped.transform.rotation.w = 1.0f;

        _timerBase = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Tf2Broadcaster::_timerCallback, this));

    }
    Tf2Broadcaster::~Tf2Broadcaster() {}

    void Tf2Broadcaster::_timerCallback() {
        _transformStamped.header.stamp = this->get_clock()->now();
        _tfBroadcaster->sendTransform(_transformStamped);
        RCLCPP_INFO(this->get_logger(), "tf2 broadcaster");


    }
} // namespace tf2_broadcaster


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tf2_broadcaster::Tf2Broadcaster>());
    rclcpp::shutdown();
    return 0;
}
