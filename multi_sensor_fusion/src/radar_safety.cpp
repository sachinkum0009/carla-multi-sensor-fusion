#include "multi_sensor_fusion/radar_safety.hpp"


namespace radar_stop
{
    RadarStop::RadarStop() : Node("radar_stop"), safetyDistance(10.0f), initializeValues(false) {
        _radarPointcloudSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/carla/ego_vehicle/radar_front", 10, std::bind(&RadarStop::radarCallback, this, std::placeholders::_1));
        _egoControlPub = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);

        _egoControlMsg.throttle = 0.0f;
        _egoControlMsg.brake = 1.0f;
        _egoControlMsg.hand_brake = true;
        
    }
    RadarStop::~RadarStop() {}

    void RadarStop::radarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(), "range size %d", msg->data.size());

        if (!initializeValues) {
            for (const auto& field : msg->fields) {
                if (field.name == "Range") {
                    rangeOffset = field.offset;
                    break;  // Found the offset, exit the loop
                }
            }
            pointStep = msg->point_step;
            RCLCPP_INFO(this->get_logger(),"RANGE OFFSET IS %d, POINT STEP IS %d", rangeOffset, pointStep);
            initializeValues = true;
        }
        else {
            // RCLCPP_INFO(this->get_logger(), "ELSE LOOP");
            
            for (size_t i = 0; i < msg->data.size(); i+=pointStep) {
                std::memcpy(&range, &msg->data[i + rangeOffset], sizeof(float));
                // RCLCPP_INFO(this->get_logger(), "radar range is %f", range);
                if (range < safetyDistance) {
                    emergencyBrake();
                    break;
                }
            }

        }

    }

    void RadarStop::emergencyBrake() {
        RCLCPP_INFO(this->get_logger(), "Emergency Brakes Applied");

        _egoControlPub->publish(_egoControlMsg);
    }
    
} // namespace radar_safety

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar_stop::RadarStop>());
    rclcpp::shutdown();
    return 0;
}
