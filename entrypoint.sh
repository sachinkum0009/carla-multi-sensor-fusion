#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash

cd /colcon_ws/src/zenoh_ros_bridge/

./zenoh-bridge-ros2dds -c robot_config.json5 &

# Run the ROS 2 opencv publisher node
ros2 launch sensor_fusion_perception opencv_publisher.launch.py



# zenoh-bridge-ros2dds -e tcp/192.168.117.110:7447