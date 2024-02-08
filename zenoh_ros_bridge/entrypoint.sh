#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

cd /zenoh

./zenoh-bridge-ros2dds -c host_config.json5