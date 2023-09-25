# Carla Multi Sensor Fusion

[![Actions Status](https://github.com/carla-simulator/ros-bridge/workflows/CI/badge.svg)](https://github.com/carla-simulator/ros-bridge)
[![Documentation](https://readthedocs.org/projects/carla/badge/?version=latest)](http://carla.readthedocs.io)
[![GitHub](https://img.shields.io/github/license/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/blob/master/LICENSE)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/releases/latest)

 This ROS package is a bridge that enables two-way communication between ROS and CARLA. The information from the CARLA server is translated to ROS topics. In the same way, the messages sent between nodes in ROS get translated to commands to be applied in CARLA.



## Features

- Provide Sensor Data (Lidar, Semantic lidar, Cameras (depth, segmentation, rgb, dvs), GNSS, Radar, IMU)
- Provide Object Data (Transforms (via [tf](http://wiki.ros.org/tf)), Traffic light status, Visualization markers, Collision, Lane invasion)
- Control AD Agents (Steer/Throttle/Brake)
- Control CARLA (Play/pause simulation, Set simulation parameters)

## Commands

```bash
# Option 1, start the basic ROS bridge package
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py

# Option 2, start the ROS bridge with an example ego vehicle
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py

# for scan matching point cloud odom
ros2 launch multi_sensor_fusion scan_matching.launch.py

# for gps to odom
ros2 launch multi_sensor_fusion gps_odom.launch.py

# for odom fusion
ros2 launch multi_sensor_fusion odom_fusion.launch.py

# for plot juggler
ros2 launch multi_sensor_fusion plot_juggler.launch.py

```

## ROS2 Graph

![ROS2 Graph](rosgraph.png)

## Sensor Fusion EKF

![Sensor Fusion](sensor_fusion.gif)

### Available sensors for fusion

1. Lidar (point cloud data ) -> (x, y) (improvize)
2. GPS (lat long) -> (x, y) (low accuracy)
3. Odometry -> (x, y) (random noise)
4. Camera -> (x, y)
5. IMU -> (euler, yaw, pitch , roll)



## Sensor Fusion

1. Kalman Filter (ekf)


## Collison Track

1. If vehicle is coming from the infront then try to predict in how much seconds the collision is going to happen

## Predict position using sensor fusion

