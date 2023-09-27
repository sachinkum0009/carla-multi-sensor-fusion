#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Nodes launching commands
    radar_safety_node = launch_ros.actions.Node(
            package='multi_sensor_fusion',
            executable='radar_safety',
            output='screen',)

        

    ld = LaunchDescription()

    ld.add_action(radar_safety_node)

    return ld