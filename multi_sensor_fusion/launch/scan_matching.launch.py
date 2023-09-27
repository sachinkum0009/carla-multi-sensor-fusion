#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Nodes launching commands
    scan_matching_node = launch_ros.actions.Node(
            package='multi_sensor_fusion',
            executable='scan_matching',
            output='screen',)

        

    ld = LaunchDescription()

    ld.add_action(scan_matching_node)

    return ld