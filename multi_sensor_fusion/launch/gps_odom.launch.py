#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Nodes launching commands
    gps_odom_node = launch_ros.actions.Node(
            package='multi_sensor_fusion',
            executable='gps_odom',
            output='screen',)

        

    ld = LaunchDescription()

    ld.add_action(gps_odom_node)

    return ld