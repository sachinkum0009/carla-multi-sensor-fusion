#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Nodes launching commands
    
    odom_fusion_node = launch_ros.actions.Node(
            package='multi_sensor_fusion',
            executable='odometry_fusion',
            output='screen',)

        

    ld = LaunchDescription()

    ld.add_action(odom_fusion_node)

    return ld