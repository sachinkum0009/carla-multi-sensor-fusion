#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Nodes launching commands
    
    opencv_publisher_node = launch_ros.actions.Node(
            package='sensor_fusion_perception',
            executable='opencv_publisher.py',
            output='screen',)

        

    ld = LaunchDescription()

    ld.add_action(opencv_publisher_node)

    return ld