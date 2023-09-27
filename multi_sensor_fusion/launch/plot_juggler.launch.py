#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Nodes launching commands
    plotjuggler = launch_ros.actions.Node(
            package='plotjuggler',
            executable='plotjuggler',
            output='screen',)

        

    ld = LaunchDescription()

    ld.add_action(plotjuggler)

    return ld