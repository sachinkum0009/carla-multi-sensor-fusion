#!/usr/bin/env python3

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    # Nodes launching commands
    pointcloud2_to_scan = launch_ros.actions.Node(
        package="pointcloud_to_laserscan", 
        executable="pointcloud_to_laserscan_node",
        remappings=[('cloud_in', '/carla/ego_vehicle/lidar'),
                    ('scan', '/carla/ego_vehicle/scan')],
        parameters=[{
            'target_frame': 'cloud',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.0087,
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': False,
            'inf_eplison': 1.0,
        
        }],
        name='pointcloud_to_laserscan'
    )
    scan_matching_node = launch_ros.actions.Node(
            package='multi_sensor_fusion',
            executable='scan_matching',
            output='screen',)

        

    ld = LaunchDescription()

    ld.add_action(scan_matching_node)
    ld.add_action(pointcloud2_to_scan)

    return ld