#!/usr/bin/env python3

"""
Launch script for turtlesim gamepad control
This script launches all required nodes with proper configuration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the config file
    config_file = os.path.join(
        get_package_share_directory('turtle_gamepad'),
        'config',
        'teleop_twist_joy_config.yaml'
    )

    return LaunchDescription([
        # Launch turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        # Launch joy node for gamepad input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,  # Usually 0 for first gamepad
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen'
        ),

        # Launch teleop_twist_joy node with configuration
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_file],
            remappings=[
                ('cmd_vel', '/turtle1/cmd_vel'),  # Remap to turtlesim topic
            ],
            output='screen'
        ),
    ])
