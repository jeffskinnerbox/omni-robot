#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_turtle_gamepad = get_package_share_directory('turtle_gamepad_2')

    # Config file path
    config_file = os.path.join(pkg_turtle_gamepad, 'config', 'f310_config.yaml')

    # Turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config_file],
        output='screen'
    )

    # Turtle gamepad controller
    turtle_controller = Node(
        package='turtle_gamepad_2',
        executable='turtle_gamepad_controller',
        name='turtle_gamepad_controller',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        turtlesim_node,
        joy_node,
        turtle_controller
    ])
