#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch arguments
    joy_config_arg = DeclareLaunchArgument(
        'joy_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('teleoperator'),
            'config',
            'f310_config.yaml'
        ]),
        description='Path to joystick configuration file'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[LaunchConfiguration('joy_config')]
    )

    # Teleop twist joy node
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[LaunchConfiguration('joy_config')],
        remappings=[
            ('/cmd_vel', '/cmd_vel_joy')
        ]
    )

    return LaunchDescription([
        joy_config_arg,
        joy_node,
        teleop_twist_joy_node
    ])
