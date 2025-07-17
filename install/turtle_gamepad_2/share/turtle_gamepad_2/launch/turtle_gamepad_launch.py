#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_turtle_gamepad = get_package_share_directory('turtle_gamepad_2')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_turtle_gamepad, 'config', 'f310_config.yaml'),
        description='Full path to joy config file'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('joy', 'joy')
        ]
    )

    # Teleop twist joy node
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('joy', 'joy'),
            ('cmd_vel', 'cmd_vel_joy')
        ]
    )

    # Turtle gamepad controller
    turtle_gamepad_controller = Node(
        package='turtle_gamepad_2',
        executable='turtle_gamepad_controller',
        name='turtle_gamepad_controller',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    # Emergency stop monitor
    emergency_stop_monitor = Node(
        package='turtle_gamepad_2',
        executable='emergency_stop_monitor',
        name='emergency_stop_monitor',
        output='screen'
    )

    # Turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        joy_node,
        teleop_twist_joy_node,
        turtle_gamepad_controller,
        emergency_stop_monitor,
        turtlesim_node
    ])
