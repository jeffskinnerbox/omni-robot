#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    twist_mux_config_arg = DeclareLaunchArgument(
        'twist_mux_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('teleoperator'),
            'config',
            'twist_mux_config.yaml'
        ]),
        description='Path to twist_mux configuration file'
    )

    # Include joy launch
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('teleoperator'),
                'launch',
                'joy_teleop.launch.py'
            ])
        ]),
        launch_arguments={
            'joy_config': LaunchConfiguration('joy_config')
        }.items()
    )

    # Turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    # Twist mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[LaunchConfiguration('twist_mux_config')],
        remappings=[
            ('/cmd_vel_out', '/turtle1/cmd_vel')
        ]
    )

    # Emergency stop node
    emergency_stop_node = Node(
        package='teleoperator',
        executable='emergency_stop',
        name='emergency_stop_node',
        output='screen'
    )

    # Keyboard teleop node
    keyboard_teleop_node = Node(
        package='teleoperator',
        executable='keyboard_teleop',
        name='keyboard_teleop_node',
        output='screen'
    )

    return LaunchDescription([
        joy_config_arg,
        twist_mux_config_arg,
        joy_launch,
        turtlesim_node,
        twist_mux_node,
        emergency_stop_node,
        keyboard_teleop_node
    ])
