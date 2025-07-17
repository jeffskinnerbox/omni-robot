#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Package path
    pkg_share = FindPackageShare('manual_drive')
    config_dir = os.path.join(pkg_share.find('manual_drive'), 'config')

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )

    # Teleop twist joy node
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[os.path.join(config_dir, 'teleop_twist_joy.yaml')],
        remappings=[
            ('cmd_vel', 'cmd_vel_joy')
        ]
    )

    # Twist mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[os.path.join(config_dir, 'twist_mux.yaml')],
        remappings=[
            ('cmd_vel_out', 'cmd_vel')
        ]
    )

    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node,
        twist_mux_node
    ])
