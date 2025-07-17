#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Package path
    pkg_share = FindPackageShare('manual_drive')

    # Include other launch files
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_share, '/launch/joy_nodes.launch.py'
        ])
    )

    manual_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_share, '/launch/manual_drive_nodes.launch.py'
        ])
    )

    # Turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    return LaunchDescription([
        turtlesim_node,
        joy_launch,
        manual_drive_launch
    ])
