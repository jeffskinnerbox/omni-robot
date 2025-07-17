#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Emergency stop node
    emergency_stop_node = Node(
        package='manual_drive',
        executable='emergency_stop_node',
        name='emergency_stop_node'
    )

    # Keyboard teleop node
    keyboard_teleop_node = Node(
        package='manual_drive',
        executable='keyboard_teleop_node',
        name='keyboard_teleop_node'
    )

    # Safety monitor node
    safety_monitor_node = Node(
        package='manual_drive',
        executable='safety_monitor_node',
        name='safety_monitor_node'
    )

    return LaunchDescription([
        emergency_stop_node,
        keyboard_teleop_node,
        safety_monitor_node
    ])
