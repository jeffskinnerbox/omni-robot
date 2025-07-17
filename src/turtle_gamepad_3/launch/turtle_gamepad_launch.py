from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory("turtle_gamepad_3")
    config_file = os.path.join(pkg_dir, "config", "f310_config.yaml")

    # Launch arguments
    gamepad_device_arg = DeclareLaunchArgument(
        "gamepad_device",
        default_value="/dev/input/js0",
        description="Gamepad device path"
    )

    turtle_name_arg = DeclareLaunchArgument(
        "turtle_name",
        default_value="turtle1",
        description="Name of the turtle to control"
    )

    # Nodes
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        output="screen"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "dev": LaunchConfiguration("gamepad_device"),
            "deadzone": 0.1,
            "autorepeat_rate": 20.0
        }],
        output="screen"
    )

    turtle_controller_node = Node(
        package="turtle_gamepad_3",
        executable="turtle_controller",
        name="turtle_controller",
        parameters=[
            config_file,
            {"turtle_name": LaunchConfiguration("turtle_name")}
        ],
        output="screen"
    )

    return LaunchDescription([
        gamepad_device_arg,
        turtle_name_arg,
        turtlesim_node,
        joy_node,
        turtle_controller_node
    ])
