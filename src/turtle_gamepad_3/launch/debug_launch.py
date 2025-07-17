from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting turtle gamepad debug session..."),

        # Gamepad test
        ExecuteProcess(
            cmd=["jstest", "/dev/input/js0"],
            output="screen",
            prefix="xterm -e"
        ),

        # Topic monitoring
        ExecuteProcess(
            cmd=["ros2", "topic", "echo", "/joy"],
            output="screen",
            prefix="xterm -e"
        ),

        # Turtle velocity monitoring
        ExecuteProcess(
            cmd=["ros2", "topic", "echo", "/turtle1/cmd_vel"],
            output="screen",
            prefix="xterm -e"
        )
    ])
