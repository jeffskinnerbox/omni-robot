from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Run tests
        ExecuteProcess(
            cmd=["python3", "-m", "pytest", "src/turtle_gamepad_3/test/", "-v"],
            cwd=os.getcwd(),
            output="screen"
        )
    ])
