<!--
Maintainer:   jeffskinnerbox@yahoo.com / www.jeffskinnerbox.me
Version:      1.1.0
-->


This package enables control of the robot simultaneous via multiple user interface devices.
This includes the keyboard, game controller, and emergency stop button.
The architecture allows for the expansion to additional hardware devices (e.g mobile phone, website, etc.).


---------------

_My goal is to using a game controller, Python, and ROS2 Jazzy Jalisco
to control the movement of the turtle within the ROS2 turtlesim node.
In the solution, use the ros-jazzy-turtlesim, ros-jazzy-joy, ros-jazzy-teleop-twist-joy,
and ros-jazzy-twist_mux ROS2 packages.
Provide me guidance on any additional ROS2 nodes I should use
and how I should configure them to control the turtlesim node turtle.__

_The human operator will control the movement of turtle using a Logitech F310 Wired Gamepad Controller,
a computer keyboard, and an emergency stop button.
For a standard Logitech F310 in XInput mode on Linux, provided a complete YAML configuration file
mapping all the F310 joysticks and buttons._

_Give me a step-by-step methodology for creating a solution, written in Python, for ROS2 Jazzy.
Make use of the ROS2 tools "colcon build" and "rosdep", make use of Python tools "pytest" for testing,
and show how to execute the ROS2 nodes manually and by using "launch"._

_I want to name the ROS2 workspace "omni-robot-2" and name the package "teleoperator".
Title the solution guide you create "ROS2 Jazzy Teleoperator".
Create a launch files for all nodes, for the joy nodes, and all nodes except ros-jazzy-turtlesim._

"ROS2 Jazzy Teleoperator" has been published publicly at this [link](https://claude.ai/public/artifacts/8bf612bc-5236-4157-9637-d78b09ae75f6)

Sources:

* [Controlling a Robot with multiple Inputs using twist_mux](https://robofoundry.medium.com/controlling-a-robot-with-multiple-inputs-using-twist-mux-4535b8ed9559)

---------------


# Kill a Running Node

To kill a running ROS2 node, do the following:

```bash
# get a list of all running nodes
ros2 node list

# for nodes that are implemented as lifecycle nodes, you can gracefully shut them down using this
ros2 lifecycle set <nodename> shutdown

# send a SIGTERM signal, which is a softs way to request a process to terminate
kill -TERM <PID>

# send a SIGKILL signal, which forces the process to terminate immediately, regardless of its state.
kill -9 <PID>
```


---------------


# Fixing Build Issues

```bash
# Clean build
cd ~/omni-robot
rm -rf build install log
colcon build
```


---------------


# Completely Rebuild your ROS2 Workspace

By following these steps, you will effectively clean out old build artifacts and rebuild your workspace,
ensuring that your ROS2 Jazzy Python environment is fresh and properly configured.

```bash
# navigate to your workspace directory and remove previously build artifacts
cd ros2_ws
rm -rf ./build/ ./install/ ./log/

# source your base ROS 2 installation (if necessary)
source /opt/ros/jazzy/setup.bash

# resolve dependencies to ensure all package dependencies are met
rosdep install -i --from-path src --rosdistro jazzy -y

# re-build the workspace
colcon build --symlink-install         # --symlink-install option can be used for faster iteration on Python files and other non-compiled resources

# Source the workspace overlay to make the newly created packages available
source ./install/setup.bash
```


