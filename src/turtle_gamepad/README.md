<!--
Maintainer:   jeffskinnerbox@yahoo.com / www.jeffskinnerbox.me
Version:      1.1.0
-->


This package enables control of the robot simultaneous via multiple user interface devices.
This includes the keyboard, game controller, and emergency stop button.
The architecture allows for the expansion to additional hardware devices (e.g mobile phone, website, etc.).


---------------


My objective is to use ROS2 Jazzy want to control the turtle `ros-jazzy-turtlesim`
and control it via a Logitech F310 Wired Gamepad Controller.
To assist in a design for this project, I used the following
[Claude](https://claude.ai/) / [Gemini](https://gemini.google.com/) Prompt:

 _My goal is to using a game controller, Python, and ROS2 Jazzy Jalisco to control the movement of the turtle within the ROS2 turtlesim node.
 The human operator will control the movement of turtle using a Logitech F310 Wired Gamepad Controller.
 In the solution, use the ros-jazzy-turtlesim, ros-jazzy-joy, and ros-jazzy-teleop-twist-joy ROS2 packages.
 For a standard Logitech F310 in XInput mode on Linux, provided a complete YAML configuration file
 and read that into the teleop node. Provide me guidance on what ROS2 nodes I should use
 and how I should configure them to control the turtlesim node turtle._

This can be found at [ROS2 Turtlesim Gamepad Control](https://claude.ai/chat/fe9862f4-2465-4564-9b3f-232c668e606f).


---------------


# Game Controller Not Working
I had a stubborn problem getting the projects working with the Logitech F310 game controller.
I initial tested it out with the Linux joystick test tool `jstest` and had no problems.
Once I created my `joy_node` to read data from the F310,
I used the `ros2 topic echo /joy` to validate my node was receiving data.
I got no data echoed out.
I also noticed, via `ros2 run joy joy_enumerate_devices`, that my node could not even see the device.

The code block below outlines the how I performed the testing and where it failed:

```bash
# this works fine - OK
jstest /dev/input/js0

# then started joy_node connection to game controller (it appeared) - OK
ros2 run joy joy_node

# but this had no output from the joystick - Failure
ros2 topic echo /joy

# and the joy_node couldn't see the game controller since this had no output - Failure
ros2 run joy joy_enumerate_devices
```

I did some extensive websurfing and found this webpost:
"[ROS2 joy_node debugging?](https://answers.ros.org/question/384456/)".
I found that the ROS2 node use `/dev/input/event*` devices
and not `/dev/input/js0` like document/implied in many places.
In effect, this site states you have to do the following:
`sudo chmod o+r /dev/input/event*`.

## Required Remediation
The silent failure of this solution is very undesirable.
An error message should have been printed when the game controller wasn't detected
and the program stopped.


---------------

# Clean Build When Modifying ROS2 Workspace
I have found that I will have difficulty in getting a clean build when making major modifications of my ROS2 workspace.
It appears that improper environment variables, left unchanged after the modification, are the root cause of my problems.
ROS 2 uses environment setup files (like `setup.bash`) to configure your shell environment so that it knows how to find
and use ROS 2 packages and executables.
These files set variables like `AMENT_PREFIX_PATH` and `CMAKE_PREFIX_PATH`.

To clean out your environment variables to prepare for a clean build of your ROS2 workspace, you should:

1. **Open a new terminal**: This is the simplest way to get a clean shell environment without any ROS 2 environment variables sourced.
2. **Ensure no ROS2 setup files are sourced in your startup scripts**: Check your `~/.bashrc` (or equivalent shell startup file)
   and remove or comment out any lines that source a ROS 2 workspace's.
3. **Manually remove the build, install, and log directories**: Navigate to the root of your ROS 2 workspace and run the following command:
   `cd <ros2_workspace>  &&  rm -r build install log`

Optional methods for cleaning the workspace:

1. **Using `colcon-clean`**: You can install the `colcon-clean` extension for `colcon` (the ROS 2 build tool)
   and use the command colcon clean workspace to clean the build, install, and log directories.
2. **Scripting the removal**: You could create a simple script to automate the removal of the
   build, install, and log directories if you need to do this often.


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
# to assure a clean build, start in a new terminal session
term-ros

# set ROS2 environment variables
source /opt/ros/jazzy/setup.bash

# clean out workspace
cd ~/omni-robot
rm -rf ./build/ ./install/ ./log/

# rebuild all your packages in the workspace
colcon build
```


---------------


# Completely Rebuild your ROS2 Workspace
By following these steps, you will effectively clean out old build artifacts and rebuild your workspace,
ensuring that your ROS2 Jazzy Python environment is fresh and properly configured.

```bash
# OPEN A NEW TERMINAL - simplest way to get a clean shell environment without any previous ROS 2 environment variables
term-ros

# navigate to your workspace directory and remove previously build artifacts
cd omni-robot
rm -rf ./build/ ./install/ ./log/

# source your base ROS 2 installation (if necessary)
source /opt/ros/jazzy/setup.bash

# resolve dependencies to ensure all package dependencies are met
rosdep install -i --from-path src/turtle_gamepad --rosdistro jazzy -y   # <--- do this to build turtle_gamepad package
rosdep install -i --from-path src --rosdistro jazzy -y                  # <--- only do this if you want to build ALL packages

# re-build the workspace
colcon build --packages-select turtle_gamepad                           # <--- do this to build turtle_gamepad package
colcon build                                                            # <--- only do this if you want to build ALL packages

# Source the workspace overlay to make the newly created packages available
source ~/omni-robot/install/setup.bash

# test the set of packages
colcon test --packages-select turtle_gamepad                            # <--- to test just the turtle_gamepad package
colcon test                                                             # <--- to test ALL packages
```


---------------


# Test Configuration - Using Manual Launch Commands
I used the code block below to setup my test of the solution.
Each line was initiated in a separate terminal so I could
control & observe what was going on.

```bash
# this provides a visual representation of the node & topic architecture
rqt_graph &

# Terminal 1 - Launch turtlesim
source ~/omni-robot/install/setup.bash
ros2 run turtlesim turtlesim_node

# Terminal 2 - Launch joy node
source ~/omni-robot/install/setup.bash
ros2 run joy joy_node --ros-args -p device_id:=0

# Terminal 3 - Launch teleop node with config
source ~/omni-robot/install/setup.bash
ros2 run teleop_twist_joy teleop_node --ros-args \
  --params-file ~/omni-robot/src/turtle_gamepad/config/teleop_twist_joy_config.yaml \
  -r cmd_vel:=/turtle1/cmd_vel
```
