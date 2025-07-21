


This package enables control of the robot simultaneous via multiple user interface devices.
This includes the keyboard, game controller, and emergency stop button.
The architecture allows for the expansion to additional hardware devices (e.g mobile phone, website, etc.).


---------------


Claude Prompt - ROS2 Jazzy Turtlesim Controller:
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

_I want to name the ROS2 workspace "omni-robot" and name the package "mux-controller".
Title the solution guide you create "ROS2 Jazzy Manual Drive".
Create a launch files for all nodes, for the joy nodes, and all nodes except ros-jazzy-turtlesim._

"ROS2 Jazzy Manual Drive" has been published publicly at this [link](https://claude.ai/public/artifacts/8bf612bc-5236-4157-9637-d78b09ae75f6)

Sources:

* [Controlling a Robot with multiple Inputs using twist_mux](https://robofoundry.medium.com/controlling-a-robot-with-multiple-inputs-using-twist-mux-4535b8ed9559)

---------------
Teleoperation in ROS2 refers to remotely controlling a robot, allowing a human operator to influence the robot's actions from a distance. This involves sending commands to the robot (e.g., movement instructions, sensor data requests) and receiving feedback (e.g., camera images, sensor readings) from it, effectively making the robot an extension of the operator.

* [Add TeleOp to your ROS2 Robot](https://medium.com/@peter.gaston/add-teleop-to-your-ros2-robot-5b7b0a5606ce)

---------------


---------------


# Using Python Packages with ROS 2

The ROS2 Jazzy documentation has a cautionary note deeply buried inside. It states:

_A cautionary note, if you intended to use pre-packaged binaries (either `deb` files, or the binary archive distributions), the Python interpreter must match what was used to build the original binaries. If you intend to use something like `virtualenv` or `pipenv`, make sure to use the system interpreter. If you use something like `conda`, it is very likely that the interpreter will not match the system interpreter and will be incompatible with ROS 2 binaries._

And yes, mixing `apt` and `conda` for installing ROS 2 Jazzy packages can cause problems, especially concerning Python dependencies.
If you use official `apt` binaries, ensure your `PATH` environment variable doesn't include `conda` paths.
This is a common source of conflict.
Mixing `apt` and `conda`` can lead to a "mess" where some packages are installed via`apt`
and others via `conda` or `pip` within a `conda` environment.
This can cause issues when running ROS 2 tools and applications.

ROS2 Jazzy recommends:

_For installing ROS 2 Jazzy using the official `apt` binaries, it's best to avoid mixing `conda` environments
and rely solely on `apt`` for installing ROS 2 packages and their dependencies.
If you need to use Python packages not available in`apt`, consider installing them in a
virtual environment that uses the system Python interpreter.
However, be aware that even this approach may have limitations when working with ROS 2 binaries._

>**NOTE:** [RoboStack](https://robostack.github.io/index.html) provides a way to use ROS2 alongside the `conda` and `Jupyter` ecosystems,
>which might help manage dependencies and avoid conflicts.

Create a ROS 2 Python Package:

1. Create a ROS 2 Python Package:

* Set up a workspace: If you don't have a ROS 2 workspace, create one using `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws`.
* Create the package structure: In your workspace's `src` directory,
    use `ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy`.
* Add your Python modules: Place your custom Python code within a directory that has the same name as your package.
    Include an `__init__.py` file within this directory so Python recognizes it as a package.

2. Add Python Libraries to Your Package:

* Dependencies in `package.xml`: List any ROS 2 dependencies (e.g., `rclpy`) in your `package.xml` file using the `<depend>` tag.
* Using `rosdep`: For faster inclusion of third-party Python packages, check if they have a corresponding `rosdep` key.
    Add these keys to your `package.xml`. If a key doesn't exist, you can contribute one.
* Package manager installation: Alternatively, you can install Python packages globally `python3 -m pip install -U <package_name>` or
    locally using pip `python3 -m pip install -U --user <package_name>` or your system's package manager `sudo apt install  <package_name>`.
* Declaring dependencies in `setup.py``: Add external Python libraries to the`install_requires` list
    in your `setup.py` file to ensure they are installed when building your package.
    For external Python libraries from PyPI, include them in the `install_requires` list within your `setup.py` file.
    For instance, to use `numpy`, add 'numpy' to the list.
* Using a virtual environment: Create a virtual environment within your workspace and activate it.
    Install desired packages within the virtual environment using `python3 -m pip install <package_names>`.
    Make sure to add a `.venv/COLCON_IGNORE` file to prevent `colcon` from trying to build the virtual environment.

3. Develop Python Nodes:

* Create a Python node file: Add your Python code to a file within your package's source directory (e.g., `my_python_package/my_node.py`).
* Include required libraries: Import `rclpy` and other necessary libraries.
* Create a ROS 2 node: Instantiate a ROS 2 node using `rclpy.node.Node()` within your Python script.
* Define entry points in `setup.py`: Add a `console_scripts` entry in your `setup.py` file
    to make your Python script an executable ROS 2 node. This allows you to run it using `ros2 run`.

4. Build and Run Your Package:

* Build the workspace: Navigate to your workspace root and use colcon build to build your package and its dependencies.
* Source the setup file: After building, source the workspace's setup file (e.g., source install/local_setup.bash)
    in a new terminal to make the installed packages available.
* Run the node: Execute your Python node using ros2 run <package_name> <node_name>.

5. Important Notes:

* Python interpreter compatibility: Ensure your Python interpreter matches the one
    used to build pre-packaged ROS 2 binaries if you're using them.
* Build after changes: Remember to run `colcon build` whenever you make changes to your package,
    especially after adding new nodes or dependencies.
* Source after building: Always source your workspace's setup file in each new terminal session to access the built packages.
* Shebang line: Start your Python node files with `#!/usr/bin/env python3` and make them executable using `chmod +x`.

Sources:

* [Using Python Packages with ROS 2](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html)
* [ROS2 Part 1 – Create a ROS2 Python Package and Node](https://www.roboticsunveiled.com/ros2-python-package-and-node/)
* [ROS2 Fundamentals](https://www.roboticsunveiled.com/contents-ros2/#Fundamentals)


---------------


# `rosdep` Package Manager

When developing your ROS2 Jazzy robot, you can use `rosdep` to manage system dependencies
and ensure your project has the necessary external libraries and software to build and run correctly.
By following the steps below, you can effectively use `rosdep` to manage dependencies in your ROS2 Jazzy development environment,
ensuring a smoother development and build process.
Here's how you can use `rosdep`:

1. Understand Rosdep and Package Dependencies:

* `rosdep` is a command-line tool that handles system dependencies for ROS packages.
* Package dependencies are specified in the `package.xml` file within your ROS package.
* These dependencies are defined as "rosdep keys" that correspond to specific packages
    or libraries available in system package managers (like apt on Ubuntu/Debian).

2. Install rosdep:

* If you installed ROS2 Jazzy through binary packages, `rosdep` is likely included.
* Otherwise, you can install it using your system's package manager, for example, on Debian/Ubuntu: `sudo apt install python3-rosdep`.

3. Initialize and Update rosdep:

* If this is the first time using `rosdep`, you need to initialize it and update the local database:

    ```bash
    sudo rosdep init
    rosdep update
    ```

* `rosdep init` may fail if already initialized, which can be safely ignored.
* `rosdep update` refreshes the local copy of the `rosdistro` index, which contains information about available ROS packages and their dependencies.

4. Install Dependencies for Your Workspace:

* Navigate to the root of your ROS2 workspace, where your `src` directory containing package sources is located.
* Run `rosdep install` to install dependencies listed in the `package.xml` files within your workspace:

    ```bash
    rosdep install --from-paths src -y --ignore-src --rosdistro jazzy
    ```

* `--from-paths src`: Specifies the path to check for `package.xml` files.
* `-y`: Automatically answers "yes" to any prompts from the package manager during installation.
* `--ignore-src`: Tells rosdep to ignore packages found within the source directory,
    assuming they are part of your workspace and don't need external dependencies installed.
* `--rosdistro jazzy`: Specifies the ROS distribution to use for dependency resolution.

5. Adding Rosdep Keys to `package.xml`:

* If your package depends on an external library that isn't a ROS package,
    find the corresponding `rosdep` key (e.g., searching in `rosdep/base.yaml` or `rosdep/python.yaml` in the rosdistro repository).
* Add this key to your package's `package.xml` file using appropriate tags like `<depend>`, `<build_depend>`, `<exec_depend>`, etc.

6. Troubleshooting:

* If you encounter errors during dependency installation, ensure your `rosdep` is initialized and updated.
* Check for conflicting `rosdep` versions or issues with specific libraries
    (e.g., uninstalling a pip-installed version and installing the apt version).

Sources:

* [Gemini Prompt: How can I use rosdep when developing my ROS2 Jazzy robot?](https://gemini.google.com/)


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

I did some extensive web-surfing and found this webpost:
"[ROS2 joy_node debugging?](https://answers.ros.org/question/384456/)".
I found that the ROS2 node use `/dev/input/event*` devices
and not `/dev/input/js0` like document/implied in many places.
In effect, this site states you have to do the following:
`sudo chmod o+r /dev/input/event*`.

NOTE: I made this change but after a period of time (after a reboot?) it changes back to its orginal setting.

## Required Remediation

The silent failure of this solution is very undesirable.
An error message should have been printed when the game controller wasn't detected
and the program stopped.


---------------


# Mixed use of `apt` and `conda`

Gemini Prompt: _Can the mixed use of apt and conda in ROS2 cause problems?_

Mixing apt and conda without careful consideration can create compatibility problems in ROS2.

[RoboStack](https://robostack.github.io/index.html) provides a way to use ROS2 alongside the conda and Jupyter ecosystems,
which might help manage dependencies and avoid conflicts.

---------------


# Build & Install

```bash
# Navigate to workspace
cd ~/omni-robot

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```


---------------


# Testing

```bash
# Run tests
cd ~/omni-robot
colcon test --packages-select manual_drive

# Run specific test
python3 -m pytest src/manual_drive/test/test_emergency_stop.py -v
python3 -m pytest src/manual_drive/test/test_keyboard_teleop.py -v
python3 -m pytest src/manual_drive/test/test_safety_monitor.py -v
```


---------------


# Manual Execution

I used the code block below to setup my test of the solution.
Each line was initiated in a separate terminal so I could
control & observe what was going on.

```bash
# in the initial terminal, source the workspace
source /opt/ros/jazzy/setup.bash
source ~/omni-robot/install/setup.bash
rqt_graph &                              # graphical representation of the node & topic architecture

# now create the remaining terminals from within the initial terminal above

# terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# terminal 2: Start joy node
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0

# terminal 3: Start teleop twist joy
ros2 run teleop_twist_joy teleop_node --ros-args \
    --params-file ~/omni-robot/src/manual_drive/config/teleop_twist_joy.yaml \
    -r cmd_vel:=cmd_vel_joy

# terminal 4: Start twist mux    <----- THIS ABORTS - terminate called after throwing an instance of 'twist_mux::ParamsHelperException' what():  could not load parameter 'use_stamped'. (namespace: /)
ros2 run twist_mux twist_mux --ros-args \
    --params-file ~/omni-robot/src/manual_drive/config/twist_mux.yaml \
    -r cmd_vel_out:=cmd_vel

# terminal 5: Start emergency stop
ros2 run manual_drive emergency_stop_node

# terminal 6: Start keyboard teleop
ros2 run manual_drive keyboard_teleop_node

# terminal 7: Start safety monitor
ros2 run manual_drive safety_monitor_node

# terminal 8
ros2 topic echo /joy
ros2 topic echo /joy_cmd_vel
ros2 topic echo /key_cmd_vel
ros2 topic echo /test_twist_mux/cmd_vel
```


---------------


# Launch File Execution


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


# Test Configuration - Using Launch System

I could not get this to work.
As of chapter 5, `ros2 launch ...` has not been covered, and its likely my error.

```bash
# launch all nodes together
#ros2 launch manual_drive manual_drive_nodes.launch.py
ros2 launch manual_drive joy_nodes.launch.py
ros2 launch manual_drive all_nodes.launch.py
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
# OPEN A NEW TERMINAL - simplest way to get a clean shell environment without any previous ROS 2 environment variables
term-ros

# navigate to your workspace directory and remove previously build artifacts
cd ~/omni-robot
rm -rf ./build/ ./install/ ./log/

# source your base ROS 2 installation (if necessary)
source /opt/ros/jazzy/setup.bash

# resolve dependencies to ensure all package dependencies are met
rosdep install -i --from-path src/manual_drive --rosdistro jazzy -y   # <--- do this to build manual_drive package
rosdep install -i --from-path src --rosdistro jazzy -y                    # <--- only do this if you want to build ALL packages

# re-build the workspace
colcon build --packages-select manual_drive                           # <--- do this to build manual_drive package
colcon build                                                              # <--- only do this if you want to build ALL packages

# Source the workspace overlay to make the newly created packages available
source ~/omni-robot/install/setup.bash

# test the set of packages
colcon test --packages-select manual_drive                            # <--- to test just the manual_drive package
colcon test                                                               # <--- to test ALL packages
```


---------------
