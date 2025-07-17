<!--
Maintainer:   jeffskinnerbox@yahoo.com / www.jeffskinnerbox.me
Version:      1.1.0
-->

<div align="center">
<img src="https://raw.githubusercontent.com/jeffskinnerbox/blog/main/content/images/banners-bkgrds/work-in-progress.jpg"
    title="These materials require additional work and are not ready for general use." align="center" width=420px height=219px>
</div>

<div align="center">
<img src="https://raw.githubusercontent.com/jeffskinnerbox/blog/main/content/images/banners-bkgrds/we-do-this-not-because-it-is-easy.jpg"
    title="U.S. President John F. Kennedy said this in his moon speech at Rice University in Houston, Texas, on September 12, 1962:
    We choose to go to the moon. We choose to go to the moon in this decade and do the other things, not because they are easy,
    but because they are hard, because that goal will serve to organize and measure the best of our energies and skills,
    because that challenge is one that we are willing to accept, one we are unwilling to postpone, and one which we intend to win,
    and the others, too." align="center" width=500px height=375px>
</div>

---------------

This package enables control of the robot simultaneous via multiple user interface devices.
This includes the keyboard, game controller, and emergency stop button.
The architecture allows for the expansion to additional hardware devices (e.g mobile phone, website, etc.).

When writing prompts for ROS2 Python code assistance, here are key strategies to get the most helpful responses:

1. Be Specific About Your ROS2 Context
Include essential details like:

* ROS2 distribution (Humble, Iron, Jazzy, etc.)
* Package structure and dependencies
* Node types you're working with (publisher, subscriber, service, action server/client)
* Message types and interfaces you need to use
* Whether you're using rclpy, lifecycle nodes, or other specific ROS2 concepts

1. Provide Clear Requirements
Instead of "help me with a ROS2 node," try:

* "Create a ROS2 publisher node that publishes geometry_msgs/Twist messages to /cmd_vel topic at 10Hz"
* "I need a service client that calls a custom service to process laser scan data and returns filtered points"
* "Build an action server for navigation that accepts goal poses and provides feedback on progress"

1. Include Relevant Code Context
When asking for modifications or debugging help:

* Share your existing code structure
* Mention your package.xml and setup.py dependencies
* Include any error messages you're encountering
* Specify which parts are working vs. problematic

1. Specify Integration Needs
Mention how your code fits into the larger system:

* Launch file requirements
* Parameter handling needs
* Integration with existing packages like nav2, tf2, etc.
* Real-time constraints or performance requirements

1. Ask for Complete Examples
Request functional, runnable code rather than snippets:

* "Provide a complete ROS2 package structure with proper setup.py and package.xml"
* "Include proper error handling and logging"
* "Show how to handle node shutdown gracefully"

1. Be Clear About Style and Practices
Specify preferences for:

* Code organization (single file vs. class-based)
* ROS2 best practices you want followed
* Testing requirements
* Documentation level needed

This approach will help you get more targeted, functional code that integrates well with ROS2 conventions and your specific project needs.

---------------

Claude Prompt - Turtle Gamepad Controller 3:

  _My goal is to using a game controller, Python, and ROS2 Jazzy Jalisco
  to control the movement of the turtle within the ROS2 turtlesim node.
  In the solution, use the ros-jazzy-turtlesim, ros-jazzy-joy.
  Provide me guidance on any additional ROS2 nodes I should use
  and how I should configure them to control the turtlesim node turtle.__

  _The human operator will control the movement of turtle using a Logitech F310 Wired Gamepad Controller
  where one of the F310 buttons is an emergency stop button.
  Do not use a keyboard for any control.
  For a standard Logitech F310 in XInput mode on Linux, provided a complete YAML configuration file
  mapping all the F310 joysticks and buttons._

  _Give me a step-by-step methodology for creating a solution, written in Python, for ROS2 Jazzy.
  Make use of the ROS2 tools "colcon build" and "rosdep", make use of Python tools "pytest" for testing,
  and show how to execute the ROS2 nodes manually and by using "launch"._

  _I want to name the ROS2 workspace "omni-robot" and name the package "turtle_gamepad_3".
  Title the solution guide you create "Turtle Gamepad 3 Guide".
  Create a launch files for all nodes._

"Turtle Gamepad 3 Guide" has been published publicly on [Anthropic Claude](https://claude.ai/public/artifacts/3e880985-ebcf-4c6b-8b26-eed279ae67e9)

---------------

# Teleoperation

Teleoperation in Robotics refers to controlling a robot from a distance, aka remote control.
This is in contrast to autonomous operation, where the robot is given a task which it goes off and completes itself.
Remotely controlling a robot, allowing a human operator to influence the robot's actions from a distance,
involves sending commands to the robot (e.g., movement instructions, sensor data requests) and receiving feedback
(e.g., camera images, sensor readings) from it, effectively making the robot an extension of the operator.

* [Add TeleOp to your ROS2 Robot](https://medium.com/@peter.gaston/add-teleop-to-your-ros2-robot-5b7b0a5606ce)
* [Teleoperation](https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/teleop/#bonus-a-nicer-way-to-check-joysticks)

----

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

1. Add Python Libraries to Your Package:

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

1. Develop Python Nodes:

* Create a Python node file: Add your Python code to a file within your package's source directory (e.g., `my_python_package/my_node.py`).
* Include required libraries: Import `rclpy` and other necessary libraries.
* Create a ROS 2 node: Instantiate a ROS 2 node using `rclpy.node.Node()` within your Python script.
* Define entry points in `setup.py`: Add a `console_scripts` entry in your `setup.py` file
    to make your Python script an executable ROS 2 node. This allows you to run it using `ros2 run`.

1. Build and Run Your Package:

* Build the workspace: Navigate to your workspace root and use colcon build to build your package and its dependencies.
* Source the setup file: After building, source the workspace's setup file (e.g., source install/local_setup.bash)
    in a new terminal to make the installed packages available.
* Run the node: Execute your Python node using ros2 run <package_name> <node_name>.

1. Important Notes:

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

1. Install rosdep:

* If you installed ROS2 Jazzy through binary packages, `rosdep` is likely included.
* Otherwise, you can install it using your system's package manager, for example, on Debian/Ubuntu: `sudo apt install python3-rosdep`.

1. Initialize and Update rosdep:

* If this is the first time using `rosdep`, you need to initialize it and update the local database:

    ```bash
    sudo rosdep init
    rosdep update
    ```

* `rosdep init` may fail if already initialized, which can be safely ignored.
* `rosdep update` refreshes the local copy of the `rosdistro` index, which contains information about available ROS packages and their dependencies.

1. Install Dependencies for Your Workspace:

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

1. Adding Rosdep Keys to `package.xml`:

* If your package depends on an external library that isn't a ROS package,
    find the corresponding `rosdep` key (e.g., searching in `rosdep/base.yaml` or `rosdep/python.yaml` in the rosdistro repository).
* Add this key to your package's `package.xml` file using appropriate tags like `<depend>`, `<build_depend>`, `<exec_depend>`, etc.

1. Troubleshooting:

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

>**NOTE:** `evtest` is another Linux command-line tool used to display and monitor events generated by input devices.
>It it to useful for debugging input devices like joysticks, gamepads, keyboards, mice, and touchpads
>by showing the raw event data being sent to the system.

The code block below outlines the how I performed the testing and where it failed:

```bash
# this works fine - OK
jstest /dev/input/js0

# see the same data in a graphical format

# source your base ROS 2 installation (if necessary)
source /opt/ros/jazzy/setup.bash

# then started joy_node connection to game controller (it appeared) - OK
ros2 run joy joy_node

# but this had no output from the joystick - Failure
ros2 topic echo /joy

# to see graphical display of all gamepad buttons / joysticks
ros2 run joy_tester test_joy

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

## Configure Gamepad Permissions

```bash
# add user to input group for gamepad access
sudo usermod -a -G input $USER

# what group is the gamepad (/dev/input/js0 and /dev/input/event*) associated with
$ ls -l /dev/input/
total 0
drwxr-xr-x  2 root root      80 Jul  9 09:41 by-id/
drwxr-xr-x  2 root root     120 Jul  9 09:41 by-path/
crw-rw----  1 root input 13, 64 Jul  9 09:41 event0
crw-rw----  1 root input 13, 65 Jul  9 09:41 event1
crw-rw----  1 root input 13, 66 Jul  9 09:41 event2
crw-rw----+ 1 root input 13, 67 Jul  9 09:41 event3
crw-rw----  1 root input 13, 68 Jul  9 09:41 event4
crw-rw----  1 root input 13, 69 Jul  9 09:41 event5
crw-rw----  1 root input 13, 70 Jul  9 09:41 event6
crw-rw----  1 root input 13, 71 Jul  9 09:41 event7
crw-rw----  1 root input 13, 72 Jul  9 09:41 event8
crw-rw-r--+ 1 root input 13,  0 Jul  9 09:41 js0
crw-rw----  1 root input 13, 63 Jul  9 09:41 mice

# get the vendor and product id for the gamepad
$ lsusb | grep Logitech
Bus 009 Device 002: ID 046d:c21d Logitech, Inc. F310 Gamepad [XInput Mode]

# create udev rule for F310 gamepad
sudo tee /etc/udev/rules.d/99-f310-gamepad.rules << 'EOF'
SUBSYSTEM=="input", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c21d", MODE="0666", GROUP="input"
EOF

# reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# log out and log back in for group changes to take effect
```

---------------

# Mixed use of `apt` and `conda`

Gemini Prompt: _Can the mixed use of apt and conda in ROS2 cause problems?_

Mixing apt and conda without careful consideration can create compatibility problems in ROS2.

[RoboStack](https://robostack.github.io/index.html) provides a way to use ROS2 alongside the conda and Jupyter ecosystems,
which might help manage dependencies and avoid conflicts.

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

* Set up a workspace: If you don't have a ROS 2 workspace, create one using `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws`.
* Create the package structure: In your workspace's `src` directory,
    use `ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy`.

# Build / Rebuild your ROS2 Workspace

By following these steps, you will effectively clean out old build artifacts and rebuild your workspace,
ensuring that your ROS2 Jazzy Python environment is fresh and properly configured.

1. To create a new workspace, do the following:

```bash
# create the required directory structure
mkdir -p ~/omni-robot/src
cd ~/omni-robot/src

# Create the required package structure in your workspace
ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy
```

1. To rebuild an existing workspace, do the following:

```bash
# OPEN A NEW TERMINAL - simplest way to get a clean shell environment without any previous ROS 2 environment variables
term-ros

# navigate to your workspace directory and remove previously build artifacts
cd ~/omni-robot
rm -rf ./build/ ./install/ ./log/

# source your base ROS 2 installation (if necessary)
source /opt/ros/jazzy/setup.bash

# resolve dependencies to ensure all package dependencies are met
rosdep install --from-path src/turtle_gamepad_3 --rosdistro jazzy -r -y   # <--- do this to build turtle_gamepad_3 package
rosdep install --from-path src --rosdistro jazzy -r -y                    # <--- only do this if you want to build ALL packages

# re-build the workspace
colcon build --packages-select turtle_gamepad_3                           # <--- do this to build turtle_gamepad_3 package
colcon build                                                              # <--- only do this if you want to build ALL packages

# Source the workspace overlay to make the newly created packages available
source ~/omni-robot/install/setup.bash

# test the set of packages
colcon test --packages-select turtle_gamepad_3                            # <--- to test just the turtle_gamepad_3 package
colcon test                                                               # <--- to test ALL packages

# inspect the test results
colcon test-result --all                                                  # view summary of test results
colcon test-result --all --verbose                                        # view detailed information for each error or failure

# Run specific test
python3 -m pytest src/turtle_gamepad_3/test/test_pep257.py -v
python3 -m pytest src/turtle_gamepad_3/test/test_flake8.py -v
python3 -m pytest src/turtle_gamepad_3/test/test_copyright.py -v
python3 -m pytest src/turtle_gamepad_3/test/test_integration.py -v
python3 -m pytest src/turtle_gamepad_3/test/test_turtle_controller.py -v
```

---------------

# Test Configuration - Manual Execution

I used the code block below to setup my test of the solution.
Each line was initiated in a separate terminal so I could
control & observe what was going on.

```bash
# in the initial terminal, source the workspace
source /opt/ros/jazzy/setup.bash
rqt_graph &                              # graphical representation of the node & topic architecture

# OPEN A NEW TERMINAL - simplest way to get a clean shell environment without any previous ROS 2 environment variables
# now create the remaining terminals from within this initial terminal above
term-ros

# terminal 1: Start turtlesim
cd ~/omni-robot
source install/setup.bash
ros2 run turtlesim turtlesim_node

# terminal 2: Start joy node
cd ~/omni-robot
source install/setup.bash
#ros2 run joy joy_node --ros-args --params-file src/turtle_gamepad_3/config/f310_config.yaml
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0

# terminal 3: Start turtle controller
cd ~/omni-robot
source install/setup.bash
#ros2 run turtle_gamepad_3 turtle_gamepad_controller --ros-args --params-file src/turtle_gamepad_3/config/f310_config.yaml
ros2 run turtle_gamepad_3 turtle_controller

# terminal 4: Monitor topics (optional)
ros2 topic echo /joy
ros2 topic echo /turtle_cmd_vel
```

---------------

# Test Configuration - Using Launch System

```bash
# launch all nodes general operation
cd ~/omni-robot
source install/setup.bash
ros2 launch turtle_gamepad_3 turtle_gamepad.launch.py

# launch all nodes together for debugging
cd ~/omni-robot
source install/setup.bash
ros2 launch turtle_gamepad_3 debug.launch.py

# launch all nodes together for testing
cd ~/omni-robot
source install/setup.bash
ros2 launch turtle_gamepad_3 test.launch.py

# launch with custom parameters
ros2 launch turtle_gamepad_3 turtle_gamepad_launch.py gamepad_device:=/dev/input/js0 turtle_name:=BigTurtle
```

----
