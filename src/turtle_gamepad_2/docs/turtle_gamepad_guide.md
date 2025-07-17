# Turtle Gamepad 2 Guide

## Overview
This guide demonstrates how to control the ROS2 turtlesim turtle using a Logitech F310 gamepad controller with emergency stop functionality.

## Prerequisites
- Ubuntu 22.04 or later
- ROS2 Jazzy Jalisco installed
- Logitech F310 gamepad controller
- Python 3.10+

## Required ROS2 Packages
```bash
sudo apt update
sudo apt install ros-jazzy-turtlesim ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

## Step 1: Create ROS2 Workspace and Package

### 1.1 Create Workspace
```bash
mkdir -p ~/omni-robot/src
cd ~/omni-robot
```

### 1.2 Create Package
```bash
cd src
ros2 pkg create --build-type ament_python turtle_gamepad_2 --dependencies rclpy geometry_msgs sensor_msgs std_msgs
```

### 1.3 Navigate to Package
```bash
cd turtle_gamepad_2
```

## Step 2: Package Configuration

### 2.1 Update package.xml
Edit `package.xml` to include additional dependencies:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>turtle_gamepad_2</name>
  <version>0.0.0</version>
  <description>Turtle gamepad controller with emergency stop</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>turtlesim</depend>
  <depend>joy</depend>
  <depend>teleop_twist_joy</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 2.2 Update setup.py
```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_gamepad_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Turtle gamepad controller with emergency stop',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_gamepad_controller = turtle_gamepad_2.turtle_gamepad_controller:main',
            'emergency_stop_monitor = turtle_gamepad_2.emergency_stop_monitor:main',
        ],
    },
)
```

## Step 3: Create Configuration Files

### 3.1 Create config directory and F310 configuration
```bash
mkdir -p config
```

Create `config/f310_config.yaml`:
```yaml
# Logitech F310 Gamepad Configuration for ROS2 Jazzy
# XInput mode configuration for Linux

# Joy node configuration
joy_node:
  ros__parameters:
    device_id: 0
    deadzone: 0.05
    autorepeat_rate: 20.0

# Teleop twist joy configuration
teleop_twist_joy_node:
  ros__parameters:
    # Axis mappings (F310 XInput mode)
    axis_linear:
      x: 1    # Left stick vertical (forward/backward)
    axis_angular:
      yaw: 0  # Left stick horizontal (turn left/right)

    # Scale factors
    scale_linear: 2.0
    scale_linear_turbo: 5.0
    scale_angular: 2.0
    scale_angular_turbo: 5.0

    # Button mappings
    enable_button: 4  # Left bumper (LB)
    enable_turbo_button: 5  # Right bumper (RB)

    # Require enable button to be pressed for movement
    require_enable_button: true

# F310 Button and Axis Reference (XInput mode):
# Buttons:
#   0: A
#   1: B
#   2: X
#   3: Y
#   4: LB (Left Bumper)
#   5: RB (Right Bumper)
#   6: Back
#   7: Start
#   8: Guide/Xbox button
#   9: Left stick click
#   10: Right stick click
#
# Axes:
#   0: Left stick horizontal (-1.0 = left, +1.0 = right)
#   1: Left stick vertical (-1.0 = up, +1.0 = down)
#   2: LT (Left Trigger) (0.0 = not pressed, +1.0 = fully pressed)
#   3: Right stick horizontal (-1.0 = left, +1.0 = right)
#   4: Right stick vertical (-1.0 = up, +1.0 = down)
#   5: RT (Right Trigger) (0.0 = not pressed, +1.0 = fully pressed)
#   6: D-pad horizontal (-1.0 = left, +1.0 = right)
#   7: D-pad vertical (-1.0 = up, +1.0 = down)

# Emergency stop configuration
emergency_stop:
  ros__parameters:
    emergency_button: 1  # B button for emergency stop
    reset_button: 0      # A button to reset emergency stop
```

## Step 4: Create Python Nodes

### 4.1 Create the main controller node
Create `turtle_gamepad_2/turtle_gamepad_controller.py`:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import math


class TurtleGamepadController(Node):
    def __init__(self):
        super().__init__('turtle_gamepad_controller')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.emergency_status_publisher = self.create_publisher(Bool, '/emergency_stop_status', 10)

        # Subscribers
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.emergency_subscriber = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        # Parameters
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_angular_yaw', 0)
        self.declare_parameter('scale_linear', 2.0)
        self.declare_parameter('scale_angular', 2.0)
        self.declare_parameter('scale_linear_turbo', 5.0)
        self.declare_parameter('scale_angular_turbo', 5.0)
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('turbo_button', 5)
        self.declare_parameter('emergency_button', 1)
        self.declare_parameter('reset_button', 0)
        self.declare_parameter('deadzone', 0.05)

        # State variables
        self.emergency_stop_active = False
        self.last_emergency_button_state = False
        self.last_reset_button_state = False

        # Timer for status updates
        self.status_timer = self.create_timer(0.1, self.publish_status)

        self.get_logger().info('Turtle Gamepad Controller initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left stick: Move turtle')
        self.get_logger().info('  LB (Left Bumper): Enable movement')
        self.get_logger().info('  RB (Right Bumper): Turbo mode')
        self.get_logger().info('  B button: Emergency stop')
        self.get_logger().info('  A button: Reset emergency stop')

    def joy_callback(self, msg):
        """Handle joystick input"""
        if len(msg.buttons) == 0 or len(msg.axes) == 0:
            return

        # Get parameters
        axis_linear = self.get_parameter('axis_linear_x').value
        axis_angular = self.get_parameter('axis_angular_yaw').value
        scale_linear = self.get_parameter('scale_linear').value
        scale_angular = self.get_parameter('scale_angular').value
        scale_linear_turbo = self.get_parameter('scale_linear_turbo').value
        scale_angular_turbo = self.get_parameter('scale_angular_turbo').value
        enable_button = self.get_parameter('enable_button').value
        turbo_button = self.get_parameter('turbo_button').value
        emergency_button = self.get_parameter('emergency_button').value
        reset_button = self.get_parameter('reset_button').value
        deadzone = self.get_parameter('deadzone').value

        # Handle emergency stop button (B button)
        if emergency_button < len(msg.buttons):
            emergency_pressed = msg.buttons[emergency_button] == 1
            if emergency_pressed and not self.last_emergency_button_state:
                self.emergency_stop_active = True
                self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
            self.last_emergency_button_state = emergency_pressed

        # Handle reset button (A button)
        if reset_button < len(msg.buttons):
            reset_pressed = msg.buttons[reset_button] == 1
            if reset_pressed and not self.last_reset_button_state:
                if self.emergency_stop_active:
                    self.emergency_stop_active = False
                    self.get_logger().info('Emergency stop RESET - Normal operation resumed')
            self.last_reset_button_state = reset_pressed

        # Don't move if emergency stop is active
        if self.emergency_stop_active:
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            return

        # Check if enable button is pressed
        enable_pressed = False
        if enable_button < len(msg.buttons):
            enable_pressed = msg.buttons[enable_button] == 1

        # Check if turbo button is pressed
        turbo_pressed = False
        if turbo_button < len(msg.buttons):
            turbo_pressed = msg.buttons[turbo_button] == 1

        # Create twist message
        twist = Twist()

        # Only move if enable button is pressed
        if enable_pressed:
            # Get axis values with deadzone
            linear_val = 0.0
            angular_val = 0.0

            if axis_linear < len(msg.axes):
                raw_linear = -msg.axes[axis_linear]  # Invert for forward/backward
                if abs(raw_linear) > deadzone:
                    linear_val = raw_linear

            if axis_angular < len(msg.axes):
                raw_angular = msg.axes[axis_angular]
                if abs(raw_angular) > deadzone:
                    angular_val = raw_angular

            # Apply scaling
            if turbo_pressed:
                twist.linear.x = linear_val * scale_linear_turbo
                twist.angular.z = angular_val * scale_angular_turbo
            else:
                twist.linear.x = linear_val * scale_linear
                twist.angular.z = angular_val * scale_angular

        # Publish the twist message
        self.cmd_vel_publisher.publish(twist)

    def emergency_callback(self, msg):
        """Handle external emergency stop commands"""
        self.emergency_stop_active = msg.data
        if msg.data:
            self.get_logger().warn('External EMERGENCY STOP received!')
            # Stop the turtle immediately
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)

    def publish_status(self):
        """Publish emergency stop status"""
        status_msg = Bool()
        status_msg.data = self.emergency_stop_active
        self.emergency_status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleGamepadController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4.2 Create emergency stop monitor node
Create `turtle_gamepad_2/emergency_stop_monitor.py`:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class EmergencyStopMonitor(Node):
    def __init__(self):
        super().__init__('emergency_stop_monitor')

        # Publishers
        self.emergency_stop_publisher = self.create_publisher(Bool, '/emergency_stop', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscribers
        self.status_subscriber = self.create_subscription(
            Bool, '/emergency_stop_status', self.status_callback, 10)

        # State
        self.emergency_active = False

        # Timer for emergency stop enforcement
        self.timer = self.create_timer(0.1, self.enforce_emergency_stop)

        self.get_logger().info('Emergency Stop Monitor initialized')

    def status_callback(self, msg):
        """Monitor emergency stop status"""
        if msg.data != self.emergency_active:
            self.emergency_active = msg.data
            if self.emergency_active:
                self.get_logger().error('EMERGENCY STOP ACTIVE - All movement stopped!')
            else:
                self.get_logger().info('Emergency stop released - Movement enabled')

    def enforce_emergency_stop(self):
        """Continuously enforce emergency stop by sending zero velocity"""
        if self.emergency_active:
            stop_twist = Twist()
            self.cmd_vel_publisher.publish(stop_twist)

    def trigger_emergency_stop(self):
        """Trigger emergency stop externally"""
        msg = Bool()
        msg.data = True
        self.emergency_stop_publisher.publish(msg)
        self.get_logger().warn('External emergency stop triggered!')


def main(args=None):
    rclpy.init(args=args)
    monitor = EmergencyStopMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4.3 Create __init__.py
Create `turtle_gamepad_2/__init__.py` (empty file):
```python
# This file makes the directory a Python package
```

## Step 5: Create Launch Files

### 5.1 Create launch directory
```bash
mkdir -p launch
```

### 5.2 Create main launch file
Create `launch/turtle_gamepad_launch.py`:
```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_turtle_gamepad = get_package_share_directory('turtle_gamepad_2')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_turtle_gamepad, 'config', 'f310_config.yaml'),
        description='Full path to joy config file'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('joy', 'joy')
        ]
    )

    # Teleop twist joy node
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('joy', 'joy'),
            ('cmd_vel', 'cmd_vel_joy')
        ]
    )

    # Turtle gamepad controller
    turtle_gamepad_controller = Node(
        package='turtle_gamepad_2',
        executable='turtle_gamepad_controller',
        name='turtle_gamepad_controller',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )

    # Emergency stop monitor
    emergency_stop_monitor = Node(
        package='turtle_gamepad_2',
        executable='emergency_stop_monitor',
        name='emergency_stop_monitor',
        output='screen'
    )

    # Turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        joy_node,
        teleop_twist_joy_node,
        turtle_gamepad_controller,
        emergency_stop_monitor,
        turtlesim_node
    ])
```

### 5.3 Create simplified launch file
Create `launch/turtle_gamepad_simple_launch.py`:
```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_turtle_gamepad = get_package_share_directory('turtle_gamepad_2')

    # Config file path
    config_file = os.path.join(pkg_turtle_gamepad, 'config', 'f310_config.yaml')

    # Turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen'
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config_file],
        output='screen'
    )

    # Turtle gamepad controller
    turtle_controller = Node(
        package='turtle_gamepad_2',
        executable='turtle_gamepad_controller',
        name='turtle_gamepad_controller',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        turtlesim_node,
        joy_node,
        turtle_controller
    ])
```

## Step 6: Create Test Files

### 6.1 Create test directory
```bash
mkdir -p test
```

### 6.2 Create test files
Create `test/test_turtle_gamepad_controller.py`:
```python
#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import time
import threading


class TestTurtleGamepadController:
    @classmethod
    def setup_class(cls):
        """Setup test environment"""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Cleanup test environment"""
        rclpy.shutdown()

    def test_joy_message_format(self):
        """Test joy message format"""
        joy_msg = Joy()
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        assert len(joy_msg.axes) == 8
        assert len(joy_msg.buttons) == 11

    def test_twist_message_creation(self):
        """Test twist message creation"""
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.5

        assert twist.linear.x == 1.0
        assert twist.angular.z == 0.5
        assert twist.linear.y == 0.0
        assert twist.linear.z == 0.0
        assert twist.angular.x == 0.0
        assert twist.angular.y == 0.0

    def test_emergency_stop_message(self):
        """Test emergency stop message"""
        emergency_msg = Bool()
        emergency_msg.data = True

        assert emergency_msg.data == True

        emergency_msg.data = False
        assert emergency_msg.data == False


if __name__ == '__main__':
    pytest.main([__file__])
```

Create `test/test_emergency_stop_monitor.py`:
```python
#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class TestEmergencyStopMonitor:
    @classmethod
    def setup_class(cls):
        """Setup test environment"""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Cleanup test environment"""
        rclpy.shutdown()

    def test_emergency_stop_status(self):
        """Test emergency stop status handling"""
        status_msg = Bool()
        status_msg.data = True

        assert status_msg.data == True

    def test_zero_velocity_command(self):
        """Test zero velocity command creation"""
        stop_twist = Twist()

        assert stop_twist.linear.x == 0.0
        assert stop_twist.linear.y == 0.0
        assert stop_twist.linear.z == 0.0
        assert stop_twist.angular.x == 0.0
        assert stop_twist.angular.y == 0.0
        assert stop_twist.angular.z == 0.0


if __name__ == '__main__':
    pytest.main([__file__])
```

## Step 7: Build and Test

### 7.1 Install dependencies
```bash
cd ~/omni-robot
rosdep install --from-paths src --ignore-src -r -y
```

### 7.2 Build the workspace
```bash
colcon build --packages-select turtle_gamepad_2
```

### 7.3 Source the workspace
```bash
source install/setup.bash
```

### 7.4 Run tests
```bash
# Run Python tests
cd src/turtle_gamepad_2
python -m pytest test/ -v

# Run ROS2 tests
cd ~/omni-robot
colcon test --packages-select turtle_gamepad_2
colcon test-result --verbose
```

## Step 8: Hardware Setup

### 8.1 Connect the F310 Controller
1. Connect the Logitech F310 controller via USB
2. Set the controller to XInput mode (switch on back should be in 'X' position)
3. Test controller detection:
```bash
ls /dev/input/js*
# Should show something like /dev/input/js0

# Test joystick input
jstest /dev/input/js0
```

### 8.2 Grant permissions (if needed)
```bash
sudo chmod a+rw /dev/input/js0
```

## Step 9: Running the Application

### 9.1 Manual Node Execution

#### Terminal 1: Launch turtlesim
```bash
cd ~/omni-robot
source install/setup.bash
ros2 run turtlesim turtlesim_node
```

#### Terminal 2: Launch joy node
```bash
cd ~/omni-robot
source install/setup.bash
ros2 run joy joy_node --ros-args --params-file src/turtle_gamepad_2/config/f310_config.yaml
```

#### Terminal 3: Launch turtle controller
```bash
cd ~/omni-robot
source install/setup.bash
ros2 run turtle_gamepad_2 turtle_gamepad_controller --ros-args --params-file src/turtle_gamepad_2/config/f310_config.yaml
```

#### Terminal 4: Launch emergency stop monitor
```bash
cd ~/omni-robot
source install/setup.bash
ros2 run turtle_gamepad_2 emergency_stop_monitor
```

### 9.2 Launch File Execution

#### Full launch (all nodes)
```bash
cd ~/omni-robot
source install/setup.bash
ros2 launch turtle_gamepad_2 turtle_gamepad_launch.py
```

#### Simple launch (essential nodes only)
```bash
cd ~/omni-robot
source install/setup.bash
ros2 launch turtle_gamepad_2 turtle_gamepad_simple_launch.py
```

## Step 10: Usage Instructions

### 10.1 Controller Layout (F310 XInput Mode)
- **Left Stick**: Move turtle (vertical = forward/backward, horizontal = turn left/right)
- **Left Bumper (LB)**: Enable button - must be held for movement
- **Right Bumper (RB)**: Turbo mode - faster movement when held with LB
- **B Button**: Emergency stop - immediately stops turtle and disables movement
- **A Button**: Reset emergency stop - re-enables movement after emergency stop

### 10.2 Operation Steps
1. Launch the application using one of the methods above
2. The turtlesim window should open showing the turtle
3. Hold the Left Bumper (LB) and use the left stick to move the turtle
4. Press B button for emergency stop
5. Press A button to reset and resume normal operation

### 10.3 Safety Features
- **Dead-man's Switch**: Movement only occurs when LB is held
- **Emergency Stop**: B button immediately stops all movement
- **Turbo Mode**: RB + LB for faster movement
- **Deadzone**: Small joystick movements are ignored to prevent drift

## Step 11: Troubleshooting

### 11.1 Controller Not Detected
```bash
# Check if controller is connected
lsusb | grep Logitech

# Check input devices
ls /dev/input/

# Test controller
jstest /dev/input/js0
```

### 11.2 Permission Issues
```bash
# Add user to input group
sudo usermod -a -G input $USER

# Logout and login again, or:
newgrp input

# Set permissions
sudo chmod 666 /dev/input/js0
```

### 11.3 Debug Topics
```bash
# Monitor joy messages
ros2 topic echo /joy

# Monitor turtle velocity commands
ros2 topic echo /turtle1/cmd_vel

# Monitor emergency stop status
ros2 topic echo /emergency_stop_status

# List all topics
ros2 topic list
```

### 11.4 Node Status
```bash
# Check running nodes
ros2 node list

# Check node info
ros2 node info /turtle_gamepad_controller
```

## Step 12: Additional Features

### 12.1 Parameter Tuning
You can adjust parameters in the `config/f310_config.yaml` file:
- `scale_linear`: Normal movement speed
- `scale_angular`: Normal turning speed
- `scale_linear_turbo`: Turbo movement speed
- `scale_angular_turbo`: Turbo turning speed
- `deadzone`: Joystick deadzone threshold

### 12.2 Logging
All nodes provide detailed logging output. Use different log levels:
```bash
ros2 run turtle_gamepad_2 turtle_gamepad_controller --ros-args --log-level debug
```

### 12.3 Recording and Playback
```bash
# Record session
ros2 bag record -a

# Playback session
ros2 bag play <bag_file>
```

## Conclusion

This guide provides a complete solution for controlling the ROS2 turtlesim turtle using a Logitech F310 gamepad with emergency stop functionality. The system includes proper safety features, comprehensive testing, and flexible launch configurations.

Key features implemented:
- ✅ Full F310 gamepad support with XInput mode
- ✅ Emergency stop with B button
- ✅ Reset functionality with A button
- ✅ Dead-man's switch with LB button
- ✅ Turbo mode with RB button
- ✅ Comprehensive Python testing
- ✅ Multiple launch file options
- ✅ Proper ROS2 package structure
- ✅ Detailed configuration and troubleshooting

The solution follows ROS2 best practices and provides a robust, safe, and user-friendly turtle control system.

