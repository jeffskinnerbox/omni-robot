
# Turtle Gamepad 3 Guide

## Overview
This guide provides a complete solution for controlling ROS2 turtlesim using a Logitech F310 gamepad controller with Python and ROS2 Jazzy Jalisco. The solution includes emergency stop functionality and comprehensive testing.

## Required ROS2 Packages
- `ros-jazzy-turtlesim` - Turtle simulation environment
- `ros-jazzy-joy` - Joystick/gamepad input handling
- `ros-jazzy-geometry-msgs` - Twist message types for movement commands

## System Requirements
- Ubuntu 22.04 or compatible Linux distribution
- ROS2 Jazzy Jalisco installed
- Logitech F310 gamepad controller
- Python 3.10+

## Step-by-Step Methodology

### Step 1: Create ROS2 Workspace and Package

```bash
# Create workspace
mkdir -p ~/omni-robot/src
cd ~/omni-robot

# Create package
cd src
ros2 pkg create --build-type ament_python turtle_gamepad_3 --dependencies rclpy geometry_msgs sensor_msgs std_msgs

# Navigate to package directory
cd turtle_gamepad_3
```

### Step 2: Install Dependencies

```bash
# Install system dependencies
sudo apt update
sudo apt install ros-jazzy-turtlesim ros-jazzy-joy python3-pytest

# Install rosdep dependencies
cd ~/omni-robot
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 3: Configure Gamepad Permissions

```bash
# Add user to input group for gamepad access
sudo usermod -a -G input $USER

# Create udev rule for F310 gamepad
sudo tee /etc/udev/rules.d/99-f310-gamepad.rules << 'EOF'
SUBSYSTEM=="input", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c21d", MODE="0666", GROUP="input"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Log out and log back in for group changes to take effect
```

### Step 4: Create Configuration Files

#### A. Gamepad Configuration (config/f310_config.yaml)

```yaml
# Logitech F310 Gamepad Configuration for XInput mode on Linux
# This configuration maps all F310 inputs for turtle control

gamepad_config:
  # Movement controls
  linear_axis: 1      # Left stick Y-axis (forward/backward)
  angular_axis: 0     # Left stick X-axis (left/right turn)

  # Speed control
  turbo_button: 5     # Right bumper (RB) - increase speed
  slow_button: 4      # Left bumper (LB) - decrease speed

  # Emergency controls
  emergency_stop_button: 6  # Back button - emergency stop
  reset_button: 7           # Start button - reset turtle position

  # Alternative movement (right stick)
  alt_linear_axis: 4   # Right stick Y-axis
  alt_angular_axis: 3  # Right stick X-axis

  # Speed scaling factors
  max_linear_speed: 2.0   # Maximum linear velocity
  max_angular_speed: 2.0  # Maximum angular velocity
  speed_increment: 0.1    # Speed adjustment increment

  # Deadzone settings
  deadzone_threshold: 0.1  # Ignore small stick movements

  # Button mappings (for reference)
  button_map:
    A: 0          # A button
    B: 1          # B button
    X: 2          # X button
    Y: 3          # Y button
    LB: 4         # Left bumper
    RB: 5         # Right bumper
    BACK: 6       # Back/Select button
    START: 7      # Start button
    GUIDE: 8      # Xbox/Guide button
    LS: 9         # Left stick click
    RS: 10        # Right stick click

  # Axis mappings (for reference)
  axis_map:
    LS_X: 0       # Left stick X-axis
    LS_Y: 1       # Left stick Y-axis
    LT: 2         # Left trigger
    RS_X: 3       # Right stick X-axis
    RS_Y: 4       # Right stick Y-axis
    RT: 5         # Right trigger
    DPAD_X: 6     # D-pad X-axis
    DPAD_Y: 7     # D-pad Y-axis
```

#### B. Launch Configuration (launch/turtle_gamepad_launch.py)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('turtle_gamepad_3')
    config_file = os.path.join(pkg_dir, 'config', 'f310_config.yaml')

    # Launch arguments
    gamepad_device_arg = DeclareLaunchArgument(
        'gamepad_device',
        default_value='/dev/input/js0',
        description='Gamepad device path'
    )

    turtle_name_arg = DeclareLaunchArgument(
        'turtle_name',
        default_value='turtle1',
        description='Name of the turtle to control'
    )

    # Nodes
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('gamepad_device'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }],
        output='screen'
    )

    turtle_controller_node = Node(
        package='turtle_gamepad_3',
        executable='turtle_controller',
        name='turtle_controller',
        parameters=[
            config_file,
            {'turtle_name': LaunchConfiguration('turtle_name')}
        ],
        output='screen'
    )

    return LaunchDescription([
        gamepad_device_arg,
        turtle_name_arg,
        turtlesim_node,
        joy_node,
        turtle_controller_node
    ])
```

### Step 5: Create Python Controller Node

#### A. Main Controller (turtle_gamepad_3/turtle_controller.py)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from std_srvs.srv import Empty
import math
import time

class TurtleGamepadController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # Parameters
        self.declare_parameter('turtle_name', 'turtle1')
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('turbo_button', 5)
        self.declare_parameter('slow_button', 4)
        self.declare_parameter('emergency_stop_button', 6)
        self.declare_parameter('reset_button', 7)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('speed_increment', 0.1)
        self.declare_parameter('deadzone_threshold', 0.1)

        # Get parameters
        self.turtle_name = self.get_parameter('turtle_name').value
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.turbo_button = self.get_parameter('turbo_button').value
        self.slow_button = self.get_parameter('slow_button').value
        self.emergency_stop_button = self.get_parameter('emergency_stop_button').value
        self.reset_button = self.get_parameter('reset_button').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.deadzone_threshold = self.get_parameter('deadzone_threshold').value

        # State variables
        self.emergency_stopped = False
        self.current_linear_scale = 1.0
        self.current_angular_scale = 1.0
        self.last_button_state = {}
        self.last_emergency_press = 0

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.turtle_name}/cmd_vel',
            10
        )

        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # Service clients
        self.teleport_abs_client = self.create_client(
            TeleportAbsolute,
            f'/{self.turtle_name}/teleport_absolute'
        )

        self.clear_client = self.create_client(
            Empty,
            '/clear'
        )

        self.get_logger().info(f'Turtle Gamepad Controller initialized for {self.turtle_name}')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left stick: Move turtle')
        self.get_logger().info('  RB (Right Bumper): Turbo mode')
        self.get_logger().info('  LB (Left Bumper): Slow mode')
        self.get_logger().info('  BACK button: Emergency stop')
        self.get_logger().info('  START button: Reset turtle position')

    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone_threshold:
            return 0.0
        return value

    def joy_callback(self, msg):
        """Handle joystick input"""
        try:
            # Handle emergency stop
            if len(msg.buttons) > self.emergency_stop_button:
                if msg.buttons[self.emergency_stop_button] == 1:
                    current_time = time.time()
                    if current_time - self.last_emergency_press > 0.5:  # Debounce
                        self.emergency_stopped = not self.emergency_stopped
                        self.last_emergency_press = current_time
                        if self.emergency_stopped:
                            self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
                            self.stop_turtle()
                        else:
                            self.get_logger().info('Emergency stop deactivated')

            # Handle reset
            if len(msg.buttons) > self.reset_button:
                if (msg.buttons[self.reset_button] == 1 and
                    self.last_button_state.get(self.reset_button, 0) == 0):
                    self.reset_turtle()

            # Handle speed controls
            if len(msg.buttons) > self.turbo_button:
                if (msg.buttons[self.turbo_button] == 1 and
                    self.last_button_state.get(self.turbo_button, 0) == 0):
                    self.current_linear_scale = min(2.0, self.current_linear_scale + self.speed_increment)
                    self.current_angular_scale = min(2.0, self.current_angular_scale + self.speed_increment)
                    self.get_logger().info(f'Speed increased: {self.current_linear_scale:.1f}x')

            if len(msg.buttons) > self.slow_button:
                if (msg.buttons[self.slow_button] == 1 and
                    self.last_button_state.get(self.slow_button, 0) == 0):
                    self.current_linear_scale = max(0.1, self.current_linear_scale - self.speed_increment)
                    self.current_angular_scale = max(0.1, self.current_angular_scale - self.speed_increment)
                    self.get_logger().info(f'Speed decreased: {self.current_linear_scale:.1f}x')

            # Store button states for edge detection
            self.last_button_state = {i: button for i, button in enumerate(msg.buttons)}

            # Don't move if emergency stopped
            if self.emergency_stopped:
                return

            # Process movement
            if len(msg.axes) > max(self.linear_axis, self.angular_axis):
                linear_input = self.apply_deadzone(msg.axes[self.linear_axis])
                angular_input = self.apply_deadzone(msg.axes[self.angular_axis])

                # Create and publish twist message
                twist = Twist()
                twist.linear.x = linear_input * self.max_linear_speed * self.current_linear_scale
                twist.angular.z = -angular_input * self.max_angular_speed * self.current_angular_scale

                self.cmd_vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Error in joy_callback: {str(e)}')

    def stop_turtle(self):
        """Stop the turtle immediately"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def reset_turtle(self):
        """Reset turtle to center position"""
        self.get_logger().info('Resetting turtle position...')

        # Clear the screen
        if self.clear_client.wait_for_service(timeout_sec=1.0):
            clear_request = Empty.Request()
            self.clear_client.call_async(clear_request)

        # Teleport to center
        if self.teleport_abs_client.wait_for_service(timeout_sec=1.0):
            teleport_request = TeleportAbsolute.Request()
            teleport_request.x = 5.544444561
            teleport_request.y = 5.544444561
            teleport_request.theta = 0.0
            self.teleport_abs_client.call_async(teleport_request)
            self.get_logger().info('Turtle reset to center position')
        else:
            self.get_logger().warn('Teleport service not available')

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

### Step 6: Create Package Structure

```bash
# Create necessary directories
cd ~/omni-robot/src/turtle_gamepad_3
mkdir -p config launch test

# Create __init__.py files
touch turtle_gamepad_3/__init__.py
touch test/__init__.py
```

### Step 7: Update Package Configuration

#### A. Update setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'turtle_gamepad_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Turtle gamepad controller for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_controller = turtle_gamepad_3.turtle_controller:main',
        ],
    },
)
```

#### B. Update package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>turtle_gamepad_3</name>
  <version>0.0.0</version>
  <description>Turtle gamepad controller for ROS2 turtlesim</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>turtlesim</depend>
  <depend>joy</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 8: Create Test Files

#### A. Unit Tests (test/test_turtle_controller.py)

```python
#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from turtle_gamepad_3.turtle_controller import TurtleGamepadController
import time

class TestTurtleController:
    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.shutdown()

    def test_controller_initialization(self):
        """Test that the controller initializes properly"""
        controller = TurtleGamepadController()
        assert controller.turtle_name == 'turtle1'
        assert controller.emergency_stopped == False
        controller.destroy_node()

    def test_deadzone_functionality(self):
        """Test deadzone application"""
        controller = TurtleGamepadController()

        # Test values within deadzone
        assert controller.apply_deadzone(0.05) == 0.0
        assert controller.apply_deadzone(-0.05) == 0.0

        # Test values outside deadzone
        assert controller.apply_deadzone(0.5) == 0.5
        assert controller.apply_deadzone(-0.5) == -0.5

        controller.destroy_node()

    def test_emergency_stop_functionality(self):
        """Test emergency stop state changes"""
        controller = TurtleGamepadController()

        # Create mock joy message
        joy_msg = Joy()
        joy_msg.buttons = [0] * 8  # 8 buttons
        joy_msg.axes = [0.0] * 8   # 8 axes

        # Test emergency stop activation
        initial_state = controller.emergency_stopped
        joy_msg.buttons[controller.emergency_stop_button] = 1

        # Simulate button press
        controller.joy_callback(joy_msg)
        time.sleep(0.1)  # Small delay

        # Note: Emergency stop toggles, so we test the toggle behavior
        assert controller.emergency_stopped != initial_state

        controller.destroy_node()

def test_gamepad_config_loading():
    """Test that gamepad configuration can be loaded"""
    # This would test loading the YAML config
    # In a real scenario, you'd test parameter loading
    pass

if __name__ == '__main__':
    pytest.main([__file__])
```

#### B. Integration Tests (test/test_integration.py)

```python
#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
import threading

class TestIntegration:
    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.shutdown()

    def test_joy_to_twist_conversion(self):
        """Test that joystick input generates correct twist messages"""
        from turtle_gamepad_3.turtle_controller import TurtleGamepadController

        controller = TurtleGamepadController()

        # Create a subscriber to capture published twist messages
        received_messages = []

        def twist_callback(msg):
            received_messages.append(msg)

        twist_sub = controller.create_subscription(
            Twist,
            f'/{controller.turtle_name}/cmd_vel',
            twist_callback,
            10
        )

        # Create mock joy message
        joy_msg = Joy()
        joy_msg.buttons = [0] * 8
        joy_msg.axes = [0.0] * 8

        # Set joystick values
        joy_msg.axes[controller.linear_axis] = 0.5  # Forward
        joy_msg.axes[controller.angular_axis] = 0.3  # Turn right

        # Process the message
        controller.joy_callback(joy_msg)

        # Spin briefly to process messages
        rclpy.spin_once(controller, timeout_sec=0.1)

        # Check that a twist message was published
        assert len(received_messages) > 0
        last_twist = received_messages[-1]

        # Check that the values are reasonable
        assert last_twist.linear.x > 0  # Moving forward
        assert last_twist.angular.z != 0  # Turning

        controller.destroy_node()

if __name__ == '__main__':
    pytest.main([__file__])
```

### Step 9: Build and Test

```bash
# Navigate to workspace root
cd ~/omni-robot

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-select turtle_gamepad_3

# Source the setup file
source install/setup.bash

# Run tests
colcon test --packages-select turtle_gamepad_3
pytest src/turtle_gamepad_3/test/
```

### Step 10: Manual Execution

#### A. Test Gamepad Connection

```bash
# Check if gamepad is detected
ls /dev/input/js*

# Test gamepad input
jstest /dev/input/js0
```

#### B. Run Individual Nodes

```bash
# Terminal 1: Run turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run joy node
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0

# Terminal 3: Run turtle controller
ros2 run turtle_gamepad_3 turtle_controller

# Terminal 4: Monitor topics (optional)
ros2 topic echo /joy
ros2 topic echo /turtle1/cmd_vel
```

### Step 11: Launch File Execution

```bash
# Launch all nodes together
ros2 launch turtle_gamepad_3 turtle_gamepad_launch.py

# Launch with custom parameters
ros2 launch turtle_gamepad_3 turtle_gamepad_launch.py gamepad_device:=/dev/input/js1 turtle_name:=turtle2
```

### Step 12: Additional Launch Files

#### A. Testing Launch File (launch/test_launch.py)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Run tests
        ExecuteProcess(
            cmd=['python3', '-m', 'pytest', 'src/turtle_gamepad_3/test/', '-v'],
            cwd=os.getcwd(),
            output='screen'
        )
    ])
```

#### B. Debug Launch File (launch/debug_launch.py)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting turtle gamepad debug session..."),

        # Gamepad test
        ExecuteProcess(
            cmd=['jstest', '/dev/input/js0'],
            output='screen',
            prefix='xterm -e'
        ),

        # Topic monitoring
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/joy'],
            output='screen',
            prefix='xterm -e'
        ),

        # Turtle velocity monitoring
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/turtle1/cmd_vel'],
            output='screen',
            prefix='xterm -e'
        )
    ])
```

## Usage Instructions

1. **Connect the F310 gamepad** and ensure it's in XInput mode (LED should be solid)
2. **Launch the system**: `ros2 launch turtle_gamepad_3 turtle_gamepad_launch.py`
3. **Control the turtle**:
   - Use left analog stick to move the turtle
   - Press RB (Right Bumper) to increase speed
   - Press LB (Left Bumper) to decrease speed
   - Press BACK button for emergency stop/resume
   - Press START button to reset turtle position

## Troubleshooting

1. **Gamepad not detected**: Check USB connection and run `ls /dev/input/js*`
2. **Permission denied**: Ensure user is in input group: `sudo usermod -a -G input $USER`
3. **No turtle movement**: Check if emergency stop is active, verify topic publishing with `ros2 topic echo /turtle1/cmd_vel`
4. **Build errors**: Ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`

## Testing

Run the test suite with:
```bash
colcon test --packages-select turtle_gamepad_3
pytest src/turtle_gamepad_3/test/ -v
```

## Architecture

The solution consists of three main components:
1. **joy_node**: Publishes gamepad input to `/joy` topic
2. **turtle_controller**: Translates gamepad input to turtle movement commands
3. **turtlesim_node**: Simulates the turtle and visualizes movement

The system uses ROS2 topics for communication and services for turtle control functions like teleportation and screen clearing.
