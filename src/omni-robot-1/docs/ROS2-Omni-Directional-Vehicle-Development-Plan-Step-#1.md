<!-- markdownlint-disable MD034 MD022 MD032 -->

----

# ROS2 Omni-Directional Vehicle Development Plan - Step #1

## Project Overview
This document outlines the comprehensive development plan for creating a teleoperated and autonomous omni-directional vehicle using ROS2 Jazzy Jalisco. The vehicle features three 48mm omni-wheels arranged at 120-degree intervals, each driven by N20 DC motors with magnetic encoders.

## 1. Development Methodology & Phase Structure

### Phase 1: Foundation & Planning (Current - Step #1)
- **Objective**: Establish project architecture and development roadmap
- **Deliverables**: This planning document, component specifications, ROS2 node architecture

### Phase 2: Hardware Integration & Basic Control
- **Objective**: Establish basic hardware communication and manual control
- **Key Tasks**:
  - Configure Raspberry Pi 5 with Ubuntu 24.04 and ROS2 Jazzy
  - Integrate Adafruit DC Motor Bonnet with N20 motors
  - Implement basic gamepad control interface
  - Test individual motor control and encoder feedback

### Phase 3: Kinematics & Movement Control
- **Objective**: Implement omni-directional movement algorithms
- **Key Tasks**:
  - Develop inverse kinematics for 3-wheel omni-drive
  - Create velocity control algorithms
  - Implement smooth acceleration/deceleration
  - Test coordinated wheel movements

### Phase 4: Sensor Integration
- **Objective**: Integrate all sensors for autonomous capabilities
- **Key Tasks**:
  - Configure IMU (LSM6DSOX + LIS3MDL) for orientation tracking
  - Integrate distance sensors (VL53L1X ToF, US-100 Ultrasonic)
  - Set up camera systems (Pi Camera Module 3, servo-controlled camera)
  - Implement sensor fusion basics

### Phase 5: Simulation Development
- **Objective**: Create comprehensive simulation environment
- **Key Tasks**:
  - Develop URDF model of the vehicle
  - Create Gazebo simulation world
  - Implement physics-accurate omni-wheel simulation
  - Test control algorithms in simulation

### Phase 6: Advanced Control Features
- **Objective**: Implement safety and advanced control features
- **Key Tasks**:
  - Emergency stop functionality
  - Deadman switch implementation
  - Communication timeout protection
  - Joystick deadzone handling
  - Turbo mode implementation

### Phase 7: Autonomous Navigation
- **Objective**: Develop autonomous navigation capabilities
- **Key Tasks**:
  - Implement SLAM (Simultaneous Localization and Mapping)
  - Develop path planning algorithms
  - Create obstacle avoidance system
  - Integrate with Nav2 stack

### Phase 8: System Integration & Testing
- **Objective**: Full system integration and comprehensive testing
- **Key Tasks**:
  - Integration testing of all subsystems
  - Performance optimization
  - Safety system validation
  - User interface refinement

## 2. ROS2 Node Architecture

### Core Custom Nodes (To Be Created)

#### 2.1 Motor Control Node (`omni_motor_controller`)
- **Purpose**: Control three N20 DC motors via Adafruit Motor Bonnet
- **Subscribes to**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
  - `/emergency_stop` (std_msgs/Bool) - Emergency stop signal
- **Publishes**:
  - `/motor_speeds` (custom_msgs/MotorSpeeds) - Individual motor speeds
  - `/motor_status` (custom_msgs/MotorStatus) - Motor health/status
- **Functions**: Inverse kinematics, motor PWM control, encoder reading

#### 2.2 Gamepad Control Node (`gamepad_controller`)
- **Purpose**: Process F310 gamepad input for teleoperation
- **Subscribes to**:
  - `/joy` (sensor_msgs/Joy) - Gamepad input
- **Publishes**:
  - `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
  - `/emergency_stop` (std_msgs/Bool) - Emergency stop
  - `/camera_tilt` (std_msgs/Float32) - Camera servo position
- **Functions**: Joystick processing, deadzone handling, turbo mode, safety checks

#### 2.3 Sensor Fusion Node (`sensor_fusion`)
- **Purpose**: Combine IMU, encoders, and distance sensors
- **Subscribes to**:
  - `/imu/data` (sensor_msgs/Imu) - IMU data
  - `/encoders` (custom_msgs/EncoderData) - Wheel encoder data
  - `/distance_sensors` (custom_msgs/DistanceArray) - Distance sensor data
- **Publishes**:
  - `/odom` (nav_msgs/Odometry) - Vehicle odometry
  - `/robot_pose` (geometry_msgs/PoseStamped) - Current pose
- **Functions**: Sensor fusion, odometry calculation, pose estimation

#### 2.4 Safety Monitor Node (`safety_monitor`)
- **Purpose**: Monitor system health and enforce safety protocols
- **Subscribes to**:
  - `/joy` (sensor_msgs/Joy) - Gamepad heartbeat
  - `/motor_status` (custom_msgs/MotorStatus) - Motor health
  - `/system_health` (diagnostic_msgs/DiagnosticArray) - System diagnostics
- **Publishes**:
  - `/emergency_stop` (std_msgs/Bool) - Emergency stop commands
  - `/safety_status` (custom_msgs/SafetyStatus) - Safety system status
- **Functions**: Deadman switch, timeout detection, emergency stop coordination

#### 2.5 Camera Servo Node (`camera_servo_controller`)
- **Purpose**: Control camera tilt servo motor
- **Subscribes to**:
  - `/camera_tilt` (std_msgs/Float32) - Desired servo position
- **Publishes**:
  - `/camera_position` (std_msgs/Float32) - Current servo position
- **Functions**: Servo PWM control, position feedback

### Standard ROS2 Packages to Utilize

#### 2.6 Navigation Stack (Nav2)
- **Packages**: `nav2_bringup`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`
- **Purpose**: Autonomous navigation, path planning, obstacle avoidance
- **Configuration**: Custom plugins for omni-directional movement

#### 2.7 Sensor Processing
- **joy**: Gamepad input processing
- **camera_ros**: Camera stream publishing
- **imu_tools**: IMU data processing and filtering
- **robot_localization**: Extended Kalman Filter for pose estimation

#### 2.8 Simulation
- **gazebo_ros_pkgs**: Gazebo integration
- **robot_state_publisher**: URDF/joint state publishing
- **joint_state_publisher**: Joint state management

#### 2.9 Visualization & Monitoring
- **rviz2**: 3D visualization
- **rqt**: GUI tools for debugging and monitoring
- **diagnostic_updater**: System health monitoring

## 3. ROS2 Jazzy Package Recommendations

### Essential Packages
- **geometry2**: Transform calculations and coordinate frame management
- **sensor_msgs**: Standard sensor message types
- **nav_msgs**: Navigation-related messages (odometry, maps)
- **control_msgs**: Control system messages
- **diagnostic_msgs**: System diagnostics
- **image_transport**: Efficient image streaming
- **cv_bridge**: OpenCV-ROS2 integration

### Hardware Interface Packages
- **joy**: Joystick/gamepad support
- **camera_ros**: Camera integration
- **serial**: Serial communication for sensors
- **gpio**: GPIO control for Raspberry Pi

### Navigation & Control
- **nav2_common**: Navigation 2 core components
- **controller_manager**: Real-time control framework
- **diff_drive_controller**: Adaptable for omni-drive
- **robot_localization**: Multi-sensor fusion

### Simulation & Visualization
- **gazebo_ros2_control**: Hardware simulation
- **urdf**: Robot description format
- **xacro**: URDF macro language
- **tf2_tools**: Transform debugging tools

## 4. Special Features & Safety Implementations

### 4.1 Safety Features

#### Deadman Switch
- **Implementation**: Left bumper (LB) must be held continuously
- **Behavior**: Vehicle stops immediately when released
- **Timeout**: 100ms maximum response time

#### Emergency Stop
- **Triggers**: A+B buttons simultaneously, communication timeout, system fault
- **Behavior**: Immediate motor shutdown, brake engagement
- **Recovery**: Manual reset required via specific button sequence

#### Communication Timeout Protection
- **Timeout Period**: 500ms for gamepad, 1000ms for autonomous commands
- **Behavior**: Gradual deceleration to stop, not abrupt halt
- **Recovery**: Automatic when communication restored

### 4.2 Control Enhancements

#### Joystick Deadzone Handling
- **Deadzone Size**: Configurable (default 0.1 normalized units)
- **Smoothing**: Exponential curve for natural feel
- **Calibration**: Runtime adjustment capability

#### Turbo Mode System
- **Levels**: 1x (normal), 2x, 3x speed multipliers
- **Activation**: Right stick click cycling
- **Visual Feedback**: LED indicators or display
- **Safety**: Reduced in proximity to obstacles

#### Smooth Acceleration/Deceleration
- **Acceleration Limiting**: Configurable ramp rates
- **Jerk Limiting**: Smooth velocity transitions
- **Emergency Override**: Instant response for safety stops

### 4.3 Advanced Features

#### Obstacle Avoidance
- **Sensors**: ToF and ultrasonic distance sensors
- **Zones**: Warning, slow, and stop zones
- **Override**: Manual override capability with warnings

#### Auto-Leveling
- **IMU Integration**: Automatic orientation correction
- **Tilt Compensation**: Maintain level operation on inclines
- **Calibration**: Ground truth establishment

## 5. Development Sequence & Timeline

### 5.1 Pre-Development Setup (Week 1)
- **Hardware Assembly**: Physical vehicle construction
- **Software Installation**: Ubuntu 24.04, ROS2 Jazzy setup
- **Initial Testing**: Basic connectivity and power systems

### 5.2 URDF Development (Week 2)
- **Timing**: After hardware assembly, before simulation
- **Components**:
  - Vehicle chassis and wheel definitions
  - Sensor mounting positions
  - Camera servo mechanism
  - Collision and visual meshes
- **Validation**: Joint movements, sensor placements

### 5.3 Simulation Environment (Week 3)
- **Prerequisites**: URDF completion
- **Components**:
  - Gazebo world creation
  - Physics parameter tuning
  - Sensor simulation plugins
  - Environmental obstacles
- **Testing**: Virtual vehicle operation

### 5.4 Hardware Integration (Weeks 4-5)
- **Motor Control**: Basic PWM and encoder integration
- **Sensor Setup**: IMU, distance sensors, camera
- **Gamepad Interface**: Initial teleoperation capability
- **Safety Systems**: Emergency stop, basic timeout

### 5.5 Kinematics Implementation (Week 6)
- **Mathematical Model**: 3-wheel omni-drive equations
- **Software Implementation**: Inverse kinematics node
- **Testing**: Simulation validation, hardware verification
- **Tuning**: PID controllers, movement smoothing

### 5.6 Advanced Control Features (Week 7)
- **Safety Enhancements**: Deadman switch, advanced timeouts
- **Control Refinement**: Turbo mode, deadzone handling
- **Sensor Integration**: Basic sensor fusion
- **User Interface**: Status indicators, feedback systems

### 5.7 Autonomous Capabilities (Weeks 8-9)
- **Navigation Stack**: Nav2 integration and configuration
- **SLAM Implementation**: Mapping and localization
- **Path Planning**: Autonomous navigation algorithms
- **Integration Testing**: Manual/autonomous mode switching

### 5.8 System Integration & Testing (Week 10)
- **End-to-End Testing**: Full system operation
- **Performance Optimization**: Speed, accuracy, reliability
- **Safety Validation**: Comprehensive safety testing
- **Documentation**: User manuals, maintenance guides

## 6. Custom Message Definitions Required

### MotorSpeeds.msg
```
float32 motor1_speed
float32 motor2_speed
float32 motor3_speed
Header header
```

### MotorStatus.msg
```
bool motor1_healthy
bool motor2_healthy
bool motor3_healthy
float32 motor1_current
float32 motor2_current
float32 motor3_current
Header header
```

### EncoderData.msg
```
int32 motor1_ticks
int32 motor2_ticks
int32 motor3_ticks
Header header
```

### DistanceArray.msg
```
float32[] distances
string[] sensor_names
Header header
```

### SafetyStatus.msg
```
bool deadman_active
bool emergency_stop_active
bool communication_timeout
bool system_healthy
Header header
```

## 7. Development Tools & Environment

### Required Software
- **Ubuntu 24.04 LTS**: Base operating system
- **ROS2 Jazzy Jalisco**: Robotics framework
- **Gazebo Garden**: Simulation environment
- **Visual Studio Code**: Development environment
- **Git**: Version control

### Hardware Testing Tools
- **Oscilloscope**: Signal analysis
- **Multimeter**: Electrical testing
- **Logic Analyzer**: Digital signal debugging
- **Camera Tools**: Image stream verification

### Recommended Development Practices
- **Version Control**: Git repository with branching strategy
- **Testing**: Unit tests for each node
- **Documentation**: Inline code documentation
- **Configuration Management**: YAML-based parameter files
- **Continuous Integration**: Automated testing pipeline

## Next Steps

Upon completion of Step #1 (this planning phase), proceed to Step #2: Hardware Integration & Basic Control. This will involve setting up the Raspberry Pi environment, configuring the motor bonnet, and implementing basic gamepad control functionality.

The detailed implementation of each phase will be provided in subsequent steps, with complete ROS2 Jazzy/Python code generation occurring in the final implementation phase.

