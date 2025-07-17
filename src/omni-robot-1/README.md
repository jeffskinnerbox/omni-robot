
<!-- markdownlint-disable MD034 -->

----

Claude Prompt - Omni-Robot-1 Step #1:

I want to use ROS2 Jazzy Jalisco to build a vehicle that will be manually controlled
by a human operator (aka teleoperated) and controlled autonomously via ROS2 Jazzy controller.
The vehicle will have three 48mm omni-directional wheels set at 120 degrees apart.
Each wheel is driven by a N20 DC motor with magnetic encoder (found at this link https://www.adafruit.com/product/4638).
The N20 DC motors will be powered and controlled by an Adafruit DC & Stepper Motor Bonnet
(found at this link https://www.adafruit.com/product/4280).
The Adafruit Bonnet will be attached to a Raspberry Pi 5 with a Ubuntu 24.04 operating system
with the platform ROS2 Jazzy installed.

For manual control by a human operator, the control of the vehicle's movement will be done by
a Logitech F310 Wired Gamepad Controller in XInput mode on Linux.
The left joystick will be used for steering, and the "A" and "B" buttons will be used for emergency stop.
Also make the left bumper switch (LB) an enable button (must hold to move)
and right stick click the turbo mode (1st press is twice as fast, 2nd presses is three time faster, and 3rd press back to normal movement).
Do not use a computer keyboard for any controls.
To assist the human operator, a forward facing camera will be provided to give the human operator real-time viewing.
The camera up/down position will be adjusted via a servomotor and that servomotor will be controlled via
the F310's D-pad vertical.

For ROS2 autonomous control, the ROS2 Jazzy vehicle mechanism will make use of the magnetic encoders on the N20 motors,
Adafruit's LSM6DSOX + LIS3MDL - Precision 9 DoF IMU (found at this link https://www.adafruit.com/product/4517)
Adafruit's VL53L1X Time of Flight Distance Sensor (found at this link https://www.adafruit.com/product/3967),
Adafruit's US-100 Ultrasonic Distance Sensor (found at this link https://www.adafruit.com/product/4019),
and a Raspberry Pi Camera Module 3 (found at this link https://www.adafruit.com/product/5657).

Creation of the this vehicle should be done in phases and you are to outline a plan of execution of those phase.
Your first assignment, that is the 1st phase, is to create this plan of execution, and we'll call this phase "Step #1".
Keep in mind that Step #1 needs to include:

1. A methodology, or series of development steps, we will proceed through to create the vehicle.
   This should include phase like creation of a simulation, and creation, testing & tuning of subsystem of the overall solution.
2. An architectural breakdown of the ROS2 nodes that will be created, what packages will be used,
   functions they will perform, purpose they fulfil, and topics they subscribe / publish too.
   Also identify what custom nodes I will have to create for this project.
3. Recommendations of the ROS2 Jazzy packages that can use for the creation of the vehicle.
4. Recommendation of special features that should be implemented like a deadman switch, emergency stop functionality,
   timeout protection for lost communication, joystick deadzone handling for smooth control, etc.
5. Identify the sequence things need to be created / specified. For example, when do I need to develop a URDF file,
   when should I begin the development of simulation, etc.

No code should be generated at this time, just provide the high level architect for the solution
and address the needs as outline above for "Step #1".
At a later time, you will provide the last step, which will generate the ROS2 Jazzy / Python code
to build, test, simulate, and operate the vehicle.

Now write for me a document covering "Step #1".


"ROS2 Omni-Directional Vehicle Development Plan - Step #1" has been published publicly on [Anthropic Claude](https://claude.ai/public/artifacts/c818ef2a-1a69-46b1-89cc-e901b4d9599e)


----




Make use of ROS2 Jazzy packages ros-jazzy-joy, ros-jazzy-teleop-twist-joy to interface with the
Logitech F310 GamePad Controller and convert gamepad inputs to velocity commands.

Make use of ROS2 Jazzy packages ros-jazzy-ros2-control, ros-jazzy-ros2-controllers to create a custom custom "N20 motor control"
and integrate this with pid_controller to implementing PID control for precise differential drive motor control.

Make use of the ROS2 Jazzy package ros-jazzy-camera-ros to stream video from Pi Camera Module 3.

Make use of the ROS2 Jazzy package ros-jazzy-imu-tools to calculate orientation, angular velocity,
linear acceleration of the vehicle
and the ROS2 Jazzy package ros-jazzy-robot-state-publisher to publish vehicle's kinematic tree.

I want to name the ROS2 workspace "omni-robot" and name this package "omni-robot-1".
Title the solution guide you create "Omni-Robot #1 Guide".
Create a launch files for all nodes.


* [ros2_control: Getting Started](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
* [How To Add A Motor Controller To Your ROS Robot](https://medium.com/exploring-ros-robotics/how-to-add-a-motor-controller-to-your-ros-robot-fd17352cd5e3)
* [ros2_control Documentation](https://control.ros.org/jazzy/index.html)
* [ros2_controllers / PID Controller](https://control.ros.org/jazzy/doc/ros2_controllers/pid_controller/doc/userdoc.html)




Provide me a draft URDF (Universal Robot Description Format) file to define the robot's mechanical structure,
including the N20 motors and encoders.

Provide me guidance on what ROS2 nodes I should use and how I should configure them.

Provide a comprehensive breakdown of the nodes you'll need and their configuration.

Provided a complete YAML configuration file mapping all the Logitech F310 Wired Gamepad Controller joysticks & buttons.


Provide me plans for each of the following build increments.

----

