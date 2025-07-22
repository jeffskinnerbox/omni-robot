<!-- markdownlint-disable MD022 MD032 -->



REVIEW these documents / videos and create content for this document
* [How to Create URDF and Launch Files in ROS2- Minimal Working Example](https://www.youtube.com/watch?v=jDsb8xEdbKM)
* [How to Create a URDF File of the UR3e Robotic Arm – ROS 2](https://automaticaddison.com/how-to-create-a-urdf-file-of-the-ur3e-robotic-arm-ros-2/)
* [How to Create a Simulated Mobile Robot in ROS 2 Using URDF](https://automaticaddison.com/how-to-create-a-simulated-mobile-robot-in-ros-2-using-urdf/)
* [Robot Modelling and Visualisation in ROS 2: A Practical Guide](https://medium.com/@thebinayak/robot-modelling-and-visualisation-in-ros-2-a-practical-guide-fa666160011d)







----

# ROS2 Robot Modelling and Visualisation
A critical aspect of designing and simulating robots is the creation of
accurate and visually representative models of the robot.
These models are blueprints for capturing the geometric properties and mechanical mobility of the robot.
They enabling you to analyze, simulate, and control robotic systems effectively.
A design document used by ROS2 for achieving this is the [Unified Robot Description Format (URDF)][01].
URDF is an XML-based file format widely used in the robotics community.
It was originally developed by [Willow Garage][02] and adopted as a standard for
robot modeling in the [Robot Operating System (ROS)][03].

Modelling refers to the process of creating a digital representation of a physical object.
In ROS 2, we can create a simple 3D model using primitive shapes or import detailed models via 3D CAD mesh files.
We model a robot using both basic shapes in URDF and imported mesh files in `.stl` format.
This approach helps visualise complex parts designed in CAD environments.

[Getting Started with Robot Operating System 2 (ROS 2)](https://medium.com/@thebinayak/getting-started-with-robot-operating-system-2-ros-2-ef56d2ac29f0)

Sources:
* [Create a URDF with ROS2 Crash Course](https://www.youtube.com/watch?v=dZ_CyyEvBE0)
* [ROS.org Documentation: urdf / XML / link](http://wiki.ros.org/urdf/XML/link)
* [ROS.org Documentation: Create your own urdf file](http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file)
* [XML Validator](https://jsonformatter.org/xml-validator)

* [The ROS Transform System (TF) | Getting Ready to Build Robots with ROS #6](https://www.youtube.com/watch?v=QyvHhY4Y_Y8)

----

## What is the Difference Between RViz and Gazebo?
When first starting our in ROS, it can be difficult to understand the differences between RViz and Gazebo.
These two popular software tools are both enable the viewing of simulated robot in 3D.
The difference between the two can be summed up in the following excerpt from the book "Programming Robots with ROS":

  _RViz shows you what the robot thinks is happening, while Gazebo shows you what is really happening._

Sources:
* [What is the Difference Between RViz and Gazebo?](https://automaticaddison.com/what-is-the-difference-between-rviz-and-gazebo/)

### What is RViz?
* [How to Launch RViz and RQt in ROS](https://automaticaddison.com/how-to-launch-rviz-and-rqt-in-ros/)

[RViz](http://wiki.ros.org/rviz) (short for “ROS visualization”) is a 3D visualization software tool for robots, sensors, and algorithms.
It enables you to see the robot’s perception of its world (real or simulated).

The purpose of RViz is to enable you to visualize the state of a robot.
It uses sensor data to try to create an accurate depiction of what is going on in the robot’s environment.

To launch `rviz`, type the following command in your terminal:

```bash
# type the following command in your terminal
roscore

# and in a different terminal type
rosrun rviz rviz
```

The left panel is the Displays panel. It has a list of plugins.
These plugins enable you to view sensor data and robot state information.
To add a plugin, you would click the Add button on the bottom left of the window.

### What is Gazebo?
* [How to Launch Gazebo in Ubuntu](https://automaticaddison.com/how-to-launch-gazebo-in-ubuntu/)
* [How to Simulate a Robot Using Gazebo and ROS 2](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/)
* [How to make custom Gazebo worlds for your robot](https://www.youtube.com/watch?v=K4rHglJW7Hg)

Gazebo is a 3D robot simulator.
Its objective is to simulate a robot, giving you a close substitute to how your robot would behave in a real-world physical environment.
It can compute the impact of forces (such as gravity).

----

## Create a Simulated Mobile Robot: Understand Joints & Links
* [How to Create a Simulated Mobile Robot in ROS 2 Using URDF](https://automaticaddison.com/how-to-create-a-simulated-mobile-robot-in-ros-2-using-urdf/)
* [Create your own urdf file](http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file)
* [How to Create URDF and Launch Files in ROS2- Minimal Working Example](https://www.youtube.com/watch?v=jDsb8xEdbKM&list=LL&index=2&t=563s)

### How to Model With a URDF File
The official tutorial for creating a URDF file is [here on the ROS 2 website][05],
but far more helpful to show you how to create a URDF file for a real-world robot,
like the ones below:

* [myCobot 280 for Arduino by Elephant Robotics][06]
* [UR3e by Universal Robots][07]
* [Gen3 Lite Robot by Kinova Robotics][08]
* [WidowX 250 Robot Arm 6DOF by Trossen Robotics][09]
* [A0509 by Doosan Robotics][10]

Sources:
* [How to Model a Robotic Arm With a URDF File – ROS 2](https://automaticaddison.com/how-to-model-a-robotic-arm-with-a-urdf-file-ros-2/)
* [How to Create a Simulated Mobile Robot in ROS 2 Using URDF](https://automaticaddison.com/how-to-create-a-simulated-mobile-robot-in-ros-2-using-urdf/)

----

## How To Visualize URDF Files
You can quickly visualize your URDF file at any time with the package `ros-jazzy-urdf-tutorial`.

### ROS2 Native URDF Editor

```bash
# install the visualizer package
sudo apt install ros-jazzy-urdf-tutorial

# execute the visualizer package
ros2 launch urdf_tutorial display.launch.py model:=/home/jeff/omni-robot/src/omni-robot-1/omni-robot-1.urdf

# creates a pdf file with the parent / child relationship for all the joints
ros2 run tf2_tools view_frames
```

There are other tools you can use to check the URDF file that do not require visualizing with [RViz2][04].

```bash
# check if the syntax is correct and print out the parent / child relationship
check_urdf /home/jeff/omni-robot/src/omni-robot-1/omni-robot-1.urdf

# you can try to visualize the URDF using graphviz
urdf_to_graphviz /home/jeff/omni-robot/src/omni-robot-1/omni-robot-1.urdf
```

### A Better URDF Editor
A visual URDF editor that does not depend on ROS would be of great value.
It is very laborious to iterate with the ROS visualizer package
(aka `ros2 launch urdf_tutorial display.launch.py model:=<path-to-urdf-file`).

```bash
# create your python virtual environment
cd $HOME
python3 -m venv urdf-editor

# activate the virtual environment and enter your virtual env
source $HOME/urdf-editor/bin/activate
cd $HOME/urdf-editor

# install the jupyterlab
pip3 install jupyterlab

# install the urdf editor
pip3 install jupyterlab-urdf

# launch jupyterlab
jupyter lab
```

----

## Creating a URDF Files

### Creating a URDF Files Using Geometric Components

### Creating a URDF Files Using CAD Mesh Files

----

### FoxGlove

### MoveIt
Using the URDF, we can generate a MoveIt config that lets us simulate movement and send real commands to the physical arm.
For that, there’s a great tutorial ([link](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html))
To save you time again, here’s my [prebuilt config](https://github.com/vladmiron85/armbot/tree/master/catkin_ws/src/armbot_moveit_config).

You can download and launch it with:

```bash
roslaunch armbot_moveit_config demo.launch
```

This is how you’ll control the real arm in [RViz](http://wiki.ros.org/rviz) once it’s connected to ROS,
as shown at this [link](https://www.youtube.com/watch?v=obNVXw3-NG4&t=10s).



[01]:https://www.linkedin.com/pulse/introduction-understanding-visual-robot-models-urdf-kangal/
[02]:https://en.wikipedia.org/wiki/Willow_Garage
[03]:https://en.wikipedia.org/wiki/Robot_Operating_System
[04]:https://turtlebot.github.io/turtlebot4-user-manual/software/rviz.html
[05]:https://automaticaddison.com/how-to-model-a-robotic-arm-with-a-urdf-file-ros-2/
[06]:https://automaticaddison.com/how-to-model-a-robotic-arm-with-a-urdf-file-ros-2/
[07]:https://automaticaddison.com/how-to-create-a-urdf-file-of-the-ur3e-robotic-arm-ros-2/
[08]:https://automaticaddison.com/how-to-create-a-urdf-file-of-the-gen3-lite-by-kinova-ros-2/
[09]:https://automaticaddison.com/how-to-create-a-urdf-file-of-the-widowx-250-by-interbotix-ros-2/
[10]:https://automaticaddison.com/how-to-create-a-urdf-file-of-the-a0509-by-doosan-robotics-ros-2/

