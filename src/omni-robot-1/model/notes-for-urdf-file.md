<!-- markdownlint-disable MD022 MD032 -->



* [How to Create URDF and Launch Files in ROS2- Minimal Working Example](https://www.youtube.com/watch?v=jDsb8xEdbKM)




# Robot Modelling and Visualisation
A critical aspect of designing and simulating robots is the creation of
accurate and visually representative models of the robot.
These models are blueprints for capturing the geometric properties and mechanical mobility of the robot.
They enabling you to analyze, simulate, and control robotic systems effectively.
A design document used by ROS2 for achieving this is the [Unified Robot Description Format (URDF)][01].
URDF is an XML-based file format widely used in the robotics community.
It was originally developed by [Willow Garage][02] and adopted as a standard for
robot modeling in the [Robot Operating System (ROS)][03].

[Getting Started with Robot Operating System 2 (ROS 2)](https://medium.com/@thebinayak/getting-started-with-robot-operating-system-2-ros-2-ef56d2ac29f0)
[Robot Modelling and Visualisation in ROS 2: A Practical Guide](https://medium.com/@thebinayak/robot-modelling-and-visualisation-in-ros-2-a-practical-guide-fa666160011d)

Sources:
* [Create a URDF with ROS2 Crash Course](https://www.youtube.com/watch?v=dZ_CyyEvBE0)
* [ROS.org Documentation: urdf / XML / link](http://wiki.ros.org/urdf/XML/link)
* [ROS.org Documentation: Create your own urdf file](http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file)
* [XML Validator](https://jsonformatter.org/xml-validator)

* [The ROS Transform System (TF) | Getting Ready to Build Robots with ROS #6](https://www.youtube.com/watch?v=QyvHhY4Y_Y8)

## To Visualize URDF Visualize
You can quickly visualize your URDF file at any time with the package `ros-jazzy-urdf-tutorial`.

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

## A Better URDF Editor
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

## FoxGlove

## MoveIt
Using the URDF, we can generate a MoveIt config that lets us simulate movement and send real commands to the physical arm.
For that, there’s a great tutorial ([link](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html))
To save you time again, here’s my [prebuilt config](https://github.com/vladmiron85/armbot/tree/master/catkin_ws/src/armbot_moveit_config).

You can download and launch it with:

```bash
roslaunch armbot_moveit_config demo.launch
```

This is how you’ll control the real arm in [rviz](http://wiki.ros.org/rviz) once it’s connected to ROS,
as shown at this [link](https://www.youtube.com/watch?v=obNVXw3-NG4&t=10s).

[01]:https://www.linkedin.com/pulse/introduction-understanding-visual-robot-models-urdf-kangal/
[02]:https://en.wikipedia.org/wiki/Willow_Garage
[03]:https://en.wikipedia.org/wiki/Robot_Operating_System
[04]:https://turtlebot.github.io/turtlebot4-user-manual/software/rviz.html

