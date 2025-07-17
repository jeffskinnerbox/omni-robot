
# Getting Started with Robot Operating System 2 (ROS 2)
The [Robot Operating System (ROS)][03] is not an operating system in the traditional sense,
but a "glue" or middleware that holds your robotic system together.
It enables different robot components, running as separate processes within multiple physical locations.
With ROS, these process can exchange information using using computer services like
communicating messages (e.g. current state), perform services (e.g. compute something for me),
or actions (e.g. (a goal is requested, perform the task, and give feedback as you execute).
ROS allows various parts of the robot system (sensors, actuators, decision-making units)
to work cooperatively to fulfill its mission.

## Robotics
What is a robot? What is robotics?

## Installing ROS2

## The Tech Stack
Robot
Robotics
Process
Middleware
Operating System (OS)
Open Source
Software Framework
Computer Vision (CV)
Neural Networks
Machine Learning (ML)
Artificial Intelligence (AI)
Generative AI
Large Language Models (LLMs)
Physics Engine - realistic interactions with physical properties such as momentum, inertia, and gravity
Modelling - refers to the process of creating a digital representation of a physical object

## Robot Modelling and Visualisation
A critical aspect of designing and simulating robots is the creation of
accurate and visually / dynamically representative models of the robot.
These models are blueprints for capturing the geometric properties and mechanical mobility of the robot.
They enabling you to analyze, simulate, and control robotic systems effectively.
A design document used by ROS2 for achieving this is the [Unified Robot Description Format (URDF)][01].
URDF is an XML-based file format widely used in the robotics community.
It was originally developed by [Willow Garage][02] and adopted as a standard for
robot modeling in the [Robot Operating System (ROS)][03].

Why is the visual & dynamic representation of the robot with URDF important?

* **Reduce Cycle-Time:** Creating a robot is an iterative process of ideation, design, build, test, re-plan, and repeat.
If you must build physical robot for each cycle, this becomes very expensive in both time & money.
So a virtual process (aka digital representation of what physical is happening) is highly desirable.
* **Visualization:** A visually realistic robot model aids in understanding how a robot will look and move in the real world,
making it easier to communicate design concepts and ideas.
* **Dynamic Simulation:** Before building a physical robot, you want to validate its behavior in a virtual environment.
Accurate visual representation allows for realistic simulations of task that need to be performed,
which can help in testing and fine-tuning control algorithms and robot behavior.
* **Sensory Simulation:** In robotics, sensor placement, and orientation are critical.
An accurate visual model assists in simulating and predicting how sensors (such as cameras or LIDAR)
interact with the environment and assist the robot in it mission.

### Components of a URDF Visual Model
The components that make up URDF Visual Model are intended to inform the robots internal controls
how the robot can configure itself in physical space.
When operated, or simulated, you want to know how the robot is positioned,
has a robot component (e.g. arm) reached it destination,
or will it collide with something.
In addition, for the robot to move from one physical position to a target position,
it must transform itself to reach its target
(essentially a mathematical operation performed to convert one coordinate system to another).

What are the components of the URDF model that captures the robots configuration?

* **Links:** In URDF, a robot is composed of individual links.
Links represent physical components of the robot, such as arms, wheels, or the base/body.
Each link can have visual and collision representations.
* **Visual Elements:** Within each link, you define visual elements to represent how it looks.
These visual elements include geometric shapes (like cylinders, spheres, and meshes)
and their associated properties, such as size, color, and texture.
* **Collision Elements:** While visual elements define how a robot looks,
collision elements define its shape for collision detection.
They may differ from the visual representation,
often simplifying the geometry for efficient computation.
* **Transforms:** URDF allows you to specify the relative pose of links and their visual/collision elements,
enabling you to accurately model complex robot structures.
* **Material Properties:** You can define material properties, such as color and transparency, for visual elements,
contributing to the realism of the model.

### Tools for Working with URDF
Creating and manipulating URDF files can be accomplished with various tools,
such as ROS packages like `urdfdom` for parsing URDF XML statements and `Rviz` for visualization,
or modeling software like Blender or CAD tools.

## Sources
* [Getting Started with Robot Operating System 2 (ROS 2)](https://medium.com/@thebinayak/getting-started-with-robot-operating-system-2-ros-2-ef56d2ac29f0)
* [An Introduction to Understanding Visual Robot Models with URDF](https://www.linkedin.com/pulse/introduction-understanding-visual-robot-models-urdf-kangal/)
* [Robot Modelling and Visualisation in ROS 2: A Practical Guide](https://medium.com/@thebinayak/robot-modelling-and-visualisation-in-ros-2-a-practical-guide-fa666160011d)



[01]:https://www.linkedin.com/pulse/introduction-understanding-visual-robot-models-urdf-kangal/
[02]:https://en.wikipedia.org/wiki/Willow_Garage
[03]:https://en.wikipedia.org/wiki/Robot_Operating_System
