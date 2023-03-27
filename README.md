# MODEL DEVELOPMENT AND SIMULATION OF A TWO-ROBOT WORK CELL FOR WORKPIECE MANIPULATION

Source code for development of a simulation for multi-robot collaboration in ROS and Gazebo.

* * *

# Table of contents
- [Introduction](https://github.com/Stark36/scarfing_robotics/README.md#introduction)
- [Methodology](https://github.com/Stark36/scarfing_robotics/README.md#methodology)
- [Requirements](https://github.com/Stark36/scarfing_robotics/README.md#requirements)
- [Installation and build](https://github.com/Stark36/scarfing_robotics/README.md#installation-and-build)
- [Usage](https://github.com/Stark36/scarfing_robotics/README.md#usage)
- [Other information](https://github.com/Stark36/scarfing_robotics/README.md#other-information)

* * *

# Introduction

This is a Final Year Project (FYP) conducted under the collaboration of the School of Electrical & Electronics Engineering of Nanyang Technology Universirty and the Agency for Science, Technology and Research (A*STAR).

This project seeks to propose and develop a simulation for multi-robot collaborative systems. While there is great potential, replicating and verifying the value of such systems in workplaces can be costly and resource intensive, especially with their physical requirements and costly equipment. Faced with such uncertainty, the justification for investments would be a tall order. Thus, an easily accessible testbed for running such tests, can provide the necessary insights to make calculated decisions.

To that end, the composite repair process of scarfing will be carried out in simulation. The simulation will consists of 2 robots (Evaluator & Processor) and the scarfing workpiece in the following:

1.  Initialisation of the simulation environment
2.  Modelling and setting up of the dual-robot work cell for simulation
    - Implementation of independent and asynchronous trajectory execution of robots
3.  Monitoring of the workpiece Evaluator
    - Development and implementation of a customised algorithm for path planning of 3D surface coverage region for tooling path by Processor
4.  Tooling of the workpiece by Processor

* * *

# Methodology

## Gazebo setup

XACRO files for the URDF of the robot models and workpiece mesh are present in the custom_urdf folder.

<u>Robots</u>
In the two-robot work cell, the Processor and Evaluator utilise the ur5 arm from the universal_robot package as their base.

- Processor: a gripper from the robotiq\_gripper package was attached to the ee\_link (end effector link) of the ur5 arm through the use of a fixed joint.
- Evaluator, a Kinect sensor from the common\_sensors (Dairal) package was attached to the ee\_link of the ur5 arm through the use of a fixed joint.

<u>Workpiece</u>
The model was designed in Autodesk Fusion 360 with a single curved surface to provide 3D surface topography for emulating a composite workpiece. Also, a support column was added for emulating a workpiece mount.

</br>

## Integration of MoveIt with Gazebo

To perform multi-robot manipulation in MoveIt, a combined URDF of the individual robots' URDF is required. One approach involves grouping the robots together in a planning group, allowing a single move\_group\_interface instance to manipulate them simultaneously. Another approach involves grouping each robot in its own planning group, and multiple move\_group\_interface instances corresponding to each robot can be used to manipulate them. However, in both cases, the manipulation of the planning groups must occur sequentially, and independent and asynchronous manipulation is not feasible.

Nonetheless, for developing collaborative multi-robot system simulations, the ability to independently and asynchronously manipulate multiple robots is crucial. As such, a total of 3 methods were explored as workarounds to MoveIt’s limitations. The source code within this repository can be separated into the following segments:

| Purpose | Package(s) | Status |
| --- | --- | --- |
| Integration of MoveIt and Gazebo through use of group namespace | ur5\_kinect\_moveit_config<br>ur5\_gripper\_moveit_config | Failure to update the planning_scene with point cloud information from the 3D sensor. Unable to avoid collision with obstacles path planning. |
| Integration of MoveIt and Gazebo through use of prefixes | ur5\_kinect2\_moveit_config<br>ur5\_gripper2\_moveit_config | Conflict of multiple controller_managers and their advertised services |
| Integration of MoveIt and Gazebo by bypassing MoveIt trajectory execution | ur5\_multi\_moveit_config | Complete |
| Composite repair process (scarfing) | one\_arm\_test | Complete |

</br>

## Scarfing process

|  Evaluator   |  Processor   |
| --- | --- |
| Actuates to observe workpiece surface |     |
| Constructs custom kd-tree from point cloud scan of workpiece surface |     |
| Constructs path plan of 3D coverage region |     |
| Communicates path plan to Processor | Reflection of processor Processor Receives path plan from Evaluator |
|     | Uses MoveIt cartesian planner to construct trajectory plan |
|     | Executes trajectory plan |
| Tracking/Reflection of Processor | Segmentation of workspace |

* * *

# Requirements

## System

- Ubuntu 20.04 with ROS noetic

## Packages

- MoveIt
- MoveIt_tutorials
- Panda\_moveit\_config
- Universal_robots (Recommended for use in Ubuntu 18.04 with ROS melodic, however Ubuntu 20.04 with ROS noetic should also work)
- Robotiq
- Common_sensors
- Openni_launch

* * *

# Installation and build

1. Clone the existing git repository

```
git clone https://github.com/Stark36/scarfing_robotics.git
```

2. Transfer the packages within src of the cloned folder to the src of your catkin workspace

3. Transfer XACRO files in custom urdf into ur_description of universal_robots package

4. Build your catkin workspace

```
catkin build
```

* * *

# Usage

## MoveIt Bypass

In separate terminals, execute the following launch files:

```
roslaunch ur5_multi_moveit_config	gazebo.launch
roslaunch ur5_multi_moveit_config	demo.launch
roslaunch one_arm_test	test2.launch
```

## Namespace
Make the following edits in the file:
<u>kinect_properties.urdf.xacro in common_sensors</u>
```
<root name="test" xmlns:xacro="http://www.ros.org/wiki/xacro">>
  <!-- Translation and rotation of camera from its base link -->
  <xacro:property name="cam_px" value="0" />
  <xacro:property name="cam_py" value="0" />
  <xacro:property name="cam_pz" value="3.0" />
  <xacro:property name="cam_or" value="0" />
  <xacro:property name="cam_op" value="1.25" />
  <xacro:property name="cam_oy" value="1.54" />
</root>
```

In separate terminals, execute the following launch files:

```
roslaunch ur5_kinect_moveit_config	gazebo_multi_arms.launch
roslaunch ur5_kinect_moveit_config	demo_multi_arms.launch
roslaunch one_arm_test	test.launch
```

## Prefix
In separate terminals, execute the following launch files:

```
roslaunch ur5_kinect2_moveit_config	gazebo_multi.launch
roslaunch ur5_kinect2_moveit_config	one_arm.launch
roslaunch ur5_gripper2_moveit_config	one_arm.launch
```

* * *

# Other information

[Link to development logs](https://docs.google.com/document/d/1xnbBOGwzZXrQN8GdOhqpwaBlRLxH1tNC1MGMIHTIg8I/edit?usp=sharing)
