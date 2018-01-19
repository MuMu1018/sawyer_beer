Sawyer Beer (Coke) Bottle Grabber
=============================================
#### ME 495 Final Project, Fall 2017, Northwestern University, Evanston, IL
#### Station: Sawyer Robot, Rethink Robotics Inc.
#### Goup members:  [Mengjiao Hong](https://github.com/MuMu1018), [Ben Don](https://github.com/benbdon), [Weilin Ma](https://github.com/KansoW) , [Huaiyu Wang](https://github.com/whycn), [Felix Wang](https://github.com/yanweiw)
---------------------------------------------

[![sawyer_coke](https://img.youtube.com/vi/asuPnFKfNog/0.jpg)](https://www.youtube.com/watch?v=asuPnFKfNog)

Another link: https://youtu.be/LEMvSSeDHZU

#### Table of Contents ####

[Instructions for running the files](#Instructions)

[Project Overview](#Project\Overview)

[Requirements](#Requirements)

[Important nodes](#nodes)

[Important topics](#topics)

[Important services](#services)

[1. Detecting the location of the bottle ](#Vision)

[2. Moving and gripping](#Movement)

---------------------------------------------
#### Instructions for running files <a name="Instructions"></a>

To run the files, the workspace must be connected to Sawyer and properly sourced. Then use the following command: `roslaunch sawyer_beer main.launch`

#### Project Overview  <a name="Project\Overview"></a>
The main goal of this project was to use Rethink Robotics' Sawyer robot to autonomously pick a bottleoff of a work surface and offer it to a user. This was the final project for ME495: Embedded Systems in Robotics at Northwestern University. The task was split into 3 major parts:
* Detecting the location of the bottle
* Moving the robot's gripper to the bottle, gripping it, moving it to the release position.


#### Requirements <a name="Requirements"></a>

  *  Intera SDK - follow the [Workstation Setup](http://sdk.rethinkrobotics.com/intera/Workstation_Setup) Instructions. This site includes all the specific dependencies such as rosdep, control-msgs, cv-bridge, etc.
  *  MoveIt! - follow the [steps](http://moveit.ros.org/install/) for install. Note the specific installation directions are for indigo, so make sure to include your OS version instead such as kinetic.

#### Important nodes <a name="nodes"></a>
 * `get_target.py` provides AR detection
 * `move_to_target.py` moves the end effector and opens/closes the gripper

#### Important topics <a name="topics"></a>
 * `ar_pose_marker` this topic is subscribed to get the AR pose information

#### Important services <a name="services"></a>
 * `ExternalTools/right/Position/KinematicsNode/IKService` this service provides IK solutions
 * `check_state_validity` this service verifies that the robot position doesn't collide with objects in the scene
 * `get_target` this service provides us the AR tag pose information from camera

#### 1. Detecting the location of the bottle <a name="Vision"></a>
In this state, a service is written that provides the pose information of the AR tag.

#### 2. Moving and gripping <a name="Movement"></a>
In this state, the gripper initializes to open. The collision objects are added to the scene. Sawyer's gripper moves to a home position that ensures the AR tag is visible by the wrist camera. Next, the node gets the AR tag pose information and calibrates it to a desired end effector position (ie where the bottle is located relative to the tag). The joint angles of the end effector in the desired orientation are computed and validated to ensure no collisions. Then the motion plan is calculated using collision information. Assuming no errors, the bottle moves to target, grips the bottle, and moves it to release position where the grip loosens enough to let the user pull the bottle out.
