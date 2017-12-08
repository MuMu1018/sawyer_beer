Sawyer Bottle Grabber
=============================================

Goup members: Mengjiao Hong, Ben Don, Weilin Ma, Huaiyu Wang, Felix Wang
---------------------------------------------


#### Table of Contents ####

[Instructions for running the files](#Instructions)

[Project Overview](#Project Overview)

[Requirements](#Requirements)

[Important nodes](#nodes)

[Important topics](#topics)

[Detecting the location of the bottle](#Vision)

[Moving gripper to location](#Movement)

[Bottle hand-off](#Handoff)
---------------------------------------------
#### Instructions for running files <a name="Instructions"></a>

To run the files, the workspace must be connected to Sawyer and properly source. Then use the following command: `XXXXXXXXXX`

#### Project Overview  <a name="Project Overview"></a>
The main goal of this project was to use Rethink Robotics' Sawyer robot to autonomously pick a bottleoff of a work surface and offer it to a user. This was the final project for ME495: Embedded Systems in Robotics at Northwestern University. The task was split into X main states which are implemented through a state machine:
* Locate a block with tags using ar_track_alvar
* Move to above the block position using a joint trajectory
* Adjust the height using a Cartesian trajectory
* Grab the block and drop it in a specified position

#### Dependencies <a name="Requirements"></a>

  *  Intera SDK - follow the [Workstation Setup](http://sdk.rethinkrobotics.com/intera/Workstation_Setup) Instructions
  * XXXXXXXXXXX

#### Important nodes <a name="nodes"></a>
 * `vision.py` provides AR detection
 * `move_to_target.py` moves the gripper at the end effector to the bottle
 * `move_to_laser.py` uses the laser range data to adjust the height
 * `move_to_goal.py` controls the grippers and drops off the block

#### Important topics <a name="topics"></a>
 * `block_position` is published by the visualization node and contains the Pose of the block
 * `hand_position` is published by the first moving node and contains the Pose of the gripper
 * `state` contains the current state of the state machine
 * `goal` contains which group the current block belongs to for sorting

#### Detecting the block with AR tracking  <a name="Vision"></a>
In this state, ar_track_alvar is used to find the block positions and orientations. Each block has a unique tag; if multiple blocks are found, the block that is closest to the current position will be used.

#### Moving above the block  <a name="Movement"></a>
In this state, a joint trajectory is calculated to move between from the current position to 10 mm above the block. The joint angles for the gripper position at the block is calculated through inverse kinematics and transformed to the robot's base frame. The joint trajectory is then calulcated with quintic time scaling and the joint positions are sent sequentially to move the gripper to the desired position.

#### Adjusting the height  <a name="fine"></a>
In this state, a Cartesian trajectory is used to hold the orientation of the gripper constant and just change the height as the gripper moves closer to the block. The laser range data from Baxter's hand is used in a feedback loop to determine when the gripper has reached the block.


#### Dropoff <a name="drop"></a>
In this state, the gripper grabs the block and moves to drop it off in a specified location. The blocks are sorted into two groups using metadata from the AR tags to specify which dropoff location to use.
