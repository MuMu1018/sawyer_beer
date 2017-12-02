#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL

from std_msgs.msg import String
from sensor_msgs.msg import JointState

class beerGrabber():
    def __init__(self):


    def getTargetEEF():
    # no Input
    # Output: target position in Cartesian Space (from vision)

    def convertToJointStates(self,pose):
    # inpout: Cartesian position of target
    # output: JointState of target

    def addCollisionObjects(self):
    # input: description of the object
    # output: none
    # scene.add_box

    def checkValidation(self,):
    ## check the JointState of target configuration,
    # input: JointState of the target configuration
    # output: True or False

    def generateValidTargetJointState(self,pose):
    # input: target in Cartesian Space
    # output: valid target in JointState (a list of 7 numbers)
        target = self.convertToJointStates(pose)
        while not checkValidation(target):
            target = self.convertToJointStates(pose)
        return target



if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('move_to_target',
                     anonymous=True)
    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander("right_arm")

    display_trajectory_publisher = rospy.Publisher(
                                         '/move_group/display_planned_path',
                                         moveit_msgs.msg.DisplayTrajectory,
                                         queue_size = 10)

    bg = beerGrabber()

    # add collision Objects
    bg.addCollisionObjects()

    # get target in Cartesian
    target = bg.getTargetEEF()

    # get target in JointState
    target_js = bg.generateValidTargetJointState(target)

    # set target
    group.set_joint_value_target(target_js)

    # generate plan
    plan = group.plan()

    # execute plan
    group.go(wait=True)
