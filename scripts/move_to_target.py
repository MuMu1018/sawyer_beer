#!/usr/bin/env python

import sys
import copy
import rospy
import random
import moveit_commander
import moveit_msgs.msg

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse

from sensor_msgs.msg import JointState

DEFAULT_IK_SERVICE = "ExternalTools/right/PositionKinematicsNode/IKService"
DEFAULT_CHECK_SERVICE = "check_state_validity"
DEFAULT_GET_TARGET = "/ar_pose_marker"

class beerGrabber():

    def __init__(self):
        # let's create MoveIt! objects:
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("right_arm")

        self.right_arm = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        rospy.loginfo("Initializing beerGrabber")

        # ik service
        self.ik_service_name = DEFAULT_IK_SERVICE
        self.ik_serv = rospy.ServiceProxy(self.ik_service_name, SolvePositionIK)
        self.ik_serv.wait_for_service()
        rospy.loginfo("Successful connection to '" + self.ik_service_name + "'.")

        # check state service
        self.check_service_name = DEFAULT_CHECK_SERVICE
        self.check_serv = rospy.ServiceProxy(self.check_service_name, GetStateValidity)
        self.check_serv.wait_for_service()
        rospy.loginfo("Successful connection to '" + self.check_service_name + "'.")


    def getTargetEEF():
        """
        Get the target position in Cartesian Space from vision node

        @return type: returns Pose() msg
        """
        return true

    def convertToJointStates(self, target):
        """
        Using IK solver to convert the target position to Joint Space from Cartesian Space

        @param target: Pose() msg to be converted
        @return type: returns a list of joint values
        """
        ikreq = SolvePositionIKRequest()

        p = PoseStamped()
        p.header = Header(stamp=rospy.Time.now(), frame_id='base')
        p.pose = target

        # Add desired pose for inverse kinematics
        ikreq.pose_stamp.append(p)
        # Request inverse kinematics from base to "right_hand" link
        ikreq.tip_names.append('right_hand')

        # The joint seed is where the IK position solver starts its optimization
        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = self.right_arm
        # use random numbers to get different solutions
        j1 = random.randint(50,340)/100.0
        # j1 range 0.5 to 3.4
        seed.position = [j1, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
        ikreq.seed_angles.append(seed)

        # get the response from IK solver Service
        resp = self.ik_serv.call(ikreq)

        # reassgin a random value to joint seed if fail to get the valid solution
        while resp.result_type[0] <= 0:
            j1 = random.randint(50,340)/100.0
            seed.position = [j1, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
            ikreq.seed_angles.append(seed)
            resp = self.ik_serv.call(ikreq)

        # message print out
        rospy.loginfo("SUCCESS - Valid Joint Solution Found")
        # rospy.loginfo("Response Message:\n%s", resp)

        # rospy.logdebug("Response message: " + str(resp))

        target_js = [resp.joints[0].position[0],
                        resp.joints[0].position[1],
                        resp.joints[0].position[2],
                        resp.joints[0].position[3],
                        resp.joints[0].position[4],
                        resp.joints[0].position[5],
                        resp.joints[0].position[6]]

        # rospy.logdebug("Target value: " + str(target_js))

        return target_js

    def addCollisionObjects(self):
    # input: description of the object
    # output: none
    # TODO: build a dictionary of objects - table, fridge, bottle
    # scene.add_box("box", p, (1.68, 1.22, 0.86))

    # TODO: add color
        # measurement of the world - check the sawyer_scene/fridge_closed_door.scene
        # f_length = 0.39
        # f_width = 0.47
        # f_height = 0.46
        # f_thickness = 0.03
        # f_d_thickness = 0.04
        # t_length = 1.22
        # t_width = 0.76
        # t_height, from base of the sawyer to the top of the table
        # t_height = -0.2
        # t_thickness = 0.022
        # f_b_height: from top of table to bot of fridge
        # f_b_height = 0.015
        # handle
        #

        # center of the ridge_bot
        # x = 0.74
        # y = -0.92
        # z = 0.17

        # table scene
        self.scene.remove_world_object("table")
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 1.0
        p.pose.position.y = 1.0
        p.pose.position.z = -0.211

        self.scene.add_box("table", p, (1.22,0.76,0.022))

        # # fridge scene
        # f_b = PoseStamped()
        # f_b.header.frame_id = robot.get_planning_frame()
        # f_b.pose.position.x = x
        # f_b.pose.position.y = y
        # f_b.pose.position.z = z
        #
        # f_t = PoseStamped()
        # f_t.header.frame_id = robot.get_planning_frame()
        # f_t.pose.position.x = x
        # f_t.pose.position.y = y
        # f_t.pose.position.z = z + 0.46 - f_thickness

        # TODO: add more scenes about the fridge

        return


    def checkCollisions(self,target):
        """
        Given a target position in JointState and check whether there is collision

        @param target: a list of joints value to be checked
        @return type: returns True when it is valid
        """
        req = GetStateValidityRequest()
        req.robot_state.joint_state.name = self.group.get_active_joints()
        req.robot_state.joint_state.position = target
        resp = self.check_serv(req)
        print "Is target valid?", resp.valid
        return resp.valid

    def generateValidTargetJointState(self,pose):
        """Generate a valid target position in Joint Space and returns a list"""
        target = self.convertToJointStates(pose)
        while not self.checkCollisions(target):
            target = self.convertToJointStates(pose)
        return target

    def testPlan(self,target):
        print "test PLan:"
        # set target
        self.group.set_joint_value_target(target)
        # generate plan
        plan = self.group.plan()

        # execute plan
        self.group.go()
        print "Done!"


if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('move_to_target',anonymous=True)

    try:

        bg = beerGrabber()

        # add collision Objects
        bg.addCollisionObjects()
        #scene.add_box("table", p, (1.22 0.76 0.022))

        # get target in Cartesian
        #pose_target = bg.getTargetEEF()

        # test point in Cartesian Space
        pose_target = Pose()
        pose_target.orientation.x=0.0
        pose_target.orientation.y=-1.0
        pose_target.orientation.z=0.0
        pose_target.orientation.w = -1.0
        pose_target.position.x = 0.309927978406
        pose_target.position.y = -0.542434554721
        pose_target.position.z = 0.0147707694269

        # get target in JointState
        target_js = bg.generateValidTargetJointState(pose_target)

        bg.testPlan(target_js)

    except rospy.ROSInterruptException:
        pass