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

from sensor_msgs.msg import JointState

DEFAULT_IK_SERVICE = "ExternalTools/right/PositionKinematicsNode/IKService"

class beerGrabber():
    def __init__(self):
        rospy.loginfo("Initializing beerGrabber")
        self.ik_service_name = DEFAULT_IK_SERVICE
        self.ik_serv = rospy.ServiceProxy(self.ik_service_name, SolvePositionIK)
        self.ik_serv.wait_for_service()
        rospy.loginfo("Successful connection to '" + self.ik_service_name + "'.")

    def getTargetEEF():
    # Input: None
    # Output: target position in Cartesian Space (from vision)
    # TODO: implement after vision group done
        return true

    def convertToJointStates(self, target):
    # inpout: Cartesian position of target
    # output: JointState of target

        ikreq = SolvePositionIKRequest()

        p = PoseStamped()
        p.header = Header(stamp=rospy.Time.now(), frame_id='base')
        p.pose = target

        # Add desired pose for inverse kinematics
        ikreq.pose_stamp.append(p)
        # Request inverse kinematics from base to "right_hand" link
        ikreq.tip_names.append('right_hand')

        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        j1 = random.randint(50,340)/100.0
        # j1 range 0.5 to 3.4
        seed.position = [j1, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
        ikreq.seed_angles.append(seed)

        resp = self.ik_serv.call(ikreq)

        while resp.result_type[0] <= 0:
            j1 = random.randint(50,340)/100.0
            seed.position = [j1, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
            ikreq.seed_angles.append(seed)
            resp = self.ik_serv.call(ikreq)

        rospy.loginfo("SUCCESS - Valid Joint Solution Found")
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", resp)

        # TODO: what result it checked?
        # whether the IK will always give us a valid solution without goal JointState values
        # Check if result valid
        # if (resp.result_type[0] > 0):
        #
        #     rospy.loginfo("SUCCESS - Valid Joint Solution Found")
        #     # Format solution into Limb API-compatible dictionary
        #     limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        #
        #     rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        #     rospy.loginfo("------------------")
        #     rospy.loginfo("Response Message:\n%s", resp)
        # else:
        #     rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        #     rospy.logerr("Result Error %d", resp.result_type[0])
        #     return False

        # print "let's see the resp"
        # print resp

        target_js = [resp.joints[0].position[0],
                        resp.joints[0].position[1],
                        resp.joints[0].position[2],
                        resp.joints[0].position[3],
                        resp.joints[0].position[4],
                        resp.joints[0].position[5],
                        resp.joints[0].position[6]]

        # print "let's see the target_js"
        # print target_js

        return target_js

    def addCollisionObjects(self):
    # input: description of the object
    # output: none
    # TODO: build a dictionary of objects - table, fridge, bottle
    # scene.add_box("box", p, (1.68, 1.22, 0.86))

    # TODO: add color
        # table scene
        p = PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        p.pose.position.x = 1.0
        p.pose.position.y = 1.0
        p.pose.position.z = -0.211
        #scene.add_box("table", p, (1.22 0.76 0.022))

        # fridge scene()


    def checkCollisions(self,target):
    ## check the JointState of target configuration,
    # input: JointState of the target configuration
    # output: return True when there is collision
        return False

    def generateValidTargetJointState(self,pose):
    # input: target in Cartesian Space
    # output: valid target in JointState (a list of 7 numbers)
        target = self.convertToJointStates(pose)
        while self.checkCollisions(target):
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

    # # test whether every time IK solver will give out different solution
    # pose1 = bg.convertToJointStates(pose_target)
    # pose2 = bg.convertToJointStates(pose_target)
    # pose3 = bg.convertToJointStates(pose_target)
    # pose4 = bg.convertToJointStates(pose_target)
    # print "pose 1:"
    # print pose1
    # print "pose 2:"
    # print pose2
    # print "pose 3:"
    # print pose3
    # print "pose 4:"
    # print pose4



    # set target
    group.set_joint_value_target(target_js)

    # generate plan
    plan = group.plan()

    # execute plan
    group.go(wait=True)
