#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg

import geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from sensor_msgs.msg import JointState

class beerGrabber():
    def __init__(self):


    def getTargetEEF():
    # no Input
    # Output: target position in Cartesian Space (from vision)
    # TODO: implement after vision group done

    def convertToJointStates(self,pose):
    # inpout: Cartesian position of target
    # output: JointState of target
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')

        # Add desired pose for inverse kinematics
        ikreq.pose_stamp.append(poses[limb])
        # Request inverse kinematics from base to "right_hand" link
        ikreq.tip_names.append('right_hand')

        if (use_advanced_options):
            # Optional Advanced IK parameters
            rospy.loginfo("Running Advanced IK Service Client example.")
            # The joint seed is where the IK position solver starts its optimization
            ikreq.seed_mode = ikreq.SEED_USER
            seed = JointState()
            seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                         'right_j4', 'right_j5', 'right_j6']
            seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
            ikreq.seed_angles.append(seed)

            # Once the primary IK task is solved, the solver will then try to bias the
            # the joint angles toward the goal joint configuration. The null space is
            # the extra degrees of freedom the joints can move without affecting the
            # primary IK task.
            ikreq.use_nullspace_goal.append(True)
            # The nullspace goal can either be the full set or subset of joint angles
            goal = JointState()
            goal.name = ['right_j1', 'right_j2', 'right_j3']
            goal.position = [0.1, -0.3, 0.5]
            ikreq.nullspace_goal.append(goal)
            # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
            # If empty, the default gain of 0.4 will be used
            ikreq.nullspace_gain.append(0.4)
        else:
            rospy.loginfo("Running Simple IK Service Client example.")

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        # Check if result valid, and type of seed ultimately used to get solution
        if (resp.result_type[0] > 0):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp.result_type[0], 'None')
            rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))convertToJointStates(self,limb = "right", use_advanced_options = True, pose)

            rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
            rospy.loginfo("------------------")
            rospy.loginfo("Response Message:\n%s", resp)
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            rospy.logerr("Result Error %d", resp.result_type[0])
            return False

        # TODO: modify resp return values to include all seven
        return resp

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
    #pose_target = bg.getTargetEEF()

    # test point in Cartesian Space
    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.w = 2.0
    # pose_target.position.x = 0.7
    # pose_target.position.y = -0.05
    # pose_target.position.z = 1.1
    pose_target = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.7,
                    y=-0.05,
                    z=1.1,
                ),
                orientation=Quaternion(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    w=2.0,
                ),
            ),
        ),
    }
    # get target in JointState
    target_js = bg.generateValidTargetJointState(pose_target)

    # set target
    group.set_joint_value_target(target_js)

    # generate plan
    plan = group.plan()

    # execute plan
    group.go(wait=True)
