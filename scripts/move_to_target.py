#!/usr/bin/env python

import sys
import copy
import rospy
import random
import moveit_commander
import moveit_msgs.msg

from sawyer_beer.srv import target, targetResponse, targetRequest
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String
# from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse

from sensor_msgs.msg import JointState

import intera_dataflow
from intera_io import IODeviceInterface
from intera_core_msgs.msg import IONodeConfiguration

from collision_objs import collisionObjects

# DEFAULT_IK_SERVICE = "ExternalTools/right/PositionKinematicsNode/IKService"
DEFAULT_IK_SERVICE = "compute_ik"
DEFAULT_CHECK_SERVICE = "check_state_validity"
DEFAULT_GET_TARGET = "/ar_pose_marker"
DEFAULT_VISION_SERVICE = "/get_target"

class beerGrabber():

    def __init__(self):
        # let's create MoveIt! objects:
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("right_arm")

        self.group.set_max_velocity_scaling_factor(0.8)
        self.group.set_max_acceleration_scaling_factor(0.8)
        self.group.set_planning_time(60)

        self.right_arm = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        rospy.loginfo("Initializing beerGrabber")

        # ik service
        self.ik_service_name = DEFAULT_IK_SERVICE
        self.ik_serv = rospy.ServiceProxy(self.ik_service_name, SolvePositionIK)
        self.ik_serv.wait_for_service()
        rospy.loginfo("Successful connection to '" + self.ik_service_name + "'.")

        # vision service
        self.vision_service_name = DEFAULT_VISION_SERVICE
        self.vision_serv = rospy.ServiceProxy(self.vision_service_name, target)
        self.vision_serv.wait_for_service()
        rospy.loginfo("Successful connection to '" + self.ik_service_name + "'.")

        # check state service
        self.check_service_name = DEFAULT_CHECK_SERVICE
        # self.check_serv = rospy.ServiceProxy(self.check_service_name, GetStateValidity)
        self.check_serv = rospy.ServiceProxy(self.check_service_name, GetPositionIK)
        self.check_serv.wait_for_service()
        rospy.loginfo("Successful connection to '" + self.check_service_name + "'.")

        # initiate gripper
        side="right"
        grip_name = '_'.join([side, 'gripper'])
        self.gripper_io = IODeviceInterface("end_effector", grip_name)

        # add target position - might be useful when add bottle in the scene
        self.target = PoseStamped()
        self.dict = collisionObjects()

    def getTargetEEF(self):
        """
        Get the target position in Cartesian Space from vision node

        @return type: returns Pose() msg
        """
        req = targetRequest()
        req.data = 0
        resp = self.vision_serv.call(req)

        return resp.pose

    # def convertToJointStates(self, target):
        # """
        # Using IK solver to convert the target position to Joint Space from Cartesian Space
        #
        # @param target: Pose() msg to be converted
        # @return type: returns a list of joint values
        # """
        # ikreq = SolvePositionIKRequest()
        #
        # p = PoseStamped()
        # p.header = Header(stamp=rospy.Time.now(), frame_id='base')
        # p.pose = target
        # print "========= target 0 ========="
        # print target
        # # add target position - might be useful when add bottle in the scene
        # self.target = p
        # # Add desired pose for inverse kinematics
        # ikreq.pose_stamp.append(p)
        # # Request inverse kinematics from base to "right_hand" link
        # ikreq.tip_names.append('right_hand')
        #
        # # The joint seed is where the IK position solver starts its optimization
        # ikreq.seed_mode = ikreq.SEED_USER
        # seed = JointState()
        # seed.name = self.right_arm
        # # use random numbers to get different solutions
        # # TODO: how to define the limitation
        # j1 = random.randint(50,340)/100.0
        # j2 = random.randint(40,300)/100.0
        # j3 = random.randint(-180,140)/100.0
        # j4 = random.randint(-150,10)/100.0
        # j5 = random.randint(-180,10)/100.0
        # j6 = random.randint(-200,10)/100.0
        # j7 = random.randint(-100,200)/100.0
        # # j1 range 0.5 to 3.4
        # seed.position = self.group.get_random_joint_values()
        # # seed.position = [j1, j2, j3, j4, j5, j6, j7]
        # # seed.position = [-1.0460302734375, 1.2869541015625, -0.343587890625, -1.6291728515625, -2.08798828125, -0.701947265625, 3.9707255859375]
        # # seed.position = [-1.1142587890625, 0.0140517578125, 0.734078125, 0.7475322265625, -1.769462890625, -0.753177734375, 2.4646884765625]
        # ikreq.seed_angles.append(seed)
        #
        # # get the response from IK solver Service
        # resp = self.ik_serv.call(ikreq)
        # print "========= resp 0 ========="
        # print resp
        #
        # # reassgin a random value to joint seed if fail to get the valid solution
        # while resp.result_type[0] <= 0:
        #     print "error type: "
        #     print resp.result_type[0]
        #     # j1 = random.randint(50,340)/100.0
        #     seed.position = self.group.get_random_joint_values()
        #     ikreq.seed_angles.append(seed)
        #     resp = self.ik_serv.call(ikreq)
        #
        # # message print out
        # rospy.loginfo("SUCCESS - Valid Joint Solution Found")
        #
        # # rospy.logdebug("Response message: " + str(resp))
        #
        # target_js = [resp.joints[0].position[0],
        #                 resp.joints[0].position[1],
        #                 resp.joints[0].position[2],
        #                 resp.joints[0].position[3],
        #                 resp.joints[0].position[4],
        #                 resp.joints[0].position[5],
        #                 resp.joints[0].position[6]]
        #
        # # rospy.logdebug("Target value: " + str(target_js))

        # return target_js

    def convertToJointStates(self, target):
        rospy.loginfo("Attempting to solve IK")
        req = GetPositionIKRequest()
        req.ik_request.group_name = "right_arm"
        req.ik_request.robot_state.joint_state.name = self.group.get_active_joints()
        req.ik_request.robot_state.joint_state.position = [0, 0, 0, 0, 0, 0, 0]
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = rospy.Duration(3.0)
        req.ik_request.pose_stamped.header = Header(stamp=rospy.Time.now(), frame_id='base')
        req.ik_request.pose_stamped.pose = target
        req.ik_request.attempts = 10
        resp = self.ik_serv(req)
        # print "Error = ", resp.error_code
        # print "Solution = ", resp.solution.joint_state.position,"\r"
        q = np.zeros(len(self.group.get_active_joints()))

        while not resp.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            req.ik_request.robot_state.joint_state.position = self.group.get_random_joint_values()
            resp = self.ik_serv(req)

        sol = {}
        for k,v in zip(resp.solution.joint_state.name, resp.solution.joint_state.position):
            sol[k] = v
        for i,n in enumerate(self.group.get_active_joints()):
            q[i] = sol[n]
        rospy.loginfo("Found IK solution! q = %s", str(q.tolist()))

        return q

    def addCollisionObjects(self):
        # TODO: add color

        rospy.sleep(5)
        # table scene
        self.scene.remove_world_object("table")
        self.scene.remove_world_object("table1")
        self.scene.remove_world_object("table2")

        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()

        p.pose.position.x = -0.5
        p.pose.position.y = -1.0
        p.pose.position.z = -0.211
        self.scene.add_box("table1", p, (1.6,1.4,0.03))

        p.pose.position.x = 1.0
        p.pose.position.y = -0.5
        p.pose.position.z = -0.211
        self.scene.add_box("table2", p, (1.4,2.0,0.03))

        print "Add tables!"

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
        print "===================================="
        print resp
        print "====================================="
        print "\r\n"
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
        self.group.execute(plan)
        print "Done!"

    def gripAct(self, pose_target):
        ## Gripper
        self.gripper_io.set_signal_value("position_m", 0.041)
        rospy.sleep(1)

        # move gripper closer
        waypoints = []
        waypoints.append(pose_target)
        # setup the sequence
        wpose = Pose()
        wpose.orientation.w = -1.0
        wpose.orientation.x=0.0
        wpose.orientation.y=-1.0
        wpose.orientation.z=0.0
        wpose.position.x = waypoints[0].position.x
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z
        waypoints.append(copy.deepcopy(wpose))

        for i in range(0,10):
            wpose.position.y -= 0.005
            waypoints.append(copy.deepcopy(wpose))

        (gripping, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.005,        # eef_step
                             0.0)         # jump_threshold
        self.group.execute(gripping)

        rospy.sleep(1)
        #calibrate gripper
        if self.gripper_io.get_signal_value("is_calibrated") != True:
            self.gripper_io.set_signal_value("calibrate", True)

        ## grabbing bottle
        self.gripper_io.set_signal_value("speed_mps", 1)
        self.gripper_io.set_signal_value("position_m", 0.01)
        light_size = self.gripper_io.get_signal_value("position_response_m")
        print "realeasing size is: ", light_size

        rospy.sleep(1)
        ## if object is detected
        if self.gripper_io.get_signal_value("is_gripping") != True:
            print "Not stable!"
            light_force = self.gripper_io.get_signal_value("force_response_n")
            print "risky force is: ", light_force

            self.gripper_io.set_signal_value("position_m", 0.00)

        # get true force and obejct size responses
        force = self.gripper_io.get_signal_value("force_response_n")
        obj_size = self.gripper_io.get_signal_value("position_response_m")
        print "force is: ", force
        print "object size is: ", obj_size

    def gripRelease(self):
        touch_size = self.gripper_io.get_signal_value("position_response_m")
        if self.gripper_io.get_signal_value("is_gripping"):
            print touch_size
            print "Releasing bottle!"
            self.gripper_io.set_signal_value("position_m", 0.012)
        final_force = gripper_io.get_signal_value("force_response_n")
        print "final force is: ", final_force

    def add_grippers(self, zoffset=0.065, yoffset=0.048):
        self.scene.remove_attached_object("right_gripper")
        self.scene.remove_world_object("right_finger")
        self.scene.remove_world_object("left_finger")

        p = PoseStamped()
        p.header.frame_id = "right_gripper"
        p.pose.position.x = 0.0
        p.pose.position.y = -yoffset
        p.pose.position.z = zoffset
        self.scene.attach_box("right_gripper", "right_finger", pose=p, size=(0.01, 0.01, 0.1))


        p2 = PoseStamped()
        p2.header.frame_id = "right_gripper"
        p2.pose.position.x = 0.0
        p2.pose.position.y = yoffset
        p2.pose.position.z = zoffset
        self.scene.attach_box("right_gripper", "left_finger", pose=p2, size=(0.01, 0.01, 0.1))

        return

    def add_bottle(self,target):
        rospy.sleep(5)
        # table scene
        self.scene.remove_world_object("coke")
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = target.position.x + 0.04
        p.pose.position.y = target.position.y
        p.pose.position.z = target.position.z + 0.12

        self.scene.add_box("coke", p, (0.05,0.05,0.225))

        print "Add coke!"


if __name__=='__main__':
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('move_to_target',anonymous=True)

    try:

        bg = beerGrabber()

        #open gripper to widest angle
        bg.gripper_io.set_signal_value("position_m", 0.041)

        # add collision Objects
        obj_name = ["table","fridge_back","fridge_top","fridge_left","fridge_right","fridge_bottom"]
        bg.addCollisionObjects(obj_name)

        # add grippers
        bg.add_grippers()

        # move to Home position
        # home_js = [-0.7774326171875, 1.287365234375, -2.6271181640625, 1.835900390625, -2.804662109375, 0.817126953125, 3.6170166015625]
        home_js_2 = [-0.4579658203125, -0.135384765625, -0.0716044921875, 1.371185546875, -0.6908564453125, -1.68330859375, 3.2467177734375]
        print "move to home!"
        bg.testPlan(home_js_2)

        # get target in Cartesian - Camera
        pose_target = bg.getTargetEEF()
        print "pose_target_from_camera: "
        print pose_target

        # bg.add_bottle(pose_target)

        # reset orientation
        pose_target.orientation.x=0.0
        pose_target.orientation.y=-1.0
        pose_target.orientation.z=0.0
        pose_target.orientation.w = -1.0
        pose_target.position.x = pose_target.position.x - 0.08
        pose_target.position.z = pose_target.position.z + 0.12
        # pose_target.pose.orientation.w = -1.0 * pose_target.pose.orientation.w

        # # test point in Cartesian Space - Manual setting
        # pose_target = Pose()
        #
        # pose_target.orientation.x= 1.0
        # pose_target.orientation.y= -1.0
        # pose_target.orientation.z= 0.0
        # pose_target.orientation.w = -1.0
        # pose_target.position.x = 0.609927978406
        # pose_target.position.y = -0.542434554721
        # pose_target.position.z = 0.0147707694269
        # print "pose_target_default: "
        # print pose_target

        # get target in JointState, generate and execute the plan
        target_js = bg.generateValidTargetJointState(pose_target)

        bg.testPlan(target_js)

        # gripper part - disabled for testing
        # start gripping
        rospy.sleep(1)
        bg.gripAct(pose_target)
        rospy.sleep(2)

        ## move to a new location
        release_target = Pose()
        release_target.orientation.x=0.0
        release_target.orientation.y=-1.0
        release_target.orientation.z=0.0
        release_target.orientation.w = -1.0
        release_target.position.x = 0.909927978406
        release_target.position.y = -0.042434554721
        release_target.position.z = 0.0547707694269

        # get target in JointState and move to it
        release_target_js = bg.generateValidTargetJointState(release_target)
        bg.testPlan(release_target_js)

        # releasing bottle after move into position
        rospy.sleep(1)
        bg.gripRelease()
        # final_force = gripper_io.get_signal_value("force_response_n")
        # print "final force is: ", final_force

    except rospy.ROSInterruptException:
        pass
