#!/usr/bin/env python
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import tf.transformations as tr

# Intera imports
import intera_interface
from intera_interface import CHECK_VERSION
from intera_core_msgs.msg import DigitalIOState
# moveit stuff:
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander

# non ROS stuff:
import numpy as np

# import set joint goals and SE(3) goals:
import joint_targets as jt
# import cartesian_targets as ct


class MoveItCollisionTest( object ):
    def __init__(self):
        rospy.loginfo("Creating MoveItCollisionTest object")

        # self.right_arm = intera_interface.limb.Limb("right")

        # # enable Baxter if needed:
        # rospy.loginfo("Getting robot state... ")
        # self._rs = intera_interface.RobotEnable()
        # self._init_state = self._rs.state().enabled
        # rospy.loginfo("Enabling robot... ")
        # self._rs.enable()

        # # set clean shutdown hook
        # rospy.on_shutdown(self.clean_shutdown)


        # let's create MoveIt! objects:
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

        # # add displays for the start and goal states:
        # self.start_state_pub = rospy.Publisher("/move_group/start_state", moveit_msgs.msg.DisplayRobotState, queue_size=1)
        # self.end_state_pub = rospy.Publisher("/move_group/end_state", moveit_msgs.msg.DisplayRobotState, queue_size=1)

        rospy.sleep(3.0)

        # let's add world collision objects:
        self.world_collisions()

        # now we can plan and go:
        self.right_arm_group.set_goal_position_tolerance(0.01)
        self.right_arm_group.set_goal_orientation_tolerance(0.01)
        self.right_arm_group.set_planning_time(5.0)
        self.right_arm_group.allow_replanning(True)
        self.right_arm_group.set_max_velocity_scaling_factor(0.3)
        self.right_arm_group.set_max_acceleration_scaling_factor(0.3)

        # q = self.right_arm_group.get_current_joint_values()
        # print q
        # explicitly set the size of the steps in the planner:
        # rospy.set_param("move_group/right_arms/longest_valid_segment_fraction", 0.001)

        self.right_arm_group.set_joint_value_target(jt.targets['zero'])
        rospy.loginfo("Attempting to plan")
        plan = self.right_arm_group.plan()
        rospy.loginfo("Done with planning")

        # move the arms there:
        self.right_arm_group.go()

        self.count = 0
        self.plansrv = rospy.Service("random_plan_service", Empty, self.plan_cb)

        self.check_client = rospy.ServiceProxy("check_state_validity", GetStateValidity)
        self.ik_client = rospy.ServiceProxy("compute_ik", GetPositionIK)
        
        return

    def plan_cb(self, req):
        self.random_plan_to_grasp()
        return EmptyResponse()


    def random_plan_to_grasp(self, move=False):
        # if self.count%2 == 0:
        #     self.right_arm_group.set_joint_value_target(jt.targets['test1'])
        # # elif self.count%3 == 0:
        # #     for i in range(10):
        # #         qrand = self.right_arm_group.get_random_joint_values()
        # #         if self.check_state(qrand):
        # #             break
        # #     self.right_arm_group.set_joint_value_target(qrand)
        # else:


        # RANDOM PLAN IN JOINT SPACE
        # good = False
        # for i in range(10):
        #     qrand = self.right_arm_group.get_random_joint_values()
        #     if self.check_state(qrand):
        #         self.right_arm_group.set_joint_value_target(qrand)
        #         good = True
        #         break
        # if not good:
        #     return
        # self.right_arm_group.set_joint_value_target(qrand)

        # RANDOM PLAN IN JOINT SPACE USING IK FROM MOVEIT
        # good = False
        # for i in range(10):
        #     pstamped = self.right_arm_group.get_random_pose()
        #     solve, qrand = self.solve_ik(pstamped)
        #     if solve:
        #         if self.check_state(qrand):
        #             self.right_arm_group.set_joint_value_target(qrand)
        #             good = True
        #             break
        # if not good:
        #     return
        # self.right_arm_group.set_joint_value_target(qrand)

        # RANDOM PLAN IN CARTESIAN SPACE
        good = False
        for i in range(10):
            pstamped = self.right_arm_group.get_random_pose()
            solve, qrand = self.solve_ik(pstamped)
            if solve:
                if self.check_state(qrand):
                    self.right_arm_group.set_pose_target(pstamped)
                    good = True
                    break
        if not good:
            return
        self.right_arm_group.set_pose_target(pstamped)
        
        self.right_arm_group.plan()
        self.right_arm_group.go()
        self.count += 1
        return

    def check_state(self, q):
        # build request:
        req = GetStateValidityRequest()
        req.robot_state.joint_state.name = self.right_arm_group.get_active_joints()
        req.robot_state.joint_state.position = q
        resp = self.check_client(req)
        print "Is q valid?", resp.valid
        return resp.valid

    
    def solve_ik(self, pstamped):
        rospy.loginfo("Attempting to solve IK")
        req = GetPositionIKRequest()
        req.ik_request.group_name = "right_arm"
        req.ik_request.robot_state.joint_state.name = self.right_arm_group.get_active_joints()
        req.ik_request.robot_state.joint_state.position = [0, 0, 0, 0, 0, 0, 0]
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = rospy.Duration(3.0)
        req.ik_request.pose_stamped = pstamped
        req.ik_request.attempts = 5
        resp = self.ik_client(req)
        print "Error = ", resp.error_code
        print "Solution = ", resp.solution.joint_state.position,"\r"
        q = np.zeros(len(self.right_arm_group.get_active_joints()))
        if resp.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            sol = {}
            for k,v in zip(resp.solution.joint_state.name, resp.solution.joint_state.position):
                sol[k] = v
            for i,n in enumerate(self.right_arm_group.get_active_joints()):
                q[i] = sol[n]
            rospy.loginfo("Found IK solution! q = %s", str(q.tolist()))
            return True, q
        else:
            return False, q


    # def clean_shutdown(self):
    #     print("\nExiting example...")
    #     #return to normal
    #     self.reset_control_modes()
    #     # self.set_neutral()
    #     if not self._init_state:
    #         print("Disabling robot...")
    #         self._rs.disable()
    #     return True


    # def reset_control_modes(self):
    #     rate = rospy.Rate(100)
    #     for _ in xrange(100):
    #         if rospy.is_shutdown():
    #             return False
    #         self.right_arm.exit_control_mode()
    #         self.left_arm.exit_control_mode()
    #         rate.sleep()
    #     return True


    def world_collisions(self):
        self.scene.remove_world_object("table")
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.8
        p.pose.position.y = 0.1
        p.pose.position.z = 0.1
        self.scene.add_box("table", p, (0.1, 0.1, 0.5))
        return



def main():
    rospy.init_node("moveit_collision_test", log_level=rospy.INFO)
    
    try:
        colltest = MoveItCollisionTest()
    except rospy.ROSInterruptException: pass

    rospy.spin()



if __name__ == '__main__':
	main()
