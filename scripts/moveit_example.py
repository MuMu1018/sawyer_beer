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

class MoveItCollisionTest( object ):
    def __init__(self):
        rospy.loginfo("Creating MoveItCollisionTest object")


        # let's create MoveIt! objects:
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

        rospy.sleep(3.0)

        # let's add world collision objects:
        self.add_grippers()
        
        return

    def add_grippers(self, zoffset=0.045, yoffset=0.038):
        # self.right_arm_group.detach_object("right_finger")
        # self.scene.remove_world_object("right_finger")
        # p = PoseStamped()
        # p.header.frame_id = "right_gripper"
        # p.pose.position.x = 0.0
        # p.pose.position.y = -yoffset
        # p.pose.position.z = zoffset
        # self.scene.add_box("right_finger", p, (0.01, 0.01, 0.05))
        # self.right_arm_group.attach_object("right_finger", "right_gripper")

        # self.right_arm_group.detach_object("left_finger")
        # self.scene.remove_world_object("left_finger")
        # p2 = PoseStamped()
        # p2.header.frame_id = "right_gripper"
        # p2.pose.position.x = 0.0
        # p2.pose.position.y = yoffset
        # p2.pose.position.z = zoffset
        # self.scene.add_box("left_finger", p2, (0.01, 0.01, 0.05))
        # self.right_arm_group.attach_object("left_finger", "right_gripper")

        self.scene.remove_attached_object("right_gripper")
        self.scene.remove_world_object("right_finger")
        self.scene.remove_world_object("left_finger")
        
        p = PoseStamped()
        p.header.frame_id = "right_gripper"
        p.pose.position.x = 0.0
        p.pose.position.y = -yoffset
        p.pose.position.z = zoffset
        self.scene.attach_box("right_gripper", "right_finger", pose=p, size=(0.01, 0.01, 0.05))


        p2 = PoseStamped()
        p2.header.frame_id = "right_gripper"
        p2.pose.position.x = 0.0
        p2.pose.position.y = yoffset
        p2.pose.position.z = zoffset
        self.scene.attach_box("right_gripper", "left_finger", pose=p2, size=(0.01, 0.01, 0.05))

        return



def main():
    rospy.init_node("moveit_collision_test", log_level=rospy.INFO)
    
    try:
        colltest = MoveItCollisionTest()
    except rospy.ROSInterruptException: pass

    rospy.spin()



if __name__ == '__main__':
	main()
