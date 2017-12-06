#!/usr/bin/env python



from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from sawyer_beer.srv import *
import rospy

def callback(data):
    print("got to this -1")
    ar_marker = data
    print("got to this 0")
    markers = ar_marker.markers
    print("got to this 1")
    print("got to this 2")
    pose1 = markers[0].pose.pose
    print("got to this 3")



def handle_gettarget(req):
    print "Returning target position"
    pose=Pose()
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    return targetResponse()

def gettarget_server():
    rospy.init_node('gettarget_server')

    s=rospy.Service('gettarget', target, handle_gettarget)
    rospy.spin()

if __name__ == "__main__":
    gettarget_server()