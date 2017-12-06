#!/usr/bin/env python



from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from sawyer_beer.srv import *
import rospy

def callback(data,pose1):
    ar_marker = data
    markers = ar_marker.markers
    id = markers.id
    pose1 = markers.pose.pose



def handle_gettarget(req):
    print "Returning target position"
    pose=[]
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback,pose)
    return targetResponse(pose)

def gettarget_server():
    rospy.init_node('gettarget_server')

    s = rospy.Service('gettarget', target, handle_gettarget)
    rospy.spin()

if __name__ == "__main__":
    gettarget_server()