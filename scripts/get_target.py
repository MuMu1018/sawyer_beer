#!/usr/bin/env python

# This script is to set up a service server and gets data from the ar_pose_marker topic and then return it service client,

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
from sawyer_beer.srv import *
import rospy

class vision():

    def __init__(self):
        self.p = Pose()
	
    def callback(self,data):
        # print("got to this -1")
	# get the msg AlvarMarkers
        ar_marker = data
        # print("got to this 0")
        markers = ar_marker.markers
        # print("got to this 1")
        # print("got to this 2")
        # print markers
	# get the msg Pose which has position and orientation information on it
        self.p = markers[0].pose.pose
        # print("got to this 3")
        return


    def handle_gettarget(self,req):
        print "Returning target position"
	# create a subscriber to get the ar poss imformation from /ar_pose_marker topic
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
	# return the ar tag pose information to the service client
        return targetResponse(self.p)

    def gettarget_server(self):
	# init node
        rospy.init_node('get_target_server')
        # print('now in server')
	# set up a service server
        s=rospy.Service('get_target', target, self.handle_gettarget)
        rospy.spin()

if __name__ == "__main__":
    v = vision()
    v.gettarget_server()
