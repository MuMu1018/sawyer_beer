#!/usr/bin/env python



from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
from sawyer_beer.srv import *
import rospy

class vision():

    def __init__(self):
        self.p = Pose()

    def callback(self,data):
        # print("got to this -1")
        ar_marker = data
        # print("got to this 0")
        markers = ar_marker.markers
        # print("got to this 1")
        # print("got to this 2")
        self.p = markers[0].pose.pose
        # print("got to this 3")
        return


    def handle_gettarget(self,req):
        print "Returning target position"
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        return targetResponse(self.p)

    def gettarget_server(self):
        rospy.init_node('get_target_server')
        # print('now in server')
        s=rospy.Service('get_target', target, self.handle_gettarget)
        rospy.spin()

if __name__ == "__main__":
    v = vision()
    v.gettarget_server()
