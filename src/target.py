#!/usr/bin/env python
#_*_coding:utf-8_*_


import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped




def callback(data):
    #get the pose of the data

    #exam if the pose is valid

    #if valid set the pose

    #renew the step



    ar_marker=data
    markers=ar_marker.markers
    px=markers[0].pose.pose
    print(px)







def main():
    rospy.init_node('gettarget', anonymous=True)



    get_times=0 #get the marker # positon
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)

    rospy.spin()



if __name__ == '__main__':
    main()