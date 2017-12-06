#!/usr/bin/env python
#_*_coding:utf-8_*_


import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped




    def callback(data,stepincall,times):
        #get the pose of the data

        #exam if the pose is valid

        #if valid set the pose

        #renew the step
        step=stepincall
        calltimes=times
        if step==2:
            if calltimes<2:
                ar_marker = data
                markers = ar_marker.markers
                if markers!=[]:
                    return calltimes
                else:
                    calltimes=calltimes+1
            else:
                ar_marker=data
                markers=ar_marker.markers
                id=markers.id
                pose=markers.pose.pose
                if id!=1:
                    calltimes=0
                    return calltimes
                else:
                    pub = rospy.Publisher('targetpose', Pose, queue_size=10)
                    rate = rospy.Rate(10)
                    step=3
                    while step==3:
                        pub.publish(pose)
                        step = rospy.get_param('stage')
                    rate.sleep()



                rospy.set_param('stage',step)


        return calltimes





def main():
    rospy.init_node('gettarget', anonymous=True)
    track_times=0
    step = rospy.get_param('stage')


    get_times=0 #get the marker # positon
    track_times=rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback,step,track_times)

    rospy.spin()



if __name__ == '__main__':
    main()