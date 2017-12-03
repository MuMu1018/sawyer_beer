#!/usr/bin/env python
#_*_coding:utf-8_*_


import rospy
from std_msgs.msg import String


def callback(data,stepincall):
    #get the pose of the data

    #exam if the pose is valid

    #if valid set the pose

    #renew the step
    stepincall=stepincall+1
    rospy.set_param('stage',stepincall)





def main():
    rospy.init_node('gettarget', anonymous=True)

    step = rospy.get_param('stage')


    get_times=0 #get the marker # positon
    rospy.Subscriber("/ar_marker", String, callback,step)

    rospy.spin()



if __name__ == '__main__':
    try:
    main()
    except rospy.ROSInterruptException:
        pass