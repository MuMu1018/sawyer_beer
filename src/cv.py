#!/usr/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sawyer_beer.msg import Centroid

def nothing(x):
    pass

class image_converter:

    def __init__(self):
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self.bridge = CvBridge()
        self.filterx = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.filtery = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.x = 0
        self.y = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print("===Camera Manage===", e)


        ###############
        # OpenCV image processing
        ###############
        # _, frame = cv_image.read()

        # cv2.createTrackbar('R',cv_image,0,255,nothing)

        # Blue filter
        # lower = np.array([110,50,50]) # 110, 50, 50
        # upper = np.array([130,255,255]) # 130,255,255

        # Red filter
        lower = np.array([0,100,100])
        upper = np.array([9,255,255])

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

        # More stuff for finding the shape
        erode = cv2.erode(mask, None, iterations=2)
        dilate = cv2.dilate(erode, None, iterations=2)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # h_max = 0
        x_max = 0
        y_max = 0
        # w_max = 0
        # for c in contours:
        #     x, y, w, h = cv2.boundingRect(c)
        #     if h > h_max:
        #         h_max = h
        #         x_max = x
        #         y_max = y
        #         w_max = w

        x_cir = 0
        y_cir = 0
        r_max = 0
        for c in contours:
            (x_cir, y_cir), radius = cv2.minEnclosingCircle(c)
            if radius > r_max:
                r_max = radius
                x_max = x_cir
                y_max = y_cir

        # cv2.rectangle(cv_image, (x_max, y_max), (x_max + w_max, y_max + h_max), [0, 0, 255], 2)



        # some filtering

        self.filterx[1:10] = self.filterx[0:9]
        self.filterx[0] = x_max
        if np.abs(x_max - self.x) > 10 :
            self.x = np.mean(self.filterx[0:5])
        else:
            self.x = np.mean(self.filterx[0:8])

        self.filtery[1:10] = self.filtery[0:9]
        self.filtery[0] = y_max
        if np.abs(y_max - self.y) > 10:
            self.y = np.mean(self.filtery[0:5])
        else:
            self.y = np.mean(self.filtery[0:8])

        # self.filterx[1:10] = self.filterx[0:9]
        # self.filterx[0] = x_cir
        # if np.abs(x_cir - self.x) > 10 :
        #     self.x = np.mean(self.filterx[0:5])
        # else:
        #     self.x = np.mean(self.filterx[0:8])

        # self.filtery[1:10] = self.filtery[0:9]
        # self.filtery[0] = y_cir
        # if np.abs(y_cir - self.y) > 10:
        #     self.y = np.mean(self.filtery[0:5])
        # else:
        #     self.y = np.mean(self.filtery[0:8])

        # Publish the position of the ball to a rostopic
        height, width = cv_image.shape[:2]
        pub = rospy.Publisher('/ball_center', Centroid, queue_size = 10)
        msg = [self.x, self.y, width, height]
        pub.publish(x_max, y_max, width, height)


        # cv2.circle(cv_image, (int(self.x) + width/2, int(self.y) + h_max/2), 7, [255,255,255], -1)
        cv2.circle(cv_image, (int(self.x), int(self.y)), 6, [255,255,255], -1)

        cv2.imshow("Image window", cv_image)
        cv2.imshow("Res window", dilate)
        cv2.waitKey(1)



if __name__ == '__main__':

    rospy.init_node('my_img_processor', anonymous=True)

    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
