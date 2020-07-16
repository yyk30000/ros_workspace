#!/usr/bin/env python

import rospy
import cv2
import numpy as np 
import os, rospkg
from utills import warp_image



from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError


class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data , np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        self.img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([20,0,240])
        upper_wlane = np.array([40,15,255])
        img_wlane=cv2.inRange(self.img_hsv, lower_wlane, upper_wlane)
        img_wlane=cv2.cvtColor(img_wlane ,cv2.COLOR_GRAY2BGR)
        img_concat = np.concatenate([img_bgr, self.img_hsv, img_wlane],axis=1)

        cv2.imshow("Image window",img_concat)
        cv2.waitKey(1)

    # def mouseRGB(self,event,x,y,flags,param):
    #     if event == cv2.EVENT_LBUTTONDOWN:
    #         colorsB= self.img_hsv[y,x,0]
    #         colorsG= self.img_hsv[y,x,1]
    #         colorsR= self.img_hsv[y,x,2]
    #         colors = self.img_hsv[y,x]
    #         print("Red: ", colorsR)
    #         print("Green: ", colorsG)
    #         print("Blue: ", colorsB)
    #         print("BGR format: ", colors)
    #         print("Coordinates of pixel: X:",x,"Y: ",y)

if __name__ == '__main__' :

    rospy.init_node('image_parser',anonymous=True)

    image_parser = IMGParser()

    rospy.spin()        

