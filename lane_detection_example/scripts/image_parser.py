#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):
        self.image_sub =rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.callback)

    def callback(self,msg):
        try:
            np_arr = np.fromstring(msg.data,np.uint8)
            img_bgr =cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.img_hsv =cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)

        

        cv2.namedWindow('mouseRGB')
        cv2.imshow('mouseRGB',self.img_hsv)

        cv2.setMouseCallback('mouseRGB',self.mouseRGB)

        cv2.waitKey(1)
    
    def mouseRGB(self,event,x,y,flags,param):
        if event ==cv2.EVENT_LBUTTONDOWN:
            colorsB = self.img_hsv[y,x,0]
            colorsG = self.img_hsv[y,x,1]
            colorsR = self.img_hsv[y,x,2]
            colors = self.img_hsv[y,x]
            print("Red: ",colorsR)
            print("Grean: ",colorsG)
            print("Blue: ",colorsB)
            print("BRG Format: ",colors)
            print("coordinates of pixel: x",x,"Y:",y) 
        
if __name__ =='__main__' :
    rospy.init_node('image_parser',anonymous=True)

    image_parser =IMGParser()

    rospy.spin()