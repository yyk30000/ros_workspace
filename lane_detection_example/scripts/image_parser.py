#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from utils import warp_image

class IMGParser:
    def __init__(self):
        self.image_sub =rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.callback)

        self.source_prop =np.float32([[0.05 , 0.65],
                                    [0.5 -0.15, 0.52],
                                    [0.5 +0.15, 0.52],
                                    [1-0.05,0.65]
        ])


    def callback(self,msg):
        try:
            np_arr = np.fromstring(msg.data,np.uint8)
            img_bgr =cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv =cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)

        lower_wlane=np.array([0,0,220])
        upper_wlane=np.array([30,8,255])

        img_wlane=cv2.inRange(img_hsv,lower_wlane,upper_wlane)

        img_warp=warp_image(img_wlane,self.source_prop)

        cv2.imshow("Image window",img_warp)
        
        cv2.waitKey(1)

if __name__ =='__main__' :
    rospy.init_node('image_parser',anonymous=True)

    image_parser =IMGParser()

    rospy.spin()