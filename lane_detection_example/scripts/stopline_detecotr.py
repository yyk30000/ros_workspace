#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from utils import BEVTransform,CURVEFIt,draw_lane_img ,purePursuit,STOPLineEstimator


class IMGParser:
    def __init__(self):
        self.image_sub =rospy.Subscriber("/image_jpeg/compressed",CompressedImage,self.callback)

        self.img_wlane=None


    def callback(self,msg):
        try:
            np_arr = np.fromstring(msg.data,np.uint8)
            img_bgr =cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.img_hsv =cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)

        lower_wlane=np.array([0,0,220])
        upper_wlane=np.array([40,15,255])
        
        self.img_wlane=cv2.inRange(self.img_hsv,lower_wlane,upper_wlane)

        h= self.img_wlane.shape[0]
        self.img_wlane[int(h*0.7):,:]=0


        
        

if __name__ =='__main__' :
    rp = rospkg.RosPack()

    currentPath =rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath,'sensor/sensor_params.json'),'r') as fp:
        sensor_params =json.load(fp)
    
    params_cam =sensor_params["params_cam"]

    rospy.init_node('image_parser',anonymous=True)

    image_parser =IMGParser()
    bev_op =BEVTransform(params_cam=params_cam)
    sline_detector =STOPLineEstimator()
    

    rate =rospy.Rate(20)

    while not rospy.is_shutdown():
        
        
        if image_parser.img_wlane is not None:
            
           
            lane_pts =bev_op.recon_lane_pts(image_parser.img_wlane)

            # print(lane_pts)
            sline_detector.get_x_points(lane_pts)
            sline_detector.estimate_dist(0.3)
            # sline_detector.visualize_dist()
            
            sline_detector.pub_sline()
           
            # cv2.imshow("Image window ",img_warp1)
            # cv2.waitKey(1)

            rate.sleep()
    