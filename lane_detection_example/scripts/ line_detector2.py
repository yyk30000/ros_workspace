#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os
import rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError
from std_msgs.msg import Float64

from utills2 import BEVTransform, CURVEFFit, draw_lane_img, purePursuit


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

        img_hsv =cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)

        lower_wlane=np.array([0,0,220])
        upper_wlane=np.array([40,15,255])

        self.img_wlane=cv2.inRange(img_hsv,lower_wlane,upper_wlane)

        

if __name__ =='__main__' :
    rp = rospkg.RosPack()

    currentPath =rp.get_path("lane_detection_example")

    with open(os.path.join(currentPath,'sensor/sensor_params.json'),'r') as fp:
        sensor_params =json.load(fp)
    
    params_cam =sensor_params["params_cam"]

    rospy.init_node('image_parser',anonymous=True)

    image_parser =IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)
    curve_learner = CURVEFFit(order=3)
    ctrller = purePursuit(lfd=0.8)

    rate = rospy.Rate(20)

    rate =rospy.Rate(30)

    while not rospy.is_shutdown():

        if image_parser.img_wlane is not None:


            lane_pts = bev_op.recon_lane_pts(image_parser.img_wlane)

            sline_detector.get_x_points(lane_pts)
            sline_detector.estimate_dist(0.3)

            sline_detector.visualize_dist()

            sline_detector.pub_sline()

          

            rate.sleep()

            