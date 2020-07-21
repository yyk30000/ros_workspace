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
        self.image_sub =rospy.Subscriber("/imag_jpeg/compressed",CompressedImage,self.callback)

        self.img_wlane=None


    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data,np.uint8)
            img_bgr =cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.img_hsv = cv2.cvtColor(img_bgr,cv2.COLOR_BGR2HSV)

        cv2.namedWindow('mouseRGB')

        cv2.imshow('mouseRGB', self.mouseRGB)

        cv2.waitKey(1)
    def mouseRGB(self,event,x,y,flags,param):    
        if event == cv2.EVENT_LBUTTONDOWN:
            colorsB = self.img_hsv[y,x,0]
            colorsG = self.img_hsv[y,x,1]
            colorsR = self.img_hsv[y,x,2]
            colors = self.img_hsv[y,x]
            print("Red: ",colorsR)
            print("Green",colorsG)
            print("Blue",colorsB)
            print("BGR Format",colors)
            print("Coordinates of pixel: x:",x,"y: ",y)
        

        

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
    curve_learner = CURVEFIt(order=3)
    ctrler = purePursuit(lfd=0.8)

    rate =rospy.Rate(20)

    while not rospy.is_shutdown():
        
        
        if image_parser.img_wlane is not None:

            img_warp =bev_op.warp_bev_img(image_parser.img_wlane)
            lane_pts =bev_op.recon_lane_pts(image_parser.img_wlane)

            # print(lane_pts)
            # sline_detector.get_x_points(lane_pts)
            # sline_detector.estimate_dist(0.3)
            # sline_detector.visualize_dist()
            # sline_detector.pub_sline()
           
            x_pred,y_pred_l,y_pred_r =curve_learner.fit_curve(lane_pts)

            ctrler.steering_angle(x_pred,y_pred_l,y_pred_r)
            ctrler.pub_cmd()
            

            xyl,xyr = bev_op.project_lane2img(x_pred,y_pred_l,y_pred_r)

            img_warp1 = draw_lane_img(img_warp, xyl[:,0].astype(np.int32),
                                                xyl[:,1].astype(np.int32),
                                                xyr[:,0].astype(np.int32),
                                                xyr[:,1].astype(np.int32))
            
           
            img_lane = cv2.cvtColor(image_parser.img_wlane, cv2.COLOR_GRAY2BGR)
            img_concat = np.concatenate([img_lane, img_warp], axis=1)
           
           

            rate.sleep()
    