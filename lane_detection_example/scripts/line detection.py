#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import os, rospkg


from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError



class TRAFFICDetector:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed",CompressedImage, self.callback)
        self.traffic_msg = String()
        self.signal_pub = rospy.Publisher("/traffic_light",String,queue_size=10)
        self.img_hsv = None
    


    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data,np.uint8)
            img_bgr =cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        self.img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
   
    def detect_signal(self):


        h = self.img_hsv.shape[0]
       

        lower_sig_y = np.array([15,250,250])
        upper_sig_y = np.array([35,255,255])

        lower_sig_r = np.array([0,230,230])
        upper_sig_r = np.array([15,255,255])

        lower_sig_g = np.array([20,205,205])
        upper_sig_g = np.array([75,255,255])

        img_r = cv2.inRange(self.img_hsv, lower_sig_r, upper_sig_r)

        img_y = cv2.inRange(self.img_hsv, lower_sig_y, upper_sig_y)

        img_g = cv2.inRange(self.img_hsv, lower_sig_g, upper_sig_g)

        img_r[int(h/3):,:]= 0
        img_y[int(h/3):,:]= 0
        img_g[int(h/3):,:]= 0

        pix_r =cv2.countNonZero(img_r)
        pix_y =cv2.countNonZero(img_y)
        pix_g =cv2.countNonZero(img_g)

        pix_max = np.max([pix_r, pix_y, pix_g])
        idx_s = np.argmax([pix_r, pix_y, pix_g])

        if pix_max>40:

            if idx_s==0:
                self.traffic_msg.data = "RED"
            elif idx_s==1:
                self.traffic_msg.data= "YELLOW"
            else:
                self.traffic_msg.data= "GREEN"  
        else:

            self.traffic_msg.data = "None"
    def pub_signal(self):

        self.signal_pub.publish(self.traffic_msg)                      
        

        

if __name__ =='__main__' :
    rospy.init_node('lane_detector', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)
    curve_learner = CURVEFit(order=1)

    

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if image_parser.edges is not None:
            img_warp =bev_op.warp_bev_img(image_parser.edges)
            lane_pts = bev_op.recon_lane_pts(image_parser.edges)
            
            x_pred, y_pred_l, y_pred_r = curve_learner.fit_curve(lane_pts)
            
            curve_learner.write_path_msg(x.x_pred, y_pred_l, y_pred_r)
            
            curve_learner.pub_path_msg()
            
            xyl,xyr = bev.op.project_lane2img(x_pred,y_pred_l,y_pred_r)
            
            img_warp1 = draw_lane_img(img_warp, xyl[:,0].astype(np.int32),
                                                xyl[:,1].astype(np.int32),
                                                xyr[:,0].astype(np.int32),
                                                xyr[:,1].astype(np.int32))

            cv2.imshow("Image window", img_warp1)
            cv2.waitKey(1)

            rate.sleep()                                     
            
           