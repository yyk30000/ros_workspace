#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import math
from time import sleep

class simple_gps:

    def __init__(self):
        rospy.init_node('simple_gps', anonymous=True)
        self.is_odom=False
        self.odom_msg =Odometry()
        rospy.Subscriber("/odom",Odometry,self.status_callback)
        self.x1= 0
        self.y1= 0
        self.x2=self.odom_msg.pose.pose.position.x
        self.y2=self.odom_msg.pose.pose.position.y
        

        
        

      
        self.theta=0


        rate =rospy.Rate(20)
        
       
        while not rospy.is_shutdown():
            if self.is_odom ==True:
                             
                
                
                self.x2=self.odom_msg.pose.pose.position.x
                self.y2=self.odom_msg.pose.pose.position.y
                dx =self.x2-self.x1
                dy =self.y2-self.y1

                dis =math.sqrt(dx*dx +dy*dy)

                if dis > 2:       
                    
                
                    self.theta = math.atan2(dx,dy)
                    print(self.theta)
                    quaternion =quaternion_from_euler(0,0,self.theta)

                    br=tf.TransformBroadcaster()
                    br.sendTransform((self.odom_msg.pose.pose.position.x-302459.942,self.odom_msg.pose.pose.position.y-4122635.537,0),
                    quaternion,
                    rospy.Time.now(),
                    "base_link",
                    "odom")
                    self.x1=self.x2
                    self.y1=self.y2



            rate.sleep()

    def status_callback(self,msg):
       self.is_odom =True
       self.odom_msg =msg

    
    
    
if __name__ =="__main__":
    try:
        test_track=simple_gps()
    except rospy.ROSInterruptException:
        pass    