#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class gps_odom:

    def __init__(self):
        self.odom_msg=Odometry()
        rospy.init_node('gps_odom',anonymous=True)
        rospy.Subscriber("/imu",Imu,self.imu_callback)
        rospy.Subscriber("/odom",Odometry,self.odom_callback)

    
        self.is_imu=False
        rate =rospy.Rate(20)
        while not rospy.is_shutdown():

            if self.is_imu==True :
            
                quaternion = self.imu_quaternion

                br=tf.TransformBroadcaster()
                br.sendTransform((self.odom_msg.pose.pose.position.x-302459.942,self.odom_msg.pose.pose.position.y-4122635.537,0),
                quaternion,
                rospy.Time.now(),
                "base_link",
                "odom")

            rate.sleep()


    def imu_callback(self,msg):
        self.is_imu=True
        self.imu_quaternion=(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
    

    def odom_callback(self,msg):
        self.odom_msg =msg


if __name__ =='__main__':
    try:
        test_track =gps_odom()
    except rospy.ROSInterruptException:
        pass