#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
import math

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class path_pub:

    def __init__(self):
        rospy.init_node("path_pub",anonymous=True)
        self.xy={}
        self.local_x=0
        self.local_y=0
        self.local_path_pub=rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.path_pub =rospy.Publisher('/global_path',Path,queue_size=1)
        self.path_local_pub =rospy.Publisher('/path',Path,queue_size=1)
        self.path_msg =Path()
        self.path_msg.header.frame_id='/map'

        self.path_local_msg =Path()
        self.path_local_msg.header.frame_id='/map'

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('gps_common')
        full_path=pkg_path+'/path'+'/kcity.txt'
       

        rate =rospy.Rate(10)
        while not rospy.is_shutdown():
            self.path_local_msg =Path()
            self.path_local_msg.header.frame_id='/map'
            self.path_msg =Path()
            self.path_msg.header.frame_id='/map'

            self.f=open(full_path,'r')
            lines=self.f.readlines()
            pub_cut =20
            self.temp_len = 9999        
        
            for line in lines:
                tmp=line.split()
                read_pose=PoseStamped()
                read_pose.pose.position.x =float(tmp[0])
                read_pose.pose.position.y =float(tmp[1])
                read_pose.pose.orientation.w =1
                self.path_msg.poses.append(read_pose)
                
                path_len =math.sqrt((self.local_x-float(tmp[0]))**2+(self.local_y-float(tmp[1]))**2)
                if path_len<= self.temp_len :
                    self.temp_len =path_len
                    self.xy[0]=tmp[0]
                    self.xy[1]=tmp[1]
            
                # print(self.temp_len)
                # print(self.xy[0],self.xy[1])
                # print(self.local_x,self.local_y)
            self.f.close()
            
            self.f2=open(full_path,'r')


            lines2=self.f2.readlines()

            for line in lines2:
                tmp=line.split()
                # print(tmp[0],tmp[1] ,self.xy[0],self.xy[1])
                path_len =math.sqrt((self.local_x-float(tmp[0]))**2+(self.local_y-float(tmp[1]))**2)
                if (self.xy[0]==tmp[0] and self.xy[1]==tmp[1]) or pub_cut <10:
                    if path_len <20:

                        read_pose2=PoseStamped()
                        read_pose2.pose.position.x =float(tmp[0])
                        read_pose2.pose.position.y =float(tmp[1])
                        read_pose2.pose.orientation.w =1
                        self.path_local_msg.poses.append(read_pose2)
                        print(tmp[0],tmp[1],self.xy[0],self.xy[1])
                        pub_cut= pub_cut+1
                        if self.xy[0]==tmp[0] and self.xy[1]==tmp[1]:
                            pub_cut =0


            self.f2.close()
           

            self.path_pub.publish(self.path_msg)
            self.path_local_pub.publish(self.path_local_msg)
            rate.sleep()

    def odom_callback(self,msg):
        self.local_x = msg.pose.pose.position.x - 302459.942
        self.local_y = msg.pose.pose.position.y - 4122635.537


if __name__=='__main__':
    try:
        test_track=path_pub()
    except rospy.ROSInterruptException:
        pass