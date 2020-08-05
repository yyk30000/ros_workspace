#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import rospkg
import string
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import os

class point_pub:

    def __init__(self):
        rospy.init_node("point_pub",anonymous=True)

        self.path_pub =rospy.Publisher('/pointcloud',PointCloud,queue_size=1)
        self.path_msg =PointCloud()
        self.path_msg.header.frame_id='map'
        self.path_msg.header.stamp= rospy.Time.now()

        
        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('gps_common')
    
        full_path=pkg_path+'/map'
        file_list=os.listdir(full_path)
        for i in file_list:
            file_path=full_path+'/'+i
            print(file_path)
            self.f=open(file_path,'r')
            lines=self.f.readlines()
        
            cut =0

            for line in lines:
                tmp=line.split()
                print(cut)
                if cut > 7:
                    read_pose=Point32()
                    read_pose.x=float(tmp[0])-302459.942
                    read_pose.y =float(tmp[1])-4122635.537
                    read_pose.z =0
                    
                    self.path_msg.points.append(read_pose)
                cut=cut+1
            self.f.close()


        rate =rospy.Rate(10)
        while not rospy.is_shutdown():
            self.path_pub.publish(self.path_msg)
            rate.sleep()


if __name__=='__main__':
    try:
        test_track=point_pub()
    except rospy.ROSInterruptException:
        pass

 