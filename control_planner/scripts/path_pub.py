#!/usr/bin/env python
#-*- coding: utf-8 -*-
import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
class velocityPlanning:
    def __init__(self,car_max_speed,road_friction):
        self.car_max_speed=car_max_speed
        self.road_friction=road_friction

    def curveBasedVelocity(self,global_path,point_num):
        out_vel_plan=[]
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)
        for i in range(point_num,len(global_path.poses)-point_num):
            x_list=[]
            y_list=[]
            for box in range(-point_num,point_num):
                x=global_path.poses[i+box].pose.position.x
                y=global_path.poses[i+box].pose.position.y
                x_list.append([-2*x, 2*y,1]) 
                y_list.append(-(x*x)-(y*y))

            x_matrix=np.array(x_list)      
            y_matrix=np.array(y_list)  
            x_trans=x_matrix.T

            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y.matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)*3.6
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
            out_vel_plan.append(self.car_max_speed)
        return out_vel_plan        

class path_pub:

    def __init__(self):
        rospy.init_node("path_pub",anonymous=True)

        self.path_pub =rospy.Publisher('/path',Path,queue_size=1)
        self.path_msg =Path()
        self.path_msg.header.frame_id='/map'

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('control_planner')
        full_path=pkg_path+'/path'+'/kcity.txt'
        self.f=open(full_path,'r')
        lines=self.f.readlines()
        for line in lines:
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.orientation.w =1
            self.path_msg.poses.append(read_pose)

        self.f.close()

        rate =rospy.Rate(10)
        while not rospy.is_shutdown():
            self.path_pub.publish(self.path_msg)
            rate.sleep()


if __name__=='__main__':
    try:
        test_track=path_pub()
    except rospy.ROSInterruptException:
        pass

 