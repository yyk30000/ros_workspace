#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos,sin,pi
from geometry_msgs.msg import Point32

class simple_controller :

    def __init__(self):
        rospy.init_node('simple controller', anonymous=True)
        rospy.Subscriber("/scan",LaserScan, self.laser_callback)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.pcd_pub = rospy.Publisher('Laser2pcd',PointCloud, queue_size=1)


        while not rospy.is_shutdown():
            rospy.spin()


    def laser_callback(self,msg):
        pcd=PointCloud()
        motor_msg=Float64()
        servo_msg=Float64()
        pcd.header.frame_id=msg.header.frame_id
        angle=0

        for r in msg.ranges:

            tmp_point=Point32()
            tmp_point.x=r*cos(angle)
            tmp_point.y=r*sin(angle)
           ## print(angle,tmp_point.x,tmp_point.y)
            angle=angle+(1.0/180*pi)
            if r<12 :
                pcd.points.append(tmp_point)

        left_count = 0
        right_count =0
        front_count =0
        for point in pcd.points :
            if point.x > 0 and point.x <0.5 and point.y >0 and point.y <2:
                left_count=left_count+1
            elif point.x > 0 and point.x <0.5 and point.y <0 and point.y >-2:
                right_count=right_count+1
            
            if point.x > 0 and point.x <2.5 and point.y >-0.5 and point.y <0.5:
                front_count=front_count+1

        
        if front_count < 10:
            servo_msg.data=0.5304
            motor_msg.data=3000
        elif front_count >10:
            if right_count-left_count>5:
                servo_msg.data=0.15
                motor_msg.data=1800
            elif -right_count+left_count>5:
                servo_msg.data=0.85
                motor_msg.data=1800
            else :
                motor_msg.data=1000
                servo_msg.data=0.5304
        
        else:
            motor_msg.data=3000
            servo_msg.data=0.5304


        print(left_count,right_count,front_count)


        self.motor_pub.publish(motor_msg)   
        self.servo_pub.publish(servo_msg)
        self.pcd_pub.publish(pcd)    


if __name__ =="__main__":
    try:
        test_track=simple_controller()
    except rospy.ROSInterruptException:
        pass    


       
