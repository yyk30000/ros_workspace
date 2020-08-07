#!/usr/bin/env python
#-*- coding: utf-8 -*-

#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point32,PoseStamped,Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
import math

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class Pure_pursuit:

    def __init__(self):
        rospy.init_node("make_path",anonymous=True)
        rospy.Subscriber("path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)
        rospy.Subscriber("/sensors/core",VescStateStamped,self.core_callback)
        rospy.Subscriber("/get_speed",Float64,self.get_speed_callback)
        #rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        self.motor_pub = rospy.Publisher("commands/motor/speed", Float64, queue_size=1)
        self.servo_pub = rospy.Publisher("commands/servo/position", Float64,queue_size=1)
        self.motor_msg=Float64()
        self.servo_msg=Float64()
        self.is_path=False
        self.is_odom=False
        self.is_amcl=False
        self.is_core=False

        self.forward_point=Point()
        self.current_position=Point()
        self.is_look_forward_point=False
        self.vihicle_length=1
        self.lfd=5
        self.steering=0
        self.check_point=-1
        self.car_speed =0 
        self.get_speed =5000

        self.steering_angle_to_servo_gain = -1.2135
        self.steering_angle_to_servo_offset=0.5304
        
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            if self.is_path ==True and (self.is_odom==True or self.is_amcl==True) :

                vehicle_position=self.current_position
                rotated_point=Point()
                self.is_look_forward_point=False

                for num,i in enumerate(self.path.poses) :
                    path_point=i.pose.position
                    dx= path_point.x - vehicle_position.x
                    dy= path_point.y - vehicle_position.y
        
                    rotated_point.x=cos(self.vihicle_yaw)*dx +sin(self.vihicle_yaw)*dy
                    rotated_point.y=sin(self.vihicle_yaw)*dx - cos(self.vihicle_yaw)*dy
                    if self.car_speed < 30000:
                        self.lfd = 5
                    elif self.car_speed >=30000:
                        self.lfd = self.car_speed *0.0002
                    if rotated_point.x>0 :
                        dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                        if dis>= self.lfd and num >= self.check_point:
                            self.forward_point=path_point
                            self.is_look_forward_point=True
                            self.check_point=num
                            break

                theta=-atan2(rotated_point.y,rotated_point.x)
                if self.is_look_forward_point :
                    self.steering=atan2((2*self.vihicle_length*sin(theta)),self.lfd) 
                    myradians = math.atan2(self.path.poses[1].pose.position.x -self.path.poses[len(self.path.poses)-1].pose.position.x,self.path.poses[1].pose.position.y -self.path.poses[len(self.path.poses)-1].pose.position.y)
                    print(self.steering*180/pi)
                    print(self.check_point)
                    print(self.lfd)
                    print(myradians)
                    if theta < 0.1 and theta >-0.1:
                        self.get_speed =100000
                    else:
                        self.get_speed =30000
                    self.motor_msg.data=self.get_speed

                else :
                    self.steering=0
                    print("no found forward point")
                    self.motor_msg.data=0

                
                self.steering_command=(self.steering_angle_to_servo_gain*self.steering)+self.steering_angle_to_servo_offset
                self.servo_msg.data=self.steering_command

                self.servo_pub.publish(self.servo_msg)
                self.motor_pub.publish(self.motor_msg)
            else:
                print("no Path")
        
            rate.sleep()

        
    def path_callback(self,msg):
        self.is_path=True
        self.path=msg
    
    def odom_callback(self,msg):
        self.is_odom=True
        # odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        # _,_,self.vihicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_position.x=msg.pose.pose.position.x-302459.942
        self.current_position.y=msg.pose.pose.position.y-4122635.537
    def amcl_callback(self,msg):
        self.is_amcl=True
        amcl_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vihicle_yaw=euler_from_quaternion(amcl_quaternion)
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
    
    def imu_callback(self,msg):
        self.is_imu=True
        self.imu_quaternion=(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        _,_,self.vihicle_yaw=euler_from_quaternion(self.imu_quaternion)
    def core_callback(self,msg):
        is_core=True
        self.car_speed =msg.state.speed
    def get_speed_callback(self,msg):
        self.get_speed =msg.data


if __name__ =="__main__":
    try:
        test_track=Pure_pursuit()
    except rospy.ROSInterruptException:
        pass    


       
