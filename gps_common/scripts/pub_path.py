#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler


class velocityPlanning :
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
            for box in  range(-point_num,point_num):
                x=global_path.poses[i+box].pose.position.x
                y=global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))
            
            x_matrix=np.array(x_list)
            y_matrix=np.array(y_list)
            x_trans=x_matrix.T
            

            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)*3.6  #0.7
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
            out_vel_plan.append(self.car_max_speed)
        
        return out_vel_plan



class path_pub_tf :

    def __init__(self):
        rospy.init_node('path_pub', anonymous=True)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
        self.local_path_pub = rospy.Publisher('/path',Path, queue_size=1)
        self.vel_pub = rospy.Publisher('/target_vel',Float64, queue_size=1)
        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='/map'
        
        self.is_odom=False
        self.local_path_size=50



        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('gps_common')
        full_path=pkg_path+'/path'+'/kcity.txt'
        self.f=open(full_path,'r')
        lines=self.f.readlines()
        for line in lines :
            tmp=line.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.orientation.w=1
            self.global_path_msg.poses.append(read_pose)
        
        self.f.close()


        vel_planner=velocityPlanning(80,0.15)
        vel_profile=vel_planner.curveBasedVelocity(self.global_path_msg,100)
        # print(vel_profile)

    

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
   
            if self.is_odom == True :
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y
                min_dis=float('inf')
                current_waypoint=-1
                for i,waypoint in enumerate(self.global_path_msg.poses) :

                    distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y,2))
                    if distance < min_dis :
                        min_dis=distance
                        current_waypoint=i

                
                if current_waypoint != -1 :
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        for num in range(current_waypoint,current_waypoint + self.local_path_size ) :
                            tmp_pose=PoseStamped()
                            tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w=1
                            local_path_msg.poses.append(tmp_pose)
                    
                    else :
                        for num in range(current_waypoint,len(self.global_path_msg.poses) ) :
                            tmp_pose=PoseStamped()
                            tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                            tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                            tmp_pose.pose.orientation.w=1
                            local_path_msg.poses.append(tmp_pose)


                vel_msg=Float64()
                vel_msg.data=vel_profile[current_waypoint]
                print(vel_profile[current_waypoint])
                
                self.global_path_pub.publish(self.global_path_msg)
                self.local_path_pub.publish(local_path_msg)
                self.vel_pub.publish(vel_msg)

            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

        self.x=msg.pose.pose.position.x- 302459.942
        self.y=msg.pose.pose.position.y- 4122635.537
        
        



if __name__ == '__main__':
    try:
        test_track=path_pub_tf()
    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python
# #-*- coding: utf-8 -*-

# import rospy
# import rospkg
# from sensor_msgs.msg import LaserScan,PointCloud,Imu
# from std_msgs.msg import Float64
# from vesc_msgs.msg import VescStateStamped
# from laser_geometry import LaserProjection
# from math import cos,sin,pi,sqrt,pow
# from geometry_msgs.msg import Point32,PoseStamped
# from nav_msgs.msg import Odometry,Path
# import math
# import numpy as np
# import tf
# from tf.transformations import euler_from_quaternion,quaternion_from_euler

# class path_pub:

#     def __init__(self):
#         rospy.init_node("path_pub",anonymous=True)
#         self.xy={}
#         self.local_x=0
#         self.local_y=0
#         self.local_path_pub=rospy.Subscriber("odom", Odometry, self.odom_callback)
#         self.path_pub =rospy.Publisher('/global_path',Path,queue_size=1)
#         self.path_local_pub =rospy.Publisher('/path',Path,queue_size=1)
#         self.speed_point_pub=rospy.Publisher('/speed_point',Float64,queue_size=1)
#         self.path_msg =Path()
#         self.path_msg.header.frame_id='/map'

#         self.path_local_msg =Path()
#         self.path_local_msg.header.frame_id='/map'
#         vp=velocityPlanning(80,0.8)
#         point_speed=vp.curveBasedVelocity(self.path_msg,100)
#         rospack=rospkg.RosPack()
#         pkg_path=rospack.get_path('gps_common')
#         full_path=pkg_path+'/path'+'/kcity.txt'
       

#         rate =rospy.Rate(10)
#         while not rospy.is_shutdown():
#             self.path_local_msg =Path()
#             self.path_local_msg.header.frame_id='/map'
#             self.path_msg =Path()
#             self.path_msg.header.frame_id='/map'

#             self.f=open(full_path,'r')
#             lines=self.f.readlines()
#             pub_cut =20
#             self.temp_len = 9999        
        
#             for line in lines:
#                 tmp=line.split()
#                 read_pose=PoseStamped()
#                 read_pose.pose.position.x =float(tmp[0])
#                 read_pose.pose.position.y =float(tmp[1])
#                 read_pose.pose.orientation.w =1
#                 self.path_msg.poses.append(read_pose)
                
#                 path_len =math.sqrt((self.local_x-float(tmp[0]))**2+(self.local_y-float(tmp[1]))**2)
#                 if path_len<= self.temp_len :
#                     self.temp_len =path_len
#                     self.xy[0]=tmp[0]
#                     self.xy[1]=tmp[1]
            
#                 # print(self.temp_len)
#                 # print(self.xy[0],self.xy[1])
#                 # print(self.local_x,self.local_y)
#             self.f.close()
            
#             self.f2=open(full_path,'r')


#             lines2=self.f2.readlines()
#             tmp_num =0
#             for line in lines2:
#                 tmp=line.split()
#                 # print(tmp[0],tmp[1] ,self.xy[0],self.xy[1])
#                 path_len =math.sqrt((self.local_x-float(tmp[0]))**2+(self.local_y-float(tmp[1]))**2)
#                 if (self.xy[0]==tmp[0] and self.xy[1]==tmp[1]) or pub_cut <10:
#                     if self.xy[0]==tmp[0] and self.xy[1]==tmp[1]:

#                     if path_len <20:

#                         read_pose2=PoseStamped()
#                         read_pose2.pose.position.x =float(tmp[0])
#                         read_pose2.pose.position.y =float(tmp[1])
#                         read_pose2.pose.orientation.w =1
#                         self.path_local_msg.poses.append(read_pose2)
#                         print(tmp[0],tmp[1],self.xy[0],self.xy[1])
#                         pub_cut= pub_cut+1
#                         if self.xy[0]==tmp[0] and self.xy[1]==tmp[1]:
#                             pub_cut =0
#                 tmp_num=tmp_num+1

#             self.f2.close()
           
            
           
            
#             self.speed_point_pub.publish(point_speed[])
#             self.path_pub.publish(self.path_msg)
#             self.path_local_pub.publish(self.path_local_msg)
#             rate.sleep()

#     def odom_callback(self,msg):
#         self.local_x = msg.pose.pose.position.x - 302459.942
#         self.local_y = msg.pose.pose.position.y - 4122635.537

# class velocityPlanning:
#     def __init__(self,car_max_speed,road_friction):
#         self.car_max_speed =car_max_speed
#         self.road_friction =road_friction

#     def curveBasedVelocity(self,global_path,point_num):
#         out_vel_plan =[]
#         for i in range(0,point_num):
#             out_vel_plan.append(self.car_max_speed)
#         for i in range(point_num,len(global_path.poses)-point_num):
#             x_list =[]
#             y_list =[]
#             for box in range(-point_num,point_num):
#                 x=global_path.poses[i+box].pose.position.x
#                 y=global_path.poses[i+box].pose.position.y
#                 x_list.append([-2*x,-2*y,1])
#                 y_list.append(-(x*x)-(y*y))

#             x_matrix =np.array(x_list)
#             y_matrix =np.array(y_list)
#             x_trans =x_matrix.T

#             a_matrix =np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
#             a=a_matrix[0]
#             b=a_matrix[1]
#             c=a_matrix[2]
#             r=sqrt(a*a+b*b-c)

#             v_max =sqrt(r*9.8*self.road_friction)*3.6 
#             if v_max>self.car_max_speed:
#                 v_max =self.car_max_speed
#             out_vel_plan.append(v_max)

#         for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
#             out_vel_plan.append(self.car_max_speed)

#         return out_vel_plan





# if __name__=='__main__':
#     try:
#         test_track=path_pub()
#     except rospy.ROSInterruptException:
#         pass