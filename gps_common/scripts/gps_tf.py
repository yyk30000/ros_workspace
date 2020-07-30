#! /usr/bin/env python

import rospy
import numpy as numpy
import tf 

from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from morai_msgs.msg import GPSMessage
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped,Quaternion,TwistStamped

class GPS_TF:
    def __init__(self):
        self.msg =NavSatFix()

        self.gps_sub= rospy.Subscriber("/gps",GPSMessage,self.gps_callback)
        self.gps_pub= rospy.Publisher("/fix",NavSatFix,queue_size=1)

    def gps_callback(self,gps_msg):
        self.msg.header.frame_id =gps_msg.header.frame_id
        self.msg.header.stamp = rospy.Time.now()
        self.msg.latitude =gps_msg.latitude
        self.msg.longitude =gps_msg.longitude
        self.msg.altitude = gps_msg.altitude


        self.gps_pub.publish(self.msg)



if __name__ =="__main__":
    rospy.init_node('gps_tf',anonymous=True)

    gps_tf=GPS_TF()

    rospy.spin()

