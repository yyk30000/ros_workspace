#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
name_node = 'controller'

class PController:
    def __init__(self, ref =10, T =0.1 ,K=1):

        self.T = T
        self.ref = ref
        self.K = K
        self.ctrl_msgs =Float32()
        self.ctrl_topic = '/pid_ctrl'
        self.measure_topic = '/measure_plant'

        self.y = None

        self.sub = rospy.Subscriber(self.measure_topic,Float32, self.measure_callback)
        self.pub = rospy.Publisher(self.ctrl_topic,Float32, queue_size = 10)

    def measure_callback(self,msgs):
        self.y = msgs.data 

    def pub_ctrl_msgs(self):
        err = self.ref - self.y
        u = self.K * err
        self.cntl_msgs.data = u 
        self.pub.publish(self.ctrl_msgs)

 if __name__ == '__main__'  :
    rospy.init_node(name_node) 

    plant =PLANT()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        plant.pub_measure_msgs()

        rate.sleep()          
