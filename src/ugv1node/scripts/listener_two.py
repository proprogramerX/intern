#! /usr/bin/env python

import hashlib
import time

import numpy
import paho.mqtt.client as mqtt
import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class node:
    def callback(odom_data):
        rospy.sleep(1)
        curr_time = odom_data.header.stamp
        global pose
        pose = odom_data#  the x,y,z pose and quaternion orientation
        pub = rospy.Publisher('Ping', Odometry, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pub.publish(odom_data)
            rate.sleep()

    def talker():
        rospy.init_node('UGV1', anonymous=True)
        sub = rospy.Subscriber("state_estimation", Odometry, node.callback)
        
        

if __name__ == "__main__":
    while (1):
        try:
            node.talker()

        except rospy.ROSInterruptException:
            pass
