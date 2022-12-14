#!/usr/bin/env python
import rospy

import time
import paho.mqtt.client as mqtt
broker="broker.hivemq.com"
broker="iot.eclipse.org"

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


pose = None

#define callback
def on_message(client, userdata, message):
    time.sleep(1)
    print("received message =",str(message.payload.decode("utf-8")))



def callback(odom_data): 
    rospy.sleep(1)
    curr_time = odom_data.header.stamp
    global pose
    pose = odom_data.pose.pose #  the x,y,z pose and quaternion orientation
    print(pose)
    #return pose
    mqttpublish(pose)

def mqttpublish(pose):
    rospy.sleep(1)
    client = mqtt.Client()
    client.connect("localhost",1883,60)
    client.publish("topic/test", pose);
    print("Published")
    #client.disconnect();


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ugv1', anonymous=True)

    rospy.Subscriber("state_estimation", Odometry, callback)

    #print(pose)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()    
