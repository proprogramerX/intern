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

broker="test.mosquitto.org"
#broker="broker.hivemq.com"
#broker="iot.eclipse.org"

##hashes
out_hash_md5 = hashlib.md5()
in_hash_md5 = hashlib.md5()

# for outfile as I'm rnning sender and receiver together

x = 0
y = 0
z = 0

global input

def process_message(msg):
   """ This is the main receiver code
   """
   if len(msg)==200: #is header or end
      msg_in=msg.decode("utf-8")
      msg_in=msg_in.split(",,")
      if msg_in[0]=="end": #is it really last packet?
         in_hash_final=in_hash_md5.hexdigest()
         if in_hash_final==msg_in[2]:
            print("File copied OK -valid hash  ",in_hash_final)
         else:
            print("Bad file receive   ",in_hash_final)
         return False
      else:
         if msg_in[0]!="header":
            in_hash_md5.update(msg)
            return True
         else:
            return False
   else:
      in_hash_md5.update(msg)
      return True
#define callback
def on_message(client, userdata, message):
   time.sleep(1)
   #print("received message =",str(message.payload.decode("utf-8")))
   if process_message(message.payload):
      fout.write(message.payload)
      publishUGV2()


def wait_for(client,msgType,period=0.25,wait_time=40,running_loop=False):
    client.running_loop=running_loop #if using external loop
    wcount=0  
    while True:
        #print("waiting"+ msgType)
        if msgType=="PUBACK":
            if client.on_publish:        
                if client.puback_flag:
                    return True
     
        if not client.running_loop:
            client.loop(.01)  #check for messages manually
        time.sleep(period)
        #print("loop flag ",client.running_loop)
        wcount+=1
        if wcount>wait_time:
            print("return from wait loop taken too long")
            return False
    return True 

def remove(string):
    return string.replace(" ", "")

class publishUGV2():
    def __init__(self):
        #self.pub()
        self.dictionary = {}
        self.odom_pub = rospy.Publisher ('/my_odom', Odometry, queue_size=10)
        self._odom = Odometry()
        self.header = Header()
        self.header.frame_id = 'map'
        self.genrosmsg()
        input = 0

    
    def genrosmsg(self):
        with open("UGV2Odom.msg") as file:
            for line in file:
                for i in line:
                    #print(line)
                    line.strip()
                    nospace = line.replace(" ","")
                    nonewline = nospace.replace("\n","")
                    (key, value) = nonewline.split(":",1)
                    self.dictionary[str(key)] = value
        print(self.dictionary)
        x = self.dictionary['x']
        x = numpy.float64(x)
        print(x)
        y = self.dictionary['y']
        y = numpy.float64(y)
        print(y)
        z = self.dictionary['z']
        z = numpy.float64(z)
        print(z)
        self._odom.pose.pose = Pose(Point(x, y, z), Quaternion(0,0,0,0))
        #self.header.child_frame_id = 'sensor'
        self.header.stamp = rospy.Time.now()
        self._odom.header = self.header

        self.odom_pub.publish(self._odom)

        
        
        

'''
    def pub(self):
        while not rospy.is_shutdown():
            
            p = self.dictionary[    position]
            print(p)



            self.odom.pose.pose = 
            self.odomgeometry_msgs.msgm_pub.publish(odom)
'''


if __name__ == "__main__":
    rospy.init_node('odom_pub')
    r = rospy.Rate(2)
    while (1):
        try:
            topic1="data/ugv1files"
            topic2="data/ugv2files"
            qos=2
            data_block_size=2000
            #with open("UGV2Odom.msg",'w') as file:
            #    pass
            fout=open("UGV2Odom.msg","wb") #use a different filename

            client = mqtt.Client()

            ######
            client.on_message=on_message
            #####

            #client.connect("localhost",1883,60)
            client.connect(broker)
            print(broker)
            client.loop_start() #start loop to process received messages
            print("subscribing ")
            client.subscribe(topic2,1)#subscribe
            time.sleep(3)


        except KeyboardInterrupt:
            client.disconnect() #disconnect
            client.loop_stop() #stop loop
            fout.close() #close files
    r.sleep()
