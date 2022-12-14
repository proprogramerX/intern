#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import time

import paho.mqtt.client as mqtt
import hashlib
broker="test.mosquitto.org"
#broker="broker.hivemq.com"
#broker="iot.eclipse.org"

##hashes
out_hash_md5 = hashlib.md5()
in_hash_md5 = hashlib.md5()

# for outfile as I'm rnning sender and receiver together

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

def on_publish(client, userdata, mid):
    #logging.debug("pub ack "+ str(mid))
    client.mid_value=mid
    client.puback_flag=True  


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

def send_header(filename):
   header="header"+",,"+filename+",,"
   header=bytearray(header,"utf-8")
   header.extend(b','*(200-len(header)))
   print(header)
   c_publish(client,topic1,header,qos)

def send_end(filename):
   end="end"+",,"+filename+",,"+out_hash_md5.hexdigest()
   end=bytearray(end,"utf-8")
   end.extend(b','*(200-len(end)))
   print(end)
   c_publish(client,topic1,end,qos)

def c_publish(client,topic1,out_message,qos):
   res,mid=client.publish(topic1,out_message,qos)#publish
   if res==0: #published ok
      if wait_for(client,"PUBACK",running_loop=True):
         if mid==client.mid_value:
            print("match mid ",str(mid))
            client.puback_flag=False #reset flag
         else:
            raise SystemExit("not got correct puback mid so quitting")
         
      else:
         raise SystemExit("not got puback so quitting")

def mqttpublish(pose):
    rospy.sleep(1)
    client.publish("topic/test", pose);
    print("Published")



class SavePoses(object):
    def __init__(self):
        
        self._pose = Pose()
        self.poses_dict = {"pose1":self._pose}
        self._pose_sub = rospy.Subscriber('state_estimation', Odometry , self.sub_callback)
        self.write_to_file()

    def sub_callback(self, msg):
        
        self._pose = msg.pose.pose.position
    
    def write_to_file(self):
        
        time.sleep(5)
        self.poses_dict["pose1"] = self._pose
        rospy.loginfo("Written pose1")
            
        
        with open('UGV1Odom.msg', 'w') as file:
            #file.write("UGV1\n")
            for key, value in self.poses_dict.items():
                if value:
                    file.write(str(value))
                    
        rospy.loginfo("Written all Poses to poses.txt file")
        

if __name__ == "__main__":
    while (1):
        try:
            rospy.init_node('spot_recorder', log_level=rospy.INFO) 
            save_spots_object = SavePoses()
            #rospy.spin() # mantain the service open.


            filename="UGV1Odom.msg" #file to send
            topic1="data/ugv1files"
            topic2="data/ugv2files"
            qos=1
            data_block_size=2000
            fo=open(filename,"rb")
            #fout=open("UGV2Odom.msg","wb") #use a different filename

            client = mqtt.Client()

            ######
            #client.on_message=on_message
            client.on_publish=on_publish
            client.puback_flag=False #use flag in publish ack
            client.mid_value=None
            #####

            #client.connect("localhost",1883,60)
            client.connect(broker,1883)
            print(broker)
            client.loop_start() #start loop to process received messages
            #print("subscribing ")
            #client.subscribe(topic2)#subscribe
            #time.sleep(2)
            start=time.time()
            print("publishing ")
            send_header(filename)
            Run_flag=True
            count=0


            while Run_flag:
                chunk=fo.read(data_block_size) # change if want smaller or larger data blcoks
                if chunk:
                    out_hash_md5.update(chunk)
                    out_message=chunk
                    #print(" length =",type(out_message))
                    c_publish(client,topic1,out_message,qos)
                        
                else:
                    #send hash
                    out_message=out_hash_md5.hexdigest()
                    send_end(filename)
                    #print("out Message ",out_message)
                    res,mid=client.publish("data/ugv1files",out_message,qos=1)#publish
                    Run_flag=False
                time_taken=time.time()-start
                print("took ",time_taken)
                time.sleep(3)

            ##
        except KeyboardInterrupt:
            client.disconnect() #disconnect
            client.loop_stop() #stop loop
            #fout.close() #close files
            fo.close()

