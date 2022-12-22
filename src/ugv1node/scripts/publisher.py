#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np


from std_msgs.msg import Header, String, Float32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Polygon, Point32, PolygonStamped

import subprocess
import signal
import os

x = 0
y = 0
z = 10


class publish:
     
    def __init__(self):
        self._pub = rospy.Publisher('sensor_coverage_planner/navigation_boundary', PolygonStamped, queue_size=10)

        self._nogo_boundary = [PolygonStamped(),PolygonStamped(),PolygonStamped()]

        rospy.init_node('talker', anonymous=True)
        
        current_time = rospy.Time.now()
        for i in range(2):
            self._nogo_boundary[i].header.stamp = current_time
        
        
        time_duration = 0
        start_time_duration = 0
        first_iteration = 'True'

        explored_volume = 0;
        traveling_distance = 0;
        run_time = 0;
        max_explored_volume = 0
        max_traveling_diatance = 0
        max_run_time = 0

        time_list1 = np.array([])
        time_list2 = np.array([])
        time_list3 = np.array([])
        run_time_list = np.array([])
        explored_volume_list = np.array([])
        traveling_distance_list = np.array([])
        self.explorationstate = False
        self.stop = False


        self.talker()

    def callback(self,data):
        for i in range(2):
            self._nogo_boundary[i].header.frame_id = data.header.frame_id

        # set the position

        x = np.float32(data.pose.pose.position.x)
        y = np.float32(data.pose.pose.position.y)
        z = np.float32(data.pose.pose.position.z)

        t1 = Point32()
        t1.x = x - 2
        t1.y = y + 2
        t1.z = z + 2

        t2 = Point32()
        t2.x = x + 2
        t2.y = y + 2
        t2.z = z + 2

        t3 = Point32()
        t3.x = x - 2
        t3.y = y - 2
        t3.z = z + 2

        t4= Point32()
        t4.x = x + 2   
        t4.y = y - 2
        t4.z = z + 2

        b1 = Point32()
        b1.x = x + 2
        b1.y = y + 2
        b1.z = z - 2

        b2 = Point32()
        b2.x = x + 2
        b2.y = y - 2
        b2.z = z - 2

        b3 = Point32()
        b3.x = x - 2
        b3.y = y + 2
        b3.z = z - 2

        b4= Point32()
        b4.x = x - 2   
        b4.y = y - 2
        b4.z = z - 2



        self._nogo_boundary[0].polygon.points = [t3, t1, t1, t2, t2, t4, t4, t3, t3, b4, t4, b2, t2, b1, t1, b3, b4, b3, b3, b1, b1, b2, b2, b4]
        #self._nogo_boundary[1].polygon.points = []
        #self._nogo_boundary[2].polygon.points = []


        # publish the message
        self._pub.publish(self._nogo_boundary[0])

        


    def talker(self):
        
        rate = rospy.Rate(10) # 10hz
        rospy.Subscriber("pong", Odometry, self.callback)
        rospy.Subscriber("/runtime", Float32, self.runTimeCallback)
        rospy.Subscriber("/explored_volume", Float32, self.exploredVolumeCallback)
        rospy.Subscriber("/traveling_distance", Float32, self.travelingDistanceCallback)
        rospy.Subscriber("/sensor_coverage_planner/exploration_finish", Bool, self.explorationstateCallback)
        rospy.Subscriber("/sensor_coverage_planner/stop", Bool, self.stopCallback)



    def runTimeCallback(self,msg):
        global run_time
        run_time = msg.data

    def exploredVolumeCallback(self,msg):
        global explored_volume
        explored_volume = msg.data

    def travelingDistanceCallback(self,msg):
        global traveling_distance
        traveling_distance = msg.data

    def explorationstateCallback(self,msg):
        self.explorationstate = msg.data


    def get_pids_by_tty(self,tty):
        pids = []
        output = subprocess.run(['ps', '-eo', 'tty,pid'], stdout=subprocess.PIPE).stdout.decode('utf-8')
        for line in output.split('\n'):
            if tty in line:
                pids.append(int(line.split()[1]))
        return pids

    def stopCallback(self,msg):
        self.stop = msg.data
        if (self.stop):
            print(traveling_distance)
            print(explored_volume)
            print(run_time)
            tty = subprocess.run(['tty'], stdout=subprocess.PIPE).stdout.decode('utf-8').strip()
            pids = self.get_pids_by_tty(tty)
            for pid in pids:
                os.kill(pid, signal.SIGINT)

        





        

if __name__ == '__main__':
    try:
        rosnode = publish()
        #screen_session = os.getenv('STY')

        # Get the process id
        #pid = p.pid
        # if (rosnode.explorationstate):
        #     print(traveling_distance)
        #     print(explored_volume)
        #     print(run_time)
        #     tty = subprocess.run(['tty'], stdout=subprocess.PIPE).stdout.decode('utf-8').strip()
        #     pids = get_pids_by_tty(tty)
        #     for pid in pids:
        #         os.kill(pid, signal.SIGINT)

        #     if not p.poll():
        #         print("Process correctly halted")

    except rospy.ROSInterruptException:
        pass
    rospy.spin()