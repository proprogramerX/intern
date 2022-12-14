#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np


from std_msgs.msg import Header, String 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Polygon, Point32, PolygonStamped


x = 0
y = 0
z = 10


class publish:
    def __init__(self):
        self._pub = rospy.Publisher('sensor_coverage_planner/nogo_boundary', PolygonStamped, queue_size=10)

        self._nogo_boundary = [PolygonStamped(),PolygonStamped(),PolygonStamped()]

        rospy.init_node('talker', anonymous=True)
        
        current_time = rospy.Time.now()
        for i in range(2):
            self._nogo_boundary[i].header.stamp = current_time


        self.talker()

    def callback(self,data):
        for i in range(2):
            self._nogo_boundary[i].header.frame_id = data.header.frame_id

        # set the position

        x = np.float32(data.pose.pose.position.x)
        y = np.float32(data.pose.pose.position.y)
        z = np.float32(data.pose.pose.position.z)

        t1 = Point32()
        t1.x = x - 1
        t1.y = y + 1
        t1.z = z + 1

        t2 = Point32()
        t2.x = x + 1
        t2.y = y + 1
        t2.z = z + 1

        t3 = Point32()
        t3.x = x - 1
        t3.y = y - 1
        t3.z = z + 1

        t4= Point32()
        t4.x = x + 1   
        t4.y = y - 1
        t4.z = z + 1

        b1 = Point32()
        b1.x = x + 1
        b1.y = y + 1
        b1.z = z - 1

        b2 = Point32()
        b2.x = x + 1
        b2.y = y - 1
        b2.z = z - 1

        b3 = Point32()
        b3.x = x - 1
        b3.y = y + 1
        b3.z = z - 1

        b4= Point32()
        b4.x = x - 1   
        b4.y = y - 1
        b4.z = z - 1



        self._nogo_boundary[0].polygon.points = [t3, t1, t1, t2, t2, t4, t4, t3, t3, b4, t4, b2, t2, b1, t1, b3, b4, b3, b3, b1, b1, b2, b2, b4]
        #self._nogo_boundary[1].polygon.points = []
        #self._nogo_boundary[2].polygon.points = []


        # publish the message
        self._pub.publish(self._nogo_boundary[0])

        


    def talker(self):
        
        rate = rospy.Rate(10) # 10hz
        rospy.Subscriber("pong", Odometry, self.callback)





        

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()