#!/usr/bin/env python

import rospy
from math import degrees
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class TurtleBot():
    def __init__(self):
        rospy.init_node('Odometry', anonymous=True)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.spin()

    def odom_callback(self, odom):
        roll, pitch, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        x = round(odom.pose.pose.position.x, 3)
        y = round(odom.pose.pose.position.y, 3)
        print "x: " + str(x) + ", y: " + str(y)
        print (round(degrees(yaw)) + 180)

if __name__ == '__main__':
    TurtleBot()
