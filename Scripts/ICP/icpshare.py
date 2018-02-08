#!/usr/bin/env python

import icp
import numpy as np
import rospy
from math import radians, sin, cos
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class TurtleBot():
    def __init__(self, namespace, other_ns):
        self.namespace = namespace
        self.other_ns = other_ns
	rospy.init_node('ICPTurtleBot2', anonymous=False)
	rospy.loginfo("To stop CTRL + C")
	self.rateValue = 10
	self.pub_icp  = rospy.Publisher(self.namespace + '/icpshare', String, queue_size=self.rateValue)
	self.sub_icp = rospy.Subscriber(self.other_ns + '/icpshare', String, self.getIcp)
	self.sub_scan = rospy.Subscriber(self.namespace + '/scan', LaserScan, self.getScan)
	self.rate = rospy.Rate(self.rateValue)
	rospy.on_shutdown(self.shutdown)	
   
        self.points = []
        self.shared_points = []

        self.skipScan = 0
   
	rospy.spin()
   
    def getScan(self, data):
        if self.skipScan == 0:
            xy = ""
            for i in range(len(data.ranges)):
                x = -sin(radians(i)) * data.ranges[i]
   	        y = cos(radians(i)) * data.ranges[i]
                self.points.append([x, y])
                xy += str(x) + "," + str(y) + ";"
            self.pub_icp.publish(xy)
            self.rate.sleep()
            self.skipScan += 1
        else:
            self.skipScan = (self.skipScan + 1) % 10

    def getIcp(self, data):
        points = data.data.split(';')
        for p in points[:-1]:
            point = p.split(',')
            x = float(point[0])
            y = float(point[1])
            self.shared_points.append([x, y])

        A = np.array(self.points)
        B = np.array(self.shared_points)

        T, _, _, R, Tr = icp.icp(B, A, max_iterations=20, tolerance=0.000001)
        C = np.ones((len(B), 3))
        C[:,0:2] = np.copy(B)
        C = np.dot(T, C.T).T

        print(R)
        print(T)
    
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter([x[0] for x in A], [y[1] for y in A], c='red')
        ax.scatter([x[0] for x in B], [y[1] for y in B], c='blue')
        ax.scatter([x[0] for x in C], [y[1] for y in C], c='green')
        plt.show()
   		
    def shutdown(self):
        rospy.loginfo("Stopping node")

if __name__ == '__main__':
    TurtleBot("/hades", "/zeus")
