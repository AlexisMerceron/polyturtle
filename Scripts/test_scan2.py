#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt

"""
float32 angle_min           start angle of the scan [rad]
float32 angle_max           end angle of the scan [rad]
float32 angle_increment     angular distance between measurements [rad]
float32 time_increment      time between measurements [seconds]
float32 scan_time           time between scans [seconds]
float32 range_min           minimum range value [m]
float32 range_max           maximum range value [m]
float32[] ranges            range data [m]
float32[] intensities       intensity data [device-specific units]
"""

def callback(data):
    print '\n'
    print "angle_min: %s" % data.angle_min
    print "angle_max: %s" % data.angle_max
    print "angle_increment: %s" % data.angle_increment
    print "time_increment: %s" % data.time_increment
    print "scan_time: %s" % data.scan_time
    print "range_min: %s" % data.range_min
    print "range_max: %s" % data.range_max

    xs = []
    ys = []
    for i in range(len(data.ranges)):
        #print "%d: (%f, %f)" % (i, data.ranges[i], data.intensities[i])
        x = -math.sin(math.radians(i)) * data.ranges[i]
        y = math.cos(math.radians(i)) * data.ranges[i]
        #print "(%f, %f)" % (x, y)
        xs.append(x)
        ys.append(y)

    plt.scatter(xs, ys)
    plt.scatter([0], [0], color='r')
    plt.show()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/zeus/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
