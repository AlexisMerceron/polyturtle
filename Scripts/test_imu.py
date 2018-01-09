#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

def callback(data):
    print '\nOrientation\n' + str(data.orientation) + '\nAngular velocity\n' + str(data.angular_velocity) + '\nLinear acceleration\n' + str(data.linear_acceleration)

def listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber("/imu", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
