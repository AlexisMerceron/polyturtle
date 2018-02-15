#!/usr/bin/env python

import rospy
from math import atan, pi
from sensor_msgs.msg import Imu

class TurtleBot():
    def __init__(self):
        rospy.init_node('turn_turtlebot', anonymous=True)
        self.sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.spin()

    def raw_to_degree(self, x_lsb, y_lsb):
        x_gauss = x_lsb * 0.48828125
        y_gauss = y_lsb * 0.48828125
        if x_gauss == 0:
            if y_gauss < 0:
                return 90
            elif y_gauss >= 0:
                return 0
        d = (atan(y_gauss / x_gauss) * (180 / pi))
        if d > 360:
            d = d - 360
        elif d < 0:
            d = d + 360
        return d

    def imu_callback(self, imu):
        print '\nOrientation\n' + str(imu.orientation) + '\nAngular velocity\n' + str(imu.angular_velocity) + '\nLinear acceleration\n' + str(imu.linear_acceleration)


if __name__ == '__main__':
    TurtleBot()
