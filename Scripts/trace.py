#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from math import radians


class TurtleBot():
    def __init__(self):
        rospy.init_node('MoveTurtleBot', anonymous=False)
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.shutdown)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.rateValue = 10
        self.rate = rospy.Rate(self.rateValue)
        self.i = 0
        self.file = open('trace.txt', 'w')

    def moveForward(self, distance):
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        for x in range(distance + 2):
            self.publisher.publish(move_cmd)
            self.rate.sleep()
        self.stop()

    def moveBackward(self, distance):
        move_cmd = Twist()
        move_cmd.linear.x = -0.1
        for x in range(distance + 2):
            self.publisher.publish(move_cmd)
            self.rate.sleep()
        self.stop()

    def turnLeft(self, angle):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = radians(angle)
        for x in range(self.rateValue):
            self.publisher.publish(move_cmd)
            self.rate.sleep()
        self.stop()

    def turnRight(self, angle):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = -radians(angle)
        for x in range(self.rateValue):
            self.publisher.publish(move_cmd)
            self.rate.sleep()
        self.stop()

    def stop(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0
        self.publisher.publish(move_cmd)

    def imu_callback(self, imu):
        self.file.write(str(self.i) + ": " + str(imu.orientation) + '\n')
        self.file.write(str(self.i) + ": " + str(imu.angular_velocity) + '\n')
        self.file.write(str(self.i) + ": " + str(imu.linear_acceleration) + '\n')
        self.i += 1

    def shutdown(self):
        rospy.loginfo("Stoping")
        self.publisher.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        bot = TurtleBot()
        distance = int(input("Distance: "))
        if distance > 0:
            bot.moveForward(distance)
        elif distance < 0:
            bot.moveBackward(-distance)
    except Exception as e:
        print e
        rospy.loginfo("MoveTurtleBot node terminated.")

