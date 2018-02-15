#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import radians


class TurtleBot():
    def __init__(self):
        rospy.init_node('MoveTurtleBot', anonymous=False)
        rospy.loginfo("To stop TurtleBot CTRL + C")
        rospy.on_shutdown(self.shutdown)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.rateValue = 5
        self.rate = rospy.Rate(self.rateValue)

    def moveForward(self, distance):
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        for x in range(self.rateValue * distance / 10):
            self.publisher.publish(move_cmd)
            self.rate.sleep()
        self.stop()

    def moveBackward(self, distance):
        move_cmd = Twist()
        move_cmd.linear.x = -0.1
        for x in range(self.rateValue * distance / 10):
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

    def shutdown(self):
        rospy.loginfo("Stoping")
        self.publisher.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        bot = TurtleBot()
        bot.moveForward(10)
        bot.turnLeft(90)
        bot.moveForward(10)
        bot.turnRight(90)
        bot.moveBackward(10)
        bot.turnRight(90)
        bot.moveForward(10)
        bot.turnLeft(90)
    except Exception as e:
        print e
        rospy.loginfo("MoveTurtleBot node terminated.")

