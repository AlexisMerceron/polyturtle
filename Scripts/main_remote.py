#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import radians


class TurtleBot():
    def __init__(self, namespace):
        self.namespace = namespace
        rospy.init_node('MainTurtleBot', anonymous=False)
        rospy.loginfo("To stop CTRL + C")
        self.pub = rospy.Publisher(self.namespace + '/move', String, queue_size=10)
        self.rate = rospy.Rate(10)

    def moveForward(self, distance):
        i = String("moveForward;" + str(distance))
        self.pub.publish(i)
        self.rate.sleep()

    def moveBackward(self, distance):
        i = String("moveBackward;" + str(distance))
        self.pub.publish(i)
        self.rate.sleep()

    def turnLeft(self, angle):
        i = String("turnLeft;" + str(angle))
        self.pub.publish(i)
        self.rate.sleep()

    def turnRight(self, angle):
        i = String("turnRight;" + str(angle))
        self.pub.publish(i)
        self.rate.sleep()



if __name__ == '__main__':
    try:
        import time
        bot = TurtleBot("/hades")
        rospy.sleep(0.5)
        bot.moveForward(10)
        #bot.turnLeft(90)
        #bot.moveForward(10)
        #bot.turnRight(90)
        #bot.moveBackward(10)
        #bot.turnRight(90)
        #bot.moveForward(10)
        #bot.turnLeft(90)
    except Exception as e:
        print e
        rospy.loginfo("MainTurtleBot node terminated.")

