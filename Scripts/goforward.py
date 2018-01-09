#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist



def shutdown():
    rospy.loginfo("Stoping")
    pub.publish(Twist())
    rospy.sleep(1)

class TurtleBot():
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.init_node('MoveTurtlebot', anonymous=False)
        rospy.loginfo("CTRL+C to stop")
        rospy.on_shutdown(self.shutdown)
        self.rateValue = 5
        self.rate = rospy.Rate(self.rateValue)

    def moveForward(self, speed, duration):
        move_cmd = Twist()
        move_cmd.linear.x = speed
        for x in range(self.rateValue * duration):
            self.pub.publish(move_cmd)
            self.rate.sleep()
        self.stop()

    def moveBackward(self, speed, duration):
        move_cmd = Twist()
        move_cmd.linear.x = -speed
        for x in range(self.rateValue * duration):
            self.pub.publish(move_cmd)
            self.rate.sleep()
        self.stop()

    def turnLeft(self, angle):
        move_cmd = Twist()
        move_cmd.angular.z = radians(angle)
        for x in range(self.rateValue):
            self.pub.publish(move_cmd)
            self.rate.sleep()
        self.stop()

    def turnRight(self, angle):
        move_cmd = Twist()
        move_cmd.angular.z = radians(-angle)
        for x in range(self.rateValue):
            self.pub.publish(move_cmd)
            self.rate.sleep()
        self.stop()

    def stop(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.angular.z = 0

    def shutdown(self):
        rospy.loginfo("Stoping bot")
        self.pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        bot = TurtleBot()
        bot.moveForward(0.02, 1)
        bot.turnLeft(90)
        bot.moveForward(0.02, 1)
        bot.turnRight(90)
        bot.moveBackward(0.02, 1)
    except:
        rospy.loginfo("MoveTurtlebot node terminated.")

