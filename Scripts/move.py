#!/usr/bin/env python

import rospy
from math import radians
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TurtleBot():
	def __init__(self):
		rospy.init_node('MoveTurtleBot', anonymous=False)
		rospy.loginfo("To stop CTRL + C")
		self.rateValue = 10
		self.publisher  = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		self.subscriber = rospy.Subscriber('/move', String, self.routeCmd)
		self.rateValue = 10
		self.rate = rospy.Rate(self.rateValue)
		rospy.on_shutdown(self.shutdown)	

                self.commands = {}
                self.commands["moveForward"]  = self.moveForward
                self.commands["moveBackward"] = self.moveBackward
                self.commands["turnLeft"]     = self.turnLeft
                self.commands["turnRight"]    = self.turnRight

		rospy.spin()

	def routeCmd(self, data):
		cmd, value = data.data.split(";")
		print str(cmd) + " " + value
                self.commands[cmd](int(value))

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
		move_cmd.angular.z = radians(angle)
		for x in range(self.rateValue):
			self.publisher.publish(move_cmd)
			self.rate.sleep()
		self.stop()

	def turnRight(self, angle):
		move_cmd = Twist()
		move_cmd.angular.z = -radians(angle)
		for x in range(self.rateValue):
			self.publisher.publish(move_cmd)
			self.rate.sleep()
		self.stop()

	def stop(self):
		move_cmd = Twist()
		self.publisher.publish(move_cmd)
		
	def shutdown(self):
		rospy.loginfo("Stopping node")
		self.stop()

if __name__ == '__main__':
    try:
        TurtleBot()
    except Exception as e:
        print e
        rospy.loginfo("MoveTurtleBot node terminated.")
