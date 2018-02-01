#!/usr/bin/env python

import rospy
from math import radians, degrees
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class TurtleBot():
	def __init__(self):
		rospy.init_node('MoveTurtleBot', anonymous=False)
		rospy.loginfo("To stop CTRL + C")
		self.rateValue = 10
		self.publisher  = rospy.Publisher('/cmd_vel', Twist, queue_size=self.rateValue)
		self.sub_move = rospy.Subscriber('/move', String, self.routeCmd)
		self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		self.rate = rospy.Rate(self.rateValue)
		rospy.on_shutdown(self.shutdown)	

                self.commands = {}
                self.commands["moveForward"]  = self.moveForward
                self.commands["moveBackward"] = self.moveBackward
                self.commands["turnLeft"]     = self.turnLeft
                self.commands["turnRight"]    = self.turnRight
		
		self.x = 0
		self.y = 0
		self.angle = 0

		rospy.spin()

	def odom_callback(self, odom):
		roll, pitch, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
		self.angle = round(degrees(yaw)) + 180
		self.x = round(odom.pose.pose.position.x, 3)
		self.y = round(odom.pose.pose.position.y, 3)

	def routeCmd(self, data):
		cmd, value = data.data.split(";")
		rospy.loginfo(cmd + " " + value)
		print "x: " + str(self.x) + ", y: " + str(self.y) + ", angle: " + str(self.angle)
                self.commands[cmd](int(value))
		print "x: " + str(self.x) + ", y: " + str(self.y) + ", angle: " + str(self.angle)

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
	TurtleBot()
