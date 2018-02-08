#!/usr/bin/env python

import rospy
from math import radians, degrees, sqrt
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class TurtleBot():
	def __init__(self, namespace):
		self.namespace = namespace
		rospy.init_node('MoveTurtleBot', anonymous=False)
		rospy.loginfo("To stop CTRL + C")
		self.rateValue = 10
		self.publisher  = rospy.Publisher(self.namespace + '/cmd_vel', Twist, queue_size=self.rateValue)
		self.sub_move = rospy.Subscriber(self.namespace + '/move', String, self.routeCmd)
		self.sub_odom = rospy.Subscriber(self.namespace + '/odom', Odometry, self.odomCallback)
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

	def dist(self, x1, y1, x2, y2): 
		return (sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))) * 100

	def odomCallback(self, odom):
		roll, pitch, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
		self.angle = (round(degrees(yaw)) + 180) % 360
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
		start_x = self.x
		start_y = self.y
		while (distance - self.dist(self.x, self.y, start_x, start_y) > 0.1) and not rospy.is_shutdown():
			self.publisher.publish(move_cmd)
			self.rate.sleep()
		self.stop()

	def moveBackward(self, distance):
		move_cmd = Twist()
		move_cmd.linear.x = -0.1
		start_x = self.x
		start_y = self.y
		while (distance - self.dist(self.x, self.y, start_x, start_y) > 0.1) and not rospy.is_shutdown():
			self.publisher.publish(move_cmd)
			self.rate.sleep()
		self.stop()

	def turnLeft(self, angle):
		move_cmd = Twist()
		move_cmd.angular.z = radians(angle / 4)
		start_angle = self.angle
		end_angle = (start_angle + angle) % 360
		if self.angle < end_angle:
			while end_angle > self.angle and not rospy.is_shutdown():
				self.publisher.publish(move_cmd)
				self.rate.sleep()
		elif self.angle > end_angle:
			while self.angle > 0 and self.angle > end_angle and not rospy.is_shutdown():
				self.publisher.publish(move_cmd)
				self.rate.sleep()
			while self.angle < end_angle and not rospy.is_shutdown():
				self.publisher.publish(move_cmd)
				self.rate.sleep()
		self.stop()

	def turnRight(self, angle):
		move_cmd = Twist()
		move_cmd.angular.z = -radians(angle / 4)
		start_angle = self.angle
		end_angle = ((start_angle - angle) + 360) % 360
		if self.angle > end_angle:
			while end_angle < self.angle and not rospy.is_shutdown():
				self.publisher.publish(move_cmd)
				self.rate.sleep()
		elif self.angle < end_angle:
			while self.angle > 0 and self.angle < end_angle and not rospy.is_shutdown():
				self.publisher.publish(move_cmd)
				self.rate.sleep()
			while self.angle > end_angle and not rospy.is_shutdown():
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
	TurtleBot("/hades")
