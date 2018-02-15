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
		# initialisation de la node MoveTurtleBot
		rospy.init_node('MoveTurtleBot', anonymous=False)
		rospy.loginfo("To stop CTRL + C")
		self.rateValue = 10
		# envoi les données pour réaliser le déplacement
		self.publisher  = rospy.Publisher(self.namespace + '/cmd_vel', Twist, queue_size=self.rateValue)
		# récupération des requetes de déplacement
		self.sub_move = rospy.Subscriber(self.namespace + '/move', String, self.routeCmd)
		# récupération des données odométriques
		self.sub_odom = rospy.Subscriber(self.namespace + '/odom', Odometry, self.odomCallback)
		# permet de limiter l'envoi de message
		self.rate = rospy.Rate(self.rateValue)
		# fonction appelée à l'arret de la node
		rospy.on_shutdown(self.shutdown)

		# fonctions associées aux requetes de déplacement
		self.commands = {}
		self.commands["moveForward"]  = self.moveForward
		self.commands["moveBackward"] = self.moveBackward
		self.commands["turnLeft"]     = self.turnLeft
		self.commands["turnRight"]    = self.turnRight

		self.x = 0
		self.y = 0
		self.angle = 0

		# boucle permettant d'indiquer de la node ne doit pas s'arreter ici
		rospy.spin()

	# calcul de la distance entre deux points
	def dist(self, x1, y1, x2, y2):
		return (sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))) * 100

	# fonction appelée lors de la réception des données odométriques
	def odomCallback(self, odom):
		roll, pitch, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
		# mise à jour des données
		self.angle = (round(degrees(yaw)) + 180) % 360
		self.x = round(odom.pose.pose.position.x, 3)
		self.y = round(odom.pose.pose.position.y, 3)

	# route les requetes de déplacement vers la bonne fonction
	def routeCmd(self, data):
		cmd, value = data.data.split(";")
		rospy.loginfo(cmd + " " + value)
		print "x: " + str(self.x) + ", y: " + str(self.y) + ", angle: " + str(self.angle)
		self.commands[cmd](int(value))
		print "x: " + str(self.x) + ", y: " + str(self.y) + ", angle: " + str(self.angle)

	# permet au robot d'avancer
	def moveForward(self, distance):
		# objet à envoyer au topic "/cmd_vel" permettant le déplacement
		move_cmd = Twist()
		move_cmd.linear.x = 0.1
		start_x = self.x
		start_y = self.y
		# mouvement du robot tant qu'il n'est pas terminé
		while ((distance - 0.5) - self.dist(self.x, self.y, start_x, start_y) > 0.1) and not rospy.is_shutdown():
			self.publisher.publish(move_cmd)
			self.rate.sleep()
		self.stop()

	# de même pour faire reculer le robot
	def moveBackward(self, distance):
		move_cmd = Twist()
		move_cmd.linear.x = -0.1
		start_x = self.x
		start_y = self.y
		while ((distance - 0.5) - self.dist(self.x, self.y, start_x, start_y) > 0.1) and not rospy.is_shutdown():
			self.publisher.publish(move_cmd)
			self.rate.sleep()
		self.stop()

	# fonction permettant de réaliser une rotation vers la gauche
	def turnLeft(self, angle):
		# objet à envoyer au topic "/cmd_vel" permettant le déplacement
		move_cmd = Twist()
		move_cmd.angular.z = radians(angle / 4)
		start_angle = self.angle
		end_angle = (start_angle + angle) % 360
		# rotation tant que l'angle n'est pas correctement réalisé
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

	# de même pour les rotations vers la droite
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

	# permet de stopper le robot
	def stop(self):
		move_cmd = Twist()
		self.publisher.publish(move_cmd)

	# appelée lorsque la node se termine
	def shutdown(self):
		rospy.loginfo("Stopping node")
		self.stop()

if __name__ == '__main__':
	# initialisation (ici du robot "hades")
	TurtleBot("/hades")
