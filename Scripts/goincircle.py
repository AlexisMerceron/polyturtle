#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

def shutdown():
    rospy.loginfo("Stoping")
    pub.publish(Twist())
    rospy.sleep(1)


if __name__ == '__main__':
    try:
        rospy.init_node('GoForward', anonymous=False)
        rospy.loginfo("CTRL+C to stop")
        rospy.on_shutdown(shutdown)
        r = rospy.Rate(10)

        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 5

        while not rospy.is_shutdown():
            pub.publish(twist)
            r.sleep()
    except:
        rospy.loginfo("GoForward node terminated.")

