#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from neato_node.msg import Bump
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import tty
import select
import sys
import termios
import time
class WallFollow(object):

	def __init__(self, target_distance):
		self.settings = termios.tcgetattr(sys.stdin)
		rospy.init_node('wall_follow')

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.rospy.Subscriber("scan", LaserScan, self.process_scan)

		self.wallDistance = .5
		self.target_distance = target_distance

	def process_scan(self):
		print 

	def run(self):
		r = rospy.Rate(10)

		while not isDone and not rospy.is_shutdown():
			twist = Twist()
			twist.linear.x = .1; twist.linear.y = 0; twist.linear.z = 0
			# if distance < wallDistance: postive z 
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

			pub.publish(twist)
			r.sleep()


if __name__ == '__main__':
	node = WallApproach(1.0)
	node.run()
