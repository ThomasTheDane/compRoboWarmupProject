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
		rospy.Subscriber("scan", LaserScan, self.process_scan)

		self.wallDistance = .5
		self.target_distance = target_distance

		self.ranges = [0] * 361

	def process_scan(self, scan):
		self.ranges = scan.ranges

	def run(self):
		r = rospy.Rate(10)

		while not rospy.is_shutdown():
			twist = Twist()
			twist.linear.x = .1; twist.linear.y = 0; twist.linear.z = 0

			smallestDistance, index = self.smallestNonZero(self.ranges)
			if smallestDistance != 0:
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = (self.target_distance - self.ranges[270]) * -1
				# if smallestDistance < self.target_distance:
				# 	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = .5
				# else:
				# 	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -.5
			else:
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

			print "Z: ", self.target_distance - smallestDistance
			print "angle of robot: ", index
			self.pub.publish(twist)
			r.sleep()

	def smallestNonZero(self, input):
		smallestSoFar = 1000
		itsIndex = -1
		for index, aThing in enumerate(input):
			if aThing < smallestSoFar and aThing != 0:
				smallestSoFar = aThing
				itsIndex = index
		if smallestSoFar == 1000:
			return 0, -1

		return smallestSoFar, itsIndex


if __name__ == '__main__':
	node = WallFollow(0.5)
	node.run()
