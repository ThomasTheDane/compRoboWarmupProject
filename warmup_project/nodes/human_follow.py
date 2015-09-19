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

class HumanFollow(object):

	def __init__(self, target_distance):
		self.target_distance = target_distance


class Runner(object)
	def __init__(self):
		self.speed = 1
		self.linear = 0
		self.angular = 0

		self.settings = termios.tcgetattr(sys.stdin)
		rospy.init_node('human_follow')

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.rospy.Subscriber("scan", LaserScan, self.process_scan)

		self.hfollower = HumanFollow(.5)

	def process_scan(self):
		print 

	def run(self):
		r = rospy.Rate(10)

		while not isDone and not rospy.is_shutdown():
			twist = Twist()
			twist.linear.x = self.linear
			twist.linear.y = 0
			twist.linear.z = 0
			# if distance < wallDistance: postive z 
			twist.angular.x = 0 
			twist.angular.y = 0
			twist.angular.z = self.angular

			pub.publish(twist)
			r.sleep()

if __name__ == '__main__':
	node = Runner()
	node.run()
