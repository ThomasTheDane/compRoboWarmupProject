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


class Runner(object):
	def __init__(self):
		self.done = False
		self.speed = .5
		self.linear = 0
		self.angular = 0

		self.ranges = []

		self.settings = termios.tcgetattr(sys.stdin)
		rospy.init_node('human_follow')

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		rospy.Subscriber("scan", LaserScan, self.process_scan)

		self.hfollower = HumanFollow(.5)

	def find_human(self):

		# Print nonzero values, with their index (angle)
		s = ""
		for i in range(0, self.ranges):
			if (self.ranges[i] != 0):
				s = s + "["+str(i)+ ", "+ +" ]"

		pass


	def process_key(self):
		print "processing key"
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		
		# Shut off on "z" press
		if (key == 'z'):
			self.done = True

		# Forward on "w" press
		elif (key == 'w'):
			self.linear = 1

	def process_scan(self, scan):
		self.ranges = scan.ranges[0:360]
		print "Scan data length:",  len(self.ranges)
		self.find_human()

	def run(self):
		r = rospy.Rate(10)

		while not self.done and not rospy.is_shutdown():
			key = self.process_key()


			# Move - Set linear and angular speeds to those owned by self.
			print "linear: " + str(self.linear) + \
			", angular: " + str(self.angular)
			twist = Twist()

			twist.linear.x = self.linear * self.speed
			twist.linear.y = 0
			twist.linear.z = 0
			# if distance < wallDistance: postive z 
			twist.angular.x = 0 
			twist.angular.y = 0
			twist.angular.z = self.angular * self.speed

			self.pub.publish(twist)


		# Quit - Set linear and angular speeds to zero.
		twist = Twist()
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		# if distance < wallDistance: postive z 
		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = 0

		self.pub.publish(twist)

if __name__ == '__main__':
	node = Runner()
	node.run()
