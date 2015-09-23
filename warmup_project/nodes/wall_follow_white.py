#!/usr/bin/env python

################

# - Find Wall
# - Determine Orientation

################

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

give = 5
target = .5 

class Runner(object):
	def __init__(self, wall_distance):
		# Robot states
		self.done = False
		self.distanced = False
		self.wall_distance = wall_distance
		self.wall_error = .1

		# Robot speed variables
		self.speed = .1
		self.linear = 1
		self.angular = 0

		# Robot Data
		self.ranges = []

		self.settings = termios.tcgetattr(sys.stdin)
		rospy.init_node('wall_follow_white')

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		rospy.Subscriber("scan", LaserScan, self.process_scan)


	def find_closest_nonzero(self):
		min_dist = 1000
		min_index = -1
		for i in range(0, len(self.ranges)):
			distance = self.ranges[i]
			if (distance != 0):
				if distance < min_dist:
					min_dist = distance
					min_index = i

		# print "[" + str(min_index) + ", "+ str(min_dist) + "]"
		return [min_index, min_dist]


	def process_key(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		print ""
		
		# Shut off on "z" press
		if (key == 'z'):
			self.done = True

		# Forward on "w" press
		elif (key == 'w'):
			self.linear = 1

	def calc_error(self):
		minimum = self.find_closest_nonzero()
		angle = minimum[0] # Angle to closest object

		if angle > 180:
			self.error = angle - 270.0
		else:
			self.error = angle - 90.0

	def orient_parallel(self):
		print "going parallel"
		
		self.calc_error()
		turn = self.error / 90.0
		self.angular = turn

		twist = Twist()

		twist.linear.x = self.linear * self.speed
		twist.linear.y = 0
		twist.linear.z = 0
		# if distance < wallDistance: postive z 
		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = self.angular
		self.pub.publish(twist)

	def process_scan(self, scan):
		self.ranges = scan.ranges[0:360]

	def go_to_distance(self):
		print "finding wall"
		self.calc_error()

		angle, distance = self.find_closest_nonzero()
		turn = ((self.wall_distance - distance) / self.wall_distance) / 2
		if angle < 180:
			turn = turn * -1
		# print "turn: ", turn
		# print "distance: ", distance
		# print "error: ", self.error
		# print "--------------------"

		twist = Twist()

		twist.linear.x = self.linear * self.speed
		twist.linear.y = 0
		twist.linear.z = 0

		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = self.angular
		self.pub.publish(twist)

		while abs(self.error) > 20:
			self.orient_parallel()
		
		self.angular = turn

	def run(self):
		r = rospy.Rate(10)
		try:
			while not self.done and not rospy.is_shutdown():

				# Run
				closest_object = self.find_closest_nonzero()
				if (closest_object[1] < self.wall_distance - (self.wall_error / 2)) or (closest_object[1] > self.wall_distance + (self.wall_error / 2)):
					# state 1: get to right distance from wall  
					self.go_to_distance()

				else:
					# state 2: go parallel to wall
					self.orient_parallel()
				
						# Move - Set linear and angular speeds to those owned by self.
				# print "linear: " + str(self.linear) + \
				# ", angular: " + str(self.angular)


				r.sleep()
		except KeyboardInterrupt:
			print "Interrupt."


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
	node = Runner(.5)
	node.run()
