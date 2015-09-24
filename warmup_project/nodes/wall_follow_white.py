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

DEBUG = True

class Runner(object):
	def __init__(self, wall_distance, wall_point_threshold):
		# Robot states

		self.done = False
		self.is_distanced = False
		self.is_parallel = False					# True: parallel to wall
		
		self.wall_distance = wall_distance
		self.wall_point_threshold = 15
		self.wall_gap_threshold = 10

		self.error_linear = 0
		self.error_angular = 0
		self.permissible_distance = .1

		# Robot speed variables
		self.speed = .1 				# Speed coefficient (0 to 1)
		self.linear = 1 				# Linear speed (-1 to 1)
		self.angular = 0				# Angular Speed (-1 to 1)

		# Robot Data
		self.ranges = []				# Laser Scan Data
		self.ranges_backup = []			# Past laser scan data

		self.settings = termios.tcgetattr(sys.stdin)
		rospy.init_node('wall_follow_white')

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.pub_wallscan = rospy.Publisher('wallscan', 
											LaserScan, 
											queue_size=10)
		rospy.Subscriber("scan", LaserScan, self.process_scan)

		if DEBUG:
			self.speed = 0


	def find_closest_nonzero(self):
		min_dist = 1000
		min_index = -1
		if self.ranges != []:
			for i in range(0, len(self.ranges)):
				try:
					distance = self.ranges[i]
					if (distance != 0):
						if distance < min_dist:
							min_dist = distance
							min_index = i
				except IndexError:
					print "!!!!! INDEX ERROR !!!!!"
					print "		attempted index:", i
					print "		self.ranges:", self.ranges


		# print "[" + str(min_index) + ", "+ str(min_dist) + "]"
		return [min_index, min_dist]

	def find_walls(self):

		# Do not run without LIDAR data
		if self.ranges == []:
			return
			
		self.ranges_backup = self.ranges
		self.wall_ranges = self.ranges
		self.wall_debug = ["*"] * len(self.ranges)

		i = 0
		# Iterate by each index, allowing for index jumps.
		while i < len(self.wall_ranges):

			# If the point could be part of a wall:
			if self.wall_ranges[i] != 0:
				is_wall = True
				

				# Check the pts following the first valid pt (i), to see if
				# object is a wall
				last = i 				# Last valid pt
				w = i					# index for "checking" iteration
				g = 0					# Count the # of "gaps" (distance = 0)

				while is_wall:

					# If the value is zero, its a gap, increase gap count
					if self.wall_ranges[w] == 0:
						g = g + 1

					# If the value is not zero, the gap is closed.
					else:
						g = 0
						last = i

					# If the number of gaps exceeds the threshold, stop counting
					if g < self.wall_gap_threshold:
						is_wall = False
						break
						
					w = w + 1

				print "wall found: ["+ str(i) + "," + str(last) + "]"
	

				# If the # of gaps is less than the # of allowed gaps
				#	Consider the whole section a wall, filling in where
				#	there were gaps with an updating average.
				if g < self.wall_gap_threshold:
					w = i + 1

					a = 1.0
					avg = self.wall_ranges[i] * 1.0

					while (w < self.wall_point_threshold):
						if self.wall_ranges[w] == 0:
							self.wall_ranges[w] = avg
						else:
							# Update the average
							avg = (avg*a + self.wall_ranges[w]) / (a+1)
							a = a + 1

				i = last + 1

			else:
				i = i + 1

			print ""
			print ""
			print "ranges:", self.ranges
			print "wall_ranges:", self.wall_ranges


			wall_scan = self.make_wallscan(self.wall_ranges)
			self.pub_wallscan.publish(wall_scan)
		
	def make_wallscan(self, data):
		num_readings = len(data)
		wall_scan = LaserScan()
		wall_scan.header.frame_id = "base_laser_link"
		wall_scan.ranges = data
		wall_scan.angle_min = -3.14;
		wall_scan.angle_max = 3.14;
		wall_scan.angle_increment = (3.14*2) / num_readings;
		wall_scan.range_min = 0.0;
		wall_scan.range_max = 5;

		return wall_scan

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

	def calc_error_angular(self):
		minimum = self.find_closest_nonzero()
		angle = minimum[0] # Angle to closest object

		# Calculates error_angular - difference in degrees
		if angle > 180:
			self.error_angular = angle - 270.0
		else:
			self.error_angular = angle - 90.0

	def orient_parallel(self):
		'''
		Determines angular velocity, depending on the robot's
		angle relative to the wall. Proportional.
		'''
		self.calc_error_angular()

		# Proportional control - 
		#	Translates from 90 to 0, to 1 to 0
		turn = self.error_angular / 90.0
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
		# print "scan.header.frame_id", scan.header.frame_id
		# print "scan.angle_min", scan.angle_min
		# print" scan.angle_min:", scan.angle_min
		# print" scan.angle_max:", scan.angle_max
		# print" scan.angle_increment:", scan.angle_increment
		# print" scan.range_min:", scan.range_min
		# print" scan.range_max:", scan.range_max
		ranges = []
		for i in range(0, 360):  # Remove redundant 361st value
			ranges.append(scan.ranges[i])
		if ranges != []:
			self.ranges = ranges
			self.find_walls()

	def go_to_distance(self):
		
		self.calc_error_angular()

		angle, distance = self.find_closest_nonzero()
		turn = ((self.wall_distance - distance) / self.wall_distance) / 2
		if angle < 180:
			turn = turn * -1

		self.error_linear = self.wall_distance - distance

		twist = Twist()

		twist.linear.x = self.linear * self.speed
		twist.linear.y = 0
		twist.linear.z = 0

		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = self.angular
		self.pub.publish(twist)

		while abs(self.error_angular) > 20:
			self.orient_parallel()
		
		self.angular = turn

	def run(self):
		r = rospy.Rate(10)
		try:
			while not self.done and not rospy.is_shutdown():

				# Run
				behavior = ""

				closest_object = self.find_closest_nonzero()
				if (closest_object[1] < self.wall_distance - (self.permissible_distance / 2)) or (closest_object[1] > self.wall_distance + (self.permissible_distance / 2)):
					# state 1: get to right distance from wall  
					if not DEBUG:
						self.go_to_distance()
						behavior = "correcting distance"

				else:
					# state 2: go parallel to wall
					self.orient_parallel()
					behavior = "correcting angle"

				print "errors: angular;", self.error_angular, \
				"linear;", self.error_linear, " | behavior:", behavior
				
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
	node = Runner(.5, 20)
	node.run()
