#!/usr/bin/env python

################

# - Avoid stuff

################

import rospy
import math

from geometry_msgs.msg import PointStamped
from neato_node.msg import Bump
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tty
import select
import sys
import termios
import time

from tf.transformations import euler_from_quaternion

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return pose.position.x, pose.position.y, angles[2]

def add_radians(radian, addition):
	radian += addition
	if radian > math.pi:
		return radian - math.pi
	if radian < -1.0 * math.pi:
		return radian + math.pi
	return radian

class Runner(object):
	def __init__(self, object_tolerance):
		# Robot speed variables
		self.speed = .1 				# Speed coefficient (0 to 1)
		self.linear = 1 				# Linear speed (-1 to 1)
		self.angular = 0				# Angular Speed (-1 to 1)
		self.object_tolerance = object_tolerance

		self.mode = "Forward"

		# Robot Data
		self.ranges = []				# Laser Scan Data

		self.settings = termios.tcgetattr(sys.stdin)
		rospy.init_node('obstacle_avoidance')

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		rospy.Subscriber("scan", LaserScan, self.process_scan)
		rospy.Subscriber("odom", Odometry, self.process_odom)

		self.positionX = False
		self.positionY = False
		self.angle = False

	def process_scan(self, scan):
		self.ranges = scan.ranges

	def process_odom(self, odom):
		self.positionX, self.positionY, self.angle = convert_pose_to_xy_and_theta(odom.pose.pose)
		# print odom.pose.pose

	def publish_twist(self, forward_speed, turn):
		twist = Twist()
		twist.linear.x = forward_speed
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = turn
		self.pub.publish(twist)

	def find_closest_nonzero_forward(self):
		min_dist = 1000
		min_index = -1
		if self.ranges:
			for i in range(-45, 45):
				distance = self.ranges[i]
				if (distance != 0):
					if distance < min_dist:
						min_dist = distance
						min_index = i

		# print "[" + str(min_index) + ", "+ str(min_dist) + "]"
		return [min_index, min_dist]

	def turn_to_zero(self):
		while self.angle > 5:
			self.publish_twist(0, -.5)
		while self.angle < 355:
			self.publish_twist(0, 0.5)

	def run(self):
		r = rospy.Rate(10)
		try:
			while not rospy.is_shutdown():

				# Run
				# Calc dx, dy
				if self.positionX:
					try:
						# Old apprach:
						# goalX = 0; goalY = 0
						# x = self.positionX
						# y = self.positionY
						# d = math.sqrt((goalX - x)**2 + (y - goalY)**2)
						# theta = (math.atan((goalY - y) / (goalX - x)))

						# # self.publish_twist(0, )
						# print "distance: ", d
						# print "angle: ", self.angle 
						# print "theta: ", theta
						# print "error: ", (theta - self.angle) / theta
						# print "------------------"

						# Go forward until something block, then turn until forward is unblock,
						# then go forward until object at old angle zero (negative of new robot angle) is free, 
						# then turn until angle is zero 
						closest_index, closest_distance = self.find_closest_nonzero_forward()
						if closest_distance < self.object_tolerance:
							if closest_index < 0:
								print "go left!"
								self.publish_twist(0, 0.5)
								self.mode = "Escape"
							else:
								self.publish_twist(0, -0.5)
								self.mode = "Escape"
								print "go right!"
						else:
							publish_twist(.5, 0)
							if self.mode == "Escape":
								if self.ranges[360 - self.angle] != 0 and self.ranges[360 - self.angle] < self.object_tolerance:
									self.turn_to_zero()
									self.mode = "Forward"


					except ZeroDivisionError:
						print "curse you zero!"

				r.sleep()
		except KeyboardInterrupt:
			print "Interrupt."


		# Quit - Set linear and angular speeds to zero.
		twist = Twist()
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = 0
		self.pub.publish(twist)

if __name__ == '__main__':
	node = Runner(1)
	node.run()
