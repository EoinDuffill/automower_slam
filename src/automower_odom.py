#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates
from am_driver.msg import WheelEncoder
from std_msgs.msg import Float32, Float32MultiArray

class automower_odom(object):

	def init(self):
		#Update rate in Hz
		self.update_rate = 50
		#Distance between the centers of the rear wheels
		self.wheel_separation_distance = 0.48
		#Setup experiment data publisher
 	   	self.pub_odom_data = rospy.Publisher('slam/odom', Float32MultiArray, queue_size=1)

	def wheel_encoder(self, data):
		if not(self.prev_tick == None):
			#Accumulate encoder data 
			self.wheel_l_accum += (data.header.seq - self.prev_tick)*data.lwheel
			self.wheel_r_accum += (data.header.seq - self.prev_tick)*data.rwheel + 00000001
			#Update time variable
			self.time = (data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)) - self.start_time
			
		else:
			#Record start time, this is only called on the first iteration
			self.start_time = data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)

		#Record previous tick
		self.prev_tick = data.header.seq
	
	#Formula for calculating orientation change over time with wheel encoder data
	def orientation_change(self, e_left, e_right, orientation_start):
		#if encoder values are similar enough ignore the difference
		if not(self.isclose(e_right, e_left, rel_tol=1e-8, abs_tol=1e-10)):
			return ((e_right-e_left)/self.wheel_separation_distance) + orientation_start
		else:	
			return orientation_start

	#Formula for calculating position change in x,y over time
	def position_change(self, e_left, e_right, x_start, y_start, orientation_start):
		#again ignoring the difference in wheel encoders if similar enough
		if not(self.isclose(e_right, e_left, rel_tol=1e-8, abs_tol=1e-10)):
			factor = (self.wheel_separation_distance * (e_right + e_left))/(2 * (e_right - e_left))
			trigonometric_input = ((e_right - e_left)/self.wheel_separation_distance) + orientation_start
			X = x_start + (factor * (math.sin(trigonometric_input) - math.sin(orientation_start)))
			Y = y_start - (factor * (math.cos(trigonometric_input) - math.cos(orientation_start)))

			return X, Y
		else:	
			return x_start,y_start

	#function to determine if two numbers are sufficiently close to each other
	def isclose(self, a, b, rel_tol, abs_tol):
		return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

	def write_data(self):
		
		#Calculate position change, then orientation
		(self.x, self.y) = self.position_change(self.wheel_l_accum, self.wheel_r_accum, self.x, self.y, self.orientation)
		self.orientation = self.orientation_change(self.wheel_l_accum, self.wheel_r_accum, self.orientation)
		self.orientation = ((self.orientation + math.pi) % (2*math.pi)) - math.pi
			
		#print(self.time, self.orientation, self.x, self.y)
		self.file.write("Time = " + str(self.time) + "\n")
		self.file.write("Odometry: x = " + str(self.x) + " y = " + str(self.y) + " theta = " + str(self.orientation) + "\n")
		
		#Record x and y, and x and y predictions from odometry
		self.x_odom_points.append(self.x)
		self.y_odom_points.append(self.y)

		#Resets accumalative encoders
		self.wheel_l_accum = 0
		self.wheel_r_accum = 0

	def update(self):
		self.write_data()

	def fini(self):
		print('Finishing...')

	def run(self):
		try:
			self.init()
			#Setup subscribers
			rospy.Subscriber("wheel_encoder", WheelEncoder, self.wheel_encoder, queue_size=None)
			r = rospy.Rate(self.update_rate)
			while not rospy.is_shutdown():
				print("hello ros")
				self.update()
				r.sleep()
		except rospy.exceptions.ROSInterruptException:
			pass
		finally:
			self.fini()


if __name__ == '__main__':
	rospy.init_node('automower_odom')
	slam_odom = automower_odom()
	slam_odom.run()
