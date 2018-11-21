#!/usr/bin/env python
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
import Image
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates
from am_driver.msg import Loop
from std_msgs.msg import Float32, Float32MultiArray

class automower_boundary_sensors(object):

	def init(self):
		#Update rate in Hz
		self.update_rate = 10
		#Setup experiment data publisher
 	   	self.pub_odom_data = rospy.Publisher('slam/boundary_sensors', Float32MultiArray, queue_size=1)
		#open file
		self.file = open("results_boundary_sensors.txt", "w")

		#List of points for sensor values
		self.x_points = []
		self.y_points = []
		self.sensorFC = None
		self.sensorFR = None
		self.sensorRL = None
		self.sensorRR = None
		self.sensorAVG = None
		self.start_time = 0
		self.time = 0

	def parse_boundary_sensors(self, data):

		if(self.sensorAVG == None):
			#Record start time, this is only called on the first iteration
			self.start_time = data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)

		self.sensorFC = data.frontCenter
		self.sensorFR = data.frontRight
		self.sensorRL = data.rearLeft
		self.sensorRR = data.rearRight
		self.sensorAVG = (self.sensorFC + self.sensorFR + self.sensorRL + self.sensorRR)/4.0
		#Record time since start time
		self.time = (data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)) - self.start_time

	def write_data(self):
		if(self.sensorAVG != None):
			print("Time="+str(self.time)+",\tfrontCenter="+str(self.sensorFC) +"\tfrontRight="+str(self.sensorFR)+"\trearLeft="+str(self.sensorRL)+"\trearRight="+str(self.sensorRR)+"\tsensorAVG="+str(self.sensorAVG)+"\n")
			self.file.write(str(self.time)+","+str(self.sensorFC) +"\t"+str(self.sensorFR)+"\t"+str(self.sensorRL)+"\t"+str(self.sensorRR)+"\t"+str(self.sensorAVG)+"\n")

	def update(self):
		self.write_data()

	def fini(self):
		self.file.close()
		print('Finishing...')

	def run(self):
		try:
			self.init()
			#Setup subscribers
			rospy.Subscriber("loop", Loop, self.parse_boundary_sensors, queue_size=None)
			r = rospy.Rate(self.update_rate)
			while not rospy.is_shutdown():
				self.update()
				r.sleep()
		except rospy.exceptions.ROSInterruptException:
			pass
		finally:
			self.fini()


if __name__ == '__main__':
	rospy.init_node('automower_boundary_sensor')
	slam_boundary_sensors = automower_boundary_sensors()
	slam_boundary_sensors.run()
