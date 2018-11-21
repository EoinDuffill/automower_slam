#!/usr/bin/env python
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
import Image
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
		#open file
		self.file = open("results.txt", "a")
		self.file_path = '/home/eoin/ROS/catkin_ws/src/automower_slam/src/figure'

		#SIM
		#Get initial variables from simulator
		data = rospy.wait_for_message("gazebo/model_states", ModelStates)
		#get the index into the data for the automower
		index = data.name.index("automower")

		#init vars
		self.wheel_l_accum = 0
		self.wheel_r_accum = 0
		self.prev_tick = None
		#initial x,y #SIM
		self.x = data.pose[index].position.x
		self.y = data.pose[index].position.y
		#initial angle, converted from a quarternion #SIM
		self.orientation = self.quarternion_to_angle(data.pose[index].orientation.x, data.pose[index].orientation.y,data.pose[index].orientation.z,data.pose[index].orientation.w)

		#List of points from the simulator #SIM
		self.x_points = []
		self.y_points = []

		#List of points from odometry
		self.x_odom_points = []
		self.y_odom_points = []

		#var to hold the tick no of prev tick
		self.prev_tick = None
		#Start time in seconds from simulator
		self.start_time = 0
		#Time since experiment start
		self.time = 0

	def wheel_encoder(self, data):
		if not(self.prev_tick == None):
			#Accumulate encoder data 
			self.wheel_l_accum += (data.header.seq - self.prev_tick)*data.lwheel
			self.wheel_r_accum += (data.header.seq - self.prev_tick)*data.rwheel
			#Update time variable
			self.time = (data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)) - self.start_time
			
		else:
			#Record start time, this is only called on the first iteration
			self.start_time = data.header.stamp.secs + (data.header.stamp.nsecs * 10e-10)

		#Record previous tick
		self.prev_tick = data.header.seq
	

	def odometry(self, e_left, e_right, x_start, y_start, theta_start):
		delta_theta = (e_right - e_left)/self.wheel_separation_distance
		delta_s = (e_right + e_left)/2
		x = x_start + (delta_s * math.cos(theta_start + (delta_theta/2)) )
		y = y_start + (delta_s * math.sin(theta_start + (delta_theta/2)) )
		theta = theta_start + delta_theta
		return x,y,theta

	#Function to take quarternion of automower orientation in 3D space and convert to an angle from a bird eye view
	def quarternion_to_angle(self,x, y, z, w):
		ysqr = y * y

		t1 = +2.0 * (w * z + x * y)
		t2 = +1.0 - 2.0 * (ysqr + z * z)
		Z = math.atan2(t1, t2)
	
		return Z

	def write_data(self):
		
		#Calculate position change and orientation
		(self.x, self.y, self.orientation) = self.odometry(self.wheel_l_accum, self.wheel_r_accum, self.x, self.y, self.orientation)
	
		#print(self.time, self.orientation, self.x, self.y)
		#self.file.write("Time = " + str(self.time) + "\n")
		#self.file.write("Odometry: x = " + str(self.x) + " y = " + str(self.y) + " theta = " + str(self.orientation) + "\n")
		#x,y plot points
		self.file.write(str(self.x)+",	"+str(self.y)+"\n")
		
		#Record x and y, and x and y predictions from odometry
		self.x_odom_points.append(self.x)
		self.y_odom_points.append(self.y)

		#Resets accumalative encoders
		self.wheel_l_accum = 0
		self.wheel_r_accum = 0
		
		#SIM
		#Get initial variables from simulator
		data = rospy.wait_for_message("gazebo/model_states", ModelStates)
		#get the index into the data for the automower
		index = data.name.index("automower")
		#SIM
		self.x_points.append(data.pose[index].position.x)
		self.y_points.append(data.pose[index].position.y)

		#TEMP plot figure, COMPUTATIONALLY INTENSIVE
		#self.plot_figure()	

	def plot_figure(self):
		#Clear points and axis
		plt.cla()
		plt.clf()
		#Plot actual x co-ords #SIM
		plt.plot(self.x_points, self.y_points, 'k')
		#Plot odometry calculated points
		plt.plot(self.x_odom_points, self.y_odom_points, 'b--')

		#SIM
		#concatenate all x and all y points for min and max calculations for the graphs axis
		all_x_points = np.concatenate([self.x_points,self.x_odom_points])
		all_y_points = np.concatenate([self.y_points,self.y_odom_points])
		
		#axis sqaured off, uniform in x and y
		plt.axis('equal')
		#Set axis based on min and max vals in x and y
		plt.axis([np.amin(all_x_points) - 1, np.amax(all_x_points) + 1, np.amin(all_y_points) - 1, np.amax(all_y_points) + 1])
		#Save figure to file
		plt.savefig(self.file_path+'.png')

	def update(self):
		print("Odometry: x = " + str(self.x) + " y = " + str(self.y) + " theta = " + str(self.orientation) + "\n")
		self.write_data()

	def fini(self):
		self.file.close()
		print('Finishing...')

	def run(self):
		try:
			self.init()
			#Setup subscribers
			rospy.Subscriber("wheel_encoder", WheelEncoder, self.wheel_encoder, queue_size=None)
			r = rospy.Rate(self.update_rate)
			while not rospy.is_shutdown():
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
