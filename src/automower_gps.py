#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import pyproj as proj
from std_msgs.msg import Float32MultiArray


class AutomowerGps(object):

	def __init__(self):
		# Update rate in Hz
		self.update_rate = 10

		# open file
		self.file = open("results_gps.txt", "w")

		# Initialising vars
		self.lats = []
		self.longs = []
		self.time = 0
		# variable to compare against gps point's time
		self.plot_time = 0
		# Time between plots
		self.plot_interval = 10

	def receive_gps_point(self, data):
		# Recieved gps point, append it to lists
		print("Point received...")
		self.lats.append(data.data[0])
		self.longs.append(data.data[1])
		self.time = data.data[2]
		self.file.write(str(data.data[0])+", "+str(data.data[1])+"\n")
		print(data.data[0], data.data[1], data.data[2])

	def update(self):
		if self.time > self.plot_time and self.time is not None:
			# If it has been plot_interval seconds since last plot then plot the data
			self.plot_data()
			self.plot_time = self.time + self.plot_interval

	def plot_data(self):
		print("plotting")
		
		# Projection vars for the UK
		crs_wgs = proj.Proj(init='epsg:4326')
		crs_bng = proj.Proj(init='epsg:27700')
		
		# Function to transform latitudes and longitudes to localised x,y co-ordinates
		x,y = proj.transform(crs_wgs, crs_bng, np.array(self.longs), np.array(self.lats))

		# Plot parsed points
		plt.plot(x, y)
		# axis squared off, uniform in x and y
		plt.axis('equal')
		# Set axis based on min and max vals in x and y
		plt.axis([np.amin(x) - 1, np.amax(x) + 1, np.amin(y)  - 1, np.amax(y) + 1])
		# Save figure to file
		print("Saving Plot...")
		plt.savefig('gps_plot.png')	

	@staticmethod
	def fini():
		print('Finishing...')

	def run(self):
		try:
			#Setup subscribers
			print ("Setting up subscriber...")
			rospy.Subscriber('slam/gps_receiver', Float32MultiArray, self.receive_gps_point, queue_size=1)
			r = rospy.Rate(self.update_rate)
			while not rospy.is_shutdown():
				self.update()
				r.sleep()
		except rospy.exceptions.ROSInterruptException:
			pass
		finally:
			self.fini()


if __name__ == '__main__':
	rospy.init_node('automower_slam_gps')
	automower_gps = AutomowerGps()
	automower_gps.run()
