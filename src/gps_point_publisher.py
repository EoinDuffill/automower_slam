#!/usr/bin/env python
import sys
import rospy
import time
import xml.etree.ElementTree as ET
import json
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension

class automower_gps(object):

	def __init__(self, source):
		#GPS Source, 0 is live, 1 is from .gpx file#
		#live data from gpspipe: gpspipe -w | ./<file_name>.py 1
		self.gps_receiver = source

	def init(self):
		#Setup experiment data publisher
 	   	self.pub_gps_data = rospy.Publisher('slam/gps_receiver', Float32MultiArray, queue_size=1)
		#initialise list variables
		self.lats = []
		self.longs = []
		self.times = []

	#parse GPS xml file
	def parse_points_file(self):
		#open gps points file
		tree = ET.parse("test_points.gpx")
		#Get root of xml tree
		root = tree.getroot()
		#start time of the gps points var
		start_time = None
		#Iterate over each gps point
		for elem in root.iter("{http://www.topografix.com/GPX/1/0}trkpt"):
			#Find sub elements that have the source GPS get time
			for sub_elem1 in elem.iter("{http://www.topografix.com/GPX/1/0}src"):
				for sub_elem2 in elem.iter("{http://www.topografix.com/GPX/1/0}time"):
					if sub_elem1.text == "gps":
						print sub_elem1.text
						#transform to seconds
						time = self.get_seconds(sub_elem2.text)
						#Initialise start time var if not done so already					
						if start_time == None:
							start_time = time
						time = time - start_time
						#append lats,longs and times to the lists of each
						self.lats.append(float(elem.attrib['lat']))
						self.longs.append(float(elem.attrib['lon']))
						self.times.append(time)
			
	#transform HH:MM:SS.mm into seconds
	def get_seconds(self, time):
		time = time.split("Z")[0].split("T")[1]
		h, m, s = time.split(":")
		return int(h) * 3600 + int(m) * 60 + float(s)

	def publish_gps_file(self):
		prev_time = None
		time_diff = 0
		#Iterate over each lat,long and time
		for i in range(len(self.times)):
			print(self.lats[i], self.longs[i], self.times[i])
			if prev_time != None:
				#difference in time between previous gps point and current
				time_diff = self.times[i] - prev_time
				print(self.times[i] - prev_time)
				if time_diff > 0:
					#Sleep thread for the time difference between last point and current
					#Then publish the GPS point
					time.sleep(self.times[i] - prev_time)
					self.publish_gps_point(self.lats[i], self.longs[i], self.times[i])
			if(time_diff >= 0):
				#Aslong as positive time_diff set new previous points time
				prev_time = self.times[i]

	def publish_gps_point(self, lat, lon, time):
		#Publish current experiment data for mobile device to subscribe to. 
		mat = Float32MultiArray()
		#set up data structure
		mat.layout.dim.append(MultiArrayDimension())
		mat.layout.dim[0].label = "gps_data"
		mat.layout.dim[0].size = 3
		mat.layout.dim[0].stride = 1
		mat.layout.data_offset = 0
		mat.data = [0]*3
		#lat, lon, and time
		mat.data[0] = lat
		mat.data[1] = lon
		mat.data[2] = time
		#publish result
		self.pub_gps_data.publish(mat)

	#Read gpspipe from stdin
	def read_gpspipe(self):
		line = sys.stdin.readline()
		if not line: print "not line"
		else:
			#into json
			sentence = json.loads(line)
			if sentence['class'] == 'TPV':
				#extract and publish relevant info
				lat = sentence['lat']
				lon = sentence['lon']
				time = self.get_seconds(sentence['time'])
		 		print sentence
				self.publish_gps_point(lat, lon, time)

	def fini(self):
		print("finishing")

	def run(self):
		try:
			self.init()
			#Live GPS
			if(self.gps_receiver):
				while not rospy.is_shutdown():
					#function to read in GPS from gpspipe
					self.read_gpspipe()
			#From file GPS
			else:
				self.parse_points_file()
				self.publish_gps_file()
		except rospy.exceptions.ROSInterruptException:
			pass
		finally:
			self.fini()

if __name__ == '__main__':
	if(len(sys.argv) > 1):
		arg = int(sys.argv[1]) 
		if (arg == 0 or arg == 1):
			rospy.init_node('automower_gps_publisher')
			slam_gps = automower_gps(arg)
			slam_gps.run()
		else:
			print "Error parsing command line argument", sys.argv[1]
	else:
		print "Command line arguments required, '0' - GPS from file, '1' - Live GPS"
