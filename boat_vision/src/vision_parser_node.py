#!/usr/bin/env python
import rospy
import math
from boat_msgs.msg import Point, PointArray, VisionTarget
from boat_msgs.srv import ConvertPoint
from std_msgs.msg import Float32

pub = rospy.Publisher('vision', PointArray, queue_size=10)
to_gps = rospy.ServiceProxy('lps_to_gps', ConvertPoint)

target = VisionTarget()
cur_pos = Point()
points = PointArray()
compass = 0

def vision_callback(data):
	global target
	target = data

def position_callback(data):
	global cur_pos
	cur_pos = data 

def compass_callback(data):
	global compass
	compass = data.data

def sind(angle):
	return math.sin(math.radians(angle))

def cosd(angle):
	return math.cos(math.radians(angle))

def initialize_node():
	rospy.init_node('vision_parser')
	
	rospy.Subscriber('vision_target', VisionTarget, vision_callback, queue_size=1)
	rospy.Subscriber('lps', Point, position_callback, queue_size=1) # Only want it to receive the most recent position
	rospy.Subscriber('compass', Float32, compass_callback, queue_size=1)
	
	# Read vision topic and boat_pos and aggregate data into waypoints
	#TODO: Maybe move to vision callback??
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		points.points = []
		
		if target.has:
			# Calculate the vision target's pos relative to the boat
			point = cur_pos
			cur_pos.x += target.pt.dist*cosd(compass-target.pt.heading)
			cur_pos.y += target.pt.dist*sind(compass-target.pt.heading)
			points.append(to_gps(cur_pos))
		
		# Publish it
		pub.publish(points)
		
		rate.sleep()
	

if __name__ == '__main__':
	try:
		initialize_node()
	except rospy.ROSInterruptException:
		pass

