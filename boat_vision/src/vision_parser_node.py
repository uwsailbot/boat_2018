#!/usr/bin/env python
import rospy
from boat_msgs.msg import Point, VisionTarget

# TODO: Read vision topic and boat_pos and aggregate data into waypoints

def initialize_node():
	"""Initialize the node. Setup the node handle and subscribers for ROS."""
	rospy.init_node('vision_parser')
	
	#Stub


if __name__ == '__main__':
	try:
		initialize_node()
	except rospy.ROSInterruptException:
		pass

