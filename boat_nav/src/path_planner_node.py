#!/usr/bin/env python
import math
import rospy
from boat_msgs.msg import BoatState, Point, PointArray
from boat_msgs.srv import ConvertPoint

# Declare global variables needed for the node
state = BoatState()
waypoints = []
rate = 0

# Declare the publishers for the node
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
waypoints_pub = rospy.Publisher('waypoints_raw', PointArray, queue_size=10)
target_pub = rospy.Publisher('target_point', Point, queue_size=10)

def boat_state_callback(new_state):
	global state
	global waypoints
	state = new_state
	
	# Move to planning state if there is a pending waypoint
	if state.major is BoatState.MAJ_AUTONOMOUS and state.minor is BoatState.MIN_COMPLETE and len(waypoints)>0:
		state.minor = BoatState.MIN_PLANNING
		boat_state_pub.publish(state)
	
	if state.challenge is BoatState.CHA_STATION:
		waypoints = []
		waypoints_pub.publish(waypoints)
		

def waypoints_callback(new_waypoint):
	global waypoints
	global cur_pt
	
	waypoints = new_waypoint.points
	if(len(waypoints) > 0):
		publish_target(waypoints[0])
	
	# If we are waiting in autonomous-complete, and a new waypoint is added, move to planning state
	if state.major is BoatState.MAJ_AUTONOMOUS and state.minor is BoatState.MIN_COMPLETE and len(waypoints)>0:
		state.minor = BoatState.MIN_PLANNING
		boat_state_pub.publish(state)

def next_point():
	global waypoints
	del waypoints[0]
	waypoints_pub.publish(waypoints)

def publish_target(point):
	local = gps_to_lps(point).pt
	target_pub.publish(local)
	rospy.loginfo(rospy.get_caller_id() + " New target waypoint: (long: %.2f, lat: %.2f) or (x: %.f, y: %.f)", point.x, point.y, local.x, local.y)

def position_callback(position):
	global waypoints
	global rate
	buoy_tolerance = 5
	
	rate = rospy.Rate(100)
	
	# If the boat isn't in the autonomous planning state, exit
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	# If the list of waypoints is not empty 
	if(len(waypoints) > 0):
		# If the boat is close enough to the waypoint...
		if is_within_dist(position, gps_to_lps(waypoints[0]).pt, buoy_tolerance):
			rospy.loginfo(rospy.get_caller_id() + " Reached intermediate waypoint (lat: %.2f, long: %.2f)", waypoints[0].y, waypoints[0].x)
			next_point()
	
	# If there are no waypoints left to navigate to, exit	
	elif state.minor != BoatState.MIN_COMPLETE:
		state.minor = BoatState.MIN_COMPLETE
		boat_state_pub.publish(state)
		rospy.loginfo(rospy.get_caller_id() + " No waypoints left. Boat State = 'Autonomous - Complete'")
	
	# Adjust the sleep to suit the node
	rate.sleep()


# Determine if the dist between two points is within the specified tolerance
def is_within_dist(p1, p2, dist):
	a = math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2)
	return math.sqrt(a) < dist


# Initialize the node
def initialize():
	rospy.init_node('path_planner')
	
	global gps_to_lps
	global lps_to_gps
	rospy.wait_for_service('gps_to_lps')
	rospy.wait_for_service('lps_to_gps')
	gps_to_lps = rospy.ServiceProxy('gps_to_lps', ConvertPoint)
	lps_to_gps = rospy.ServiceProxy('lps_to_gps', ConvertPoint)
	
	# If the filters work, change lps to use /odometry/filtered
	rospy.Subscriber('lps', Point, position_callback)
	rospy.Subscriber('waypoints_raw', PointArray, waypoints_callback)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		initialize()
	except rospy.ROSInterruptException:
		pass

