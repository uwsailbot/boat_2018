#!/usr/bin/env python
import math
import rospy
from boat_msgs.msg import BoatState
from path_planners.config import Globals, boat_reached_target

def traverse_waypoints_planner():
	
	waypoints = Globals.waypoints
	state = Globals.state
	
	# If the list of waypoints is not empty 
	if(len(waypoints) > 0):
	
		# If the boat is close enough to the waypoint, start navigating towards the next waypoint in the path
		if boat_reached_target():
			rospy.loginfo(rospy.get_caller_id() + " Reached intermediate waypoint (lat: %.2f, long: %.2f)", waypoints[0].pt.y, waypoints[0].pt.x)
			
			if state.challenge is BoatState.CHA_LONG:
				waypoints.append(waypoints[0])
			del waypoints[0]
			Globals.update_waypoints(waypoints)
	
	# If there are no waypoints left to navigate to, exit
	else:
		Globals.set_minor_state(BoatState.MIN_COMPLETE)
		rospy.loginfo(rospy.get_caller_id() + " No waypoints left. Boat State = 'Autonomous - Complete'")
