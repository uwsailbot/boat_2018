#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True

import math
import rospy
from boat_msgs.msg import BoatState, Point, WaypointArray
from path_planners.planner_base import Planner, initialize
from path_planners.search import SearchPlanner
from path_planners.station import StationPlanner
from path_planners.nav import NavPlanner
from path_planners.long import LongPlanner

search_planner = SearchPlanner()
station_planner = StationPlanner()
nav_planner = NavPlanner()
long_planner = LongPlanner()

def boat_state_callback(new_state):
	
	prev_state = Planner.state
	state = Planner.state = new_state
	waypoints = Planner.waypoints
	
	if state.major is not BoatState.MAJ_AUTONOMOUS:
		return
	
	new_cha = state.challenge is not prev_state.challenge
	new_maj = state.major is not prev_state.major
	
	# For if search area is already present when we switch to search mode
	if state.challenge is BoatState.CHA_SEARCH and (new_cha or new_maj)  and search_planner.search_radius > 0:
		search_planner.setup()
	
	# For if bounding box is already present when we switch to station mode
	if state.challenge is BoatState.CHA_STATION and (new_cha or new_maj) and len(station_planner.box) is 4:
		station_planner.setup()
	
	# Whenever we switch into nav or distance, startup the waypoints callback to load the first waypoint, and switch from INITIALIZE to PLANNING
	if (state.challenge is BoatState.CHA_NAV or state.challenge is BoatState.CHA_LONG) and (new_cha or new_maj) and len(waypoints)>0:
		waypoints_callback(WaypointArray(waypoints))
		Planner.set_minor_state(BoatState.MIN_PLANNING)


def waypoints_callback(new_waypoint):
	waypoints = Planner.waypoints = new_waypoint.points
	state = Planner.state
	
	if state.major is not BoatState.MAJ_AUTONOMOUS:
		return
	
	if state.challenge is BoatState.CHA_NAV or state.challenge is BoatState.CHA_LONG or state.challenge is BoatState.CHA_SEARCH:
		if len(waypoints) > 0:
			Planner.target_waypoint = waypoints[0]
			Planner.publish_target()
		
	elif state.challenge is BoatState.CHA_STATION:
		return
	
	# If we are waiting in autonomous-complete, and a new waypoint is added, move to planning state
	if state.minor is BoatState.MIN_COMPLETE or state.minor is BoatState.MIN_INITIALIZE and len(waypoints)>0:
		Planner.set_minor_state(BoatState.MIN_PLANNING)


def position_callback(position):
	Planner.cur_pos = position
	state = Planner.state
	
	# If the boat isn't in the autonomous planning state, exit
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	# Call the appropriate path planner
	
	# Search challenge
	if state.challenge is BoatState.CHA_SEARCH:
		search_planner.planner()
	
	# Navigation challenge
	if state.challenge is BoatState.CHA_NAV:
		nav_planner.planner()
	
	# Long distance challenge
	if state.challenge is BoatState.CHA_LONG:
		long_planner.planner()
	
	# Station keeping challenge
	elif state.challenge is BoatState.CHA_STATION:
		station_planner.planner()
	
	# Adjust the sleep to suit the node
	rospy.Rate(100).sleep()


# =*=*=*=*=*=*=*=*=*=*=*=*= Initialization =*=*=*=*=*=*=*=*=*=*=*=*=

# Initialize the node
def initialize_node():
	rospy.init_node('path_planner')
	
	initialize()
	
	# If the filters work, change lps to use /odometry/filtered
	rospy.Subscriber('lps', Point, position_callback)
	rospy.Subscriber('waypoints_raw', WaypointArray, waypoints_callback)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		initialize_node()
	except rospy.ROSInterruptException:
		pass

