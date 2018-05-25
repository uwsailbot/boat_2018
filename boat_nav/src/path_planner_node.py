#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True

import math
import rospy
from boat_msgs.msg import BoatState, Point, WaypointArray
from path_planners.config import Data, Globals as g, initialize
from path_planners.search import SearchPlanner
from path_planners.station import StationPlanner
from path_planners.nav import traverse_waypoints_planner

search_planner = SearchPlanner()
station_planner = StationPlanner()

def boat_state_callback(new_state):
	
	prev_state = g.state
	state = g.state = new_state
	
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
	if (state.challenge is BoatState.CHA_NAV or state.challenge is BoatState.CHA_LONG) and (new_cha or new_maj) and len(g.waypoints)>0:
		waypoints_callback(WaypointArray(g.waypoints))
		g.set_minor_state(BoatState.MIN_PLANNING)


def waypoints_callback(new_waypoint):
	g.waypoints = new_waypoint.points
	state = g.state
	
	if state.major is not BoatState.MAJ_AUTONOMOUS:
		return
	
	if state.challenge is BoatState.CHA_NAV or state.challenge is BoatState.CHA_LONG or state.challenge is BoatState.CHA_SEARCH:
		if len(g.waypoints) > 0:
			g.target_waypoint = g.waypoints[0]
			g.publish_target()
		
	elif state.challenge is BoatState.CHA_STATION:
		return
	
	# If we are waiting in autonomous-complete, and a new waypoint is added, move to planning state
	if state.minor is BoatState.MIN_COMPLETE or state.minor is BoatState.MIN_INITIALIZE and len(g.waypoints)>0:
		g.set_minor_state(BoatState.MIN_PLANNING)


def position_callback(position):
	Data.cur_pos = position
	state = g.state
	
	# If the boat isn't in the autonomous planning state, exit
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	if state.challenge is BoatState.CHA_SEARCH:
		search_planner.planner()
	
	# Navigation and long-distance challenge
	if state.challenge is BoatState.CHA_NAV or state.challenge is BoatState.CHA_LONG:
		traverse_waypoints_planner()
	
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

