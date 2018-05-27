#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True

import rospy
from boat_msgs.msg import BoatState, Point, WaypointArray
from path_planners.planner_base import Planner, initialize
from path_planners.search import SearchPlanner
from path_planners.station import StationPlanner
from path_planners.nav import NavPlanner
from path_planners.long import LongPlanner

planners = {BoatState.CHA_NAV: NavPlanner(),
			BoatState.CHA_LONG: LongPlanner(),
			BoatState.CHA_SEARCH: SearchPlanner(),
			BoatState.CHA_STATION: StationPlanner() }

def boat_state_callback(new_state):
	
	prev_state = Planner.state
	state = Planner.state = new_state
	waypoints = Planner.waypoints
	
	if state.major is not BoatState.MAJ_AUTONOMOUS:
		return
	
	new_cha = state.challenge is not prev_state.challenge
	new_maj = state.major is not prev_state.major
	
	
	# Setup the planner when the challenge or major state changes
	if (new_cha or new_maj):
		planners[state.challenge].setup()


def waypoints_callback(new_waypoints):
	Planner.waypoints = new_waypoints.points
	
	# If we are waiting in autonomous-complete, and a new waypoint is added, rerun setup
	if Planner.state.major is BoatState.MAJ_AUTONOMOUS and (Planner.state.minor is BoatState.MIN_COMPLETE or Planner.state.minor is BoatState.MIN_INITIALIZE) and len(Planner.waypoints)>0:
		planners[Planner.state.challenge].setup()


def position_callback(position):
	Planner.cur_pos = position
	state = Planner.state
	
	# If the boat isn't in the autonomous planning state, exit
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	# Call the appropriate path planner
	planners[state.challenge].planner()
	
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

