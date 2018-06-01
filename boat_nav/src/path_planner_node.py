#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True
import rospy
from boat_msgs.msg import BoatState, Point, WaypointArray
from path_planners import *

# Dict of the planners to use
planners = {BoatState.CHA_NAV: NavPlanner(),
			BoatState.CHA_LONG: LongPlanner(),
			BoatState.CHA_SEARCH: SearchPlanner(),
			BoatState.CHA_STATION: StationPlanner() }

def boat_state_callback(new_state):
	"""Callback for boat state."""
	prev_state = Planner.state
	state = Planner.state = new_state
	waypoints = Planner.waypoints
	
	if state.major is not BoatState.MAJ_AUTONOMOUS:
		return
	
	# Setup the planner when the challenge or major state changes
	if state.challenge is not prev_state.challenge or state.major is not prev_state.major:
		planners[state.challenge].setup()


def waypoints_callback(new_waypoints):
	"""Callback for waypoints_raw topic."""
	Planner.waypoints = new_waypoints.points
	
	# If we are waiting in autonomous-complete, and a new waypoint is added, rerun setup
	if Planner.state.major is BoatState.MAJ_AUTONOMOUS and (Planner.state.minor is BoatState.MIN_COMPLETE or Planner.state.minor is BoatState.MIN_INITIALIZE) and len(Planner.waypoints)>0:
		planners[Planner.state.challenge].setup()


def position_callback(position):
	"""Callback for boat position. Run the current planner."""
	Planner.cur_pos = position
	state = Planner.state
	
	# If the boat isn't in the autonomous-planning state, exit
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	# Call the appropriate path planner
	planners[state.challenge].planner()
	
	# Adjust the sleep to suit the node
	rospy.Rate(100).sleep()


# =*=*=*=*=*=*=*=*=*=*=*=*= Initialization =*=*=*=*=*=*=*=*=*=*=*=*=

def initialize_node():
	"""Initialize the node. Setup the node handle and subscribers for ROS."""
	rospy.init_node('path_planner')
	
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

