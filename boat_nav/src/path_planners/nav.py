#!/usr/bin/env python
from overrides import overrides
from boat_msgs.msg import BoatState
from path_planners.planner_base import Planner

class NavPlanner(Planner):
	
	@overrides
	def setup(self):
		if len(self.waypoints) is 0:
			return
		
		self.publish_target(self.waypoints[0])
		self.set_minor_state(BoatState.MIN_PLANNING)
	
	
	@overrides
	def planner(self):
		
		# Run the default traverse_waypoints planner
		self.traverse_waypoints_planner();
