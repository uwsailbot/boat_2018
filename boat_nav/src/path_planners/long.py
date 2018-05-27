#!/usr/bin/env python
from overrides import overrides
from path_planners.planner_base import Planner

class LongPlanner(Planner):
	
	@overrides
	def setup(self):
		pass
	
	
	@overrides
	def planner(self):
		
		# If we've reached a waypoint, add it to the end of the list to continue looping forever
		if len(self.waypoints) > 0 and self.boat_reached_target():
			self.waypoints.append(self.waypoints[0])
		
		# Run the default traverse_waypoints planner
		self.traverse_waypoints_planner();
