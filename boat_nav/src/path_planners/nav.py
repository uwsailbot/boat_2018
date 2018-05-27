#!/usr/bin/env python
from overrides import overrides
from path_planners.planner_base import Planner

class NavPlanner(Planner):
	
	@overrides
	def setup(self):
		pass
	
	
	@overrides
	def planner(self):
		self.traverse_waypoints_planner();
