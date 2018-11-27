#!/usr/bin/env python
import rospy
from overrides import overrides
from boat_msgs.msg import BoatState
from path_planners import Planner

class LongPlanner(Planner):
	"""Simple Planner implementation that traverses the waypoints in waypoints_raw,
	but does not destroy achieved waypoints, allowing infinite looping.
	"""

	@overrides
	def setup(self):
		if len(self.waypoints) is 0:
			rospy.loginfo(rospy.get_caller_id() + " No points to nav")
			self.set_minor_state(BoatState.MIN_COMPLETE)
			return

		self.publish_target(self.waypoints[0])
		self.set_minor_state(BoatState.MIN_PLANNING)


	@overrides
	def planner(self):

		# If we've reached a waypoint, add it to the end of the list to continue looping forever
		if len(self.waypoints) > 0 and self._boat_reached_target():
			self.waypoints.append(self.waypoints[0])

		# Run the default traverse_waypoints planner
		self._traverse_waypoints_planner()
