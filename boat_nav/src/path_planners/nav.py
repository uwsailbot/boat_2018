#!/usr/bin/env python
import rospy
from overrides import overrides
from boat_msgs.msg import BoatState
from path_planners import Planner


class NavPlanner(Planner):
    """Simple Planner implementation that traverses the waypoints in waypoints_raw once each."""

    @overrides
    def setup(self):
        if len(self.waypoints) is 0:
            rospy.loginfo(rospy.get_caller_id() + " No points to nav")
            self._set_minor_state(BoatState.MIN_COMPLETE)
            return

        self._publish_target(self.waypoints[0])
        self._set_minor_state(BoatState.MIN_PLANNING)

    @overrides
    def planner(self):

        # Run the default traverse_waypoints planner
        self._traverse_waypoints_planner()
