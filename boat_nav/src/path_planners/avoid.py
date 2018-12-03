#!/usr/bin/env python
import rospy
from overrides import overrides
from boat_msgs.msg import BoatState, PointArray
from std_msgs.msg import Bool, Float32
from path_planners import Planner

class AvoidPlanner(Planner):
	"""Not a real planner just some BS to get partial points."""

	def __init__(self):
		self.has_obstacle = False
		self.rudder_pub = rospy.Publisher('rudder', Float32, queue_size=10)
		self.pid_enable_pub = rospy.Publisher('rudder_pid/enable', Bool, queue_size=10)
		rospy.Subscriber('vision', PointArray, self._obstacle_callback)


	@overrides
	def setup(self):
		self.pid_enable_pub.publish(Bool(False))
		self._set_minor_state(BoatState.MIN_PLANNING)


	@overrides
	def planner(self):
		if self.has_obstacle:
			self.rudder_pub.publish(Float32(150))
		else:
			self.rudder_pub.publish(Float32(90))


	def _obstacle_callback(self, targets):
		self.has_obstacle = len(targets.points) > 0

		if Planner.state.challenge is BoatState.CHA_AVOID:
			self.planner()
