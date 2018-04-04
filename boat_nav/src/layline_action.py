#! /usr/bin/env python
import actionlib
import boat_msgs.msg
import rospy
import math
from boat_msgs.msg import BoatState, Point, TackingAction, TackingGoal
from std_msgs.msg import Float32

class LaylineAction(object):
	# create messages that are used to publish feedback/result
	_feedback = boat_msgs.msg.LaylineFeedback()
	_result = boat_msgs.msg.LaylineResult()

	def __init__(self, name):
		self._name = name
		self._as = actionlib.SimpleActionServer(self._name, boat_msgs.msg.LaylineAction, execute_cb=self.layline_callback, auto_start = False)
		self._as.start()
		self.tacking_client = actionlib.SimpleActionClient('tacking_action', TackingAction)
		self.cur_pos = Point()
		self.target_heading = 0
		self.target_sub = rospy.Subscriber('target_heading', Float32, self.target_heading_callback)
		self.pos_sub = rospy.Subscriber('lps', Point, self.position_callback)
		self.rate = rospy.Rate(100)
		self.tacking_client.wait_for_server()

	def target_heading_callback(self, new_target_heading):
		self.target_heading = new_target_heading.data

	def position_callback(self, position):
		self.cur_pos = position

	def is_within_bounds(self, val, boundA, boundB):
		return (boundA < val and val < boundB) or (boundB < val and val < boundA)

	def layline_callback(self, goal):
		# helper variables
		success = False
		preempted = False
		
		# publish info to the console for the user
		self._feedback.status = " Entered Layline Action Callback. "
		rospy.loginfo(rospy.get_caller_id() + self._feedback.status)

		if self.is_within_bounds(goal.alt_tack_angle, 90, 270):
			tacking_direction = 1
		else:
			tacking_direction = -1
		
		self._feedback.status = " Tacking away from mark to hit layline. "
		rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
		tacking_goal = TackingGoal(direction = tacking_direction)
		self.tacking_client.send_goal(tacking_goal)
			
		# Adjust time delay until the tack is considered failed, and we return to planning
		if not self.tacking_client.wait_for_result(rospy.Duration(10)):
			self.tacking_client.cancel_goal()
			tacking_goal.direction = tacking_goal.direction * -1
			self.tacking_client.send_goal(tacking_goal)
			if not self.tacking_client.wait_for_result(rospy.Duration(10)):
				self.tacking_client.cancel_goal()
			self._result.success = success
			self._result.target_heading = self.target_heading
			self._feedback.status = " Tacking goal expired"
			rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
			return
		
		self.target_heading = self.tacking_client.get_result().target_heading
		hit_layline = False
		self._feedback.status = " Waiting to hit layline. "
		rospy.loginfo(rospy.get_caller_id() + self._feedback.status)

		# Wait until we hit the layline heading
		while not hit_layline and not preempted:
			direct_heading = math.atan2(goal.target.y - self.cur_pos.y, goal.target.x - self.cur_pos.x) * 180 / math.pi
			if (tacking_direction is 1 and direct_heading > (goal.alt_tack_angle + goal.overshoot_angle)) or\
				(tacking_direction is -1 and direct_heading < (goal.alt_tack_angle - goal.overshoot_angle)):
				hit_layline = True

			if self._as.is_preempt_requested():
				self._result.success = success
				self._result.target_heading = self.target_heading
				self._feedback.status = " Preempted"
				rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
				self._as.set_preempted()
				preempted = True
			self.rate.sleep()
		
		self._feedback.status = " Tacking towards mark after hitting layline. "
		rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
		# Reverse tacking direction
		tacking_goal.direction = tacking_direction * -1
		self.tacking_client.send_goal(tacking_goal)
			
		# Adjust time delay until the tack is considered failed, and we return to planning
		if not self.tacking_client.wait_for_result(rospy.Duration(10)):
			self.tacking_client.cancel_goal()
			tacking_goal.direction = tacking_goal.direction * -1
			self.tacking_client.send_goal(tacking_goal)
			if not self.tacking_client.wait_for_result(rospy.Duration(10)):
				self.tacking_client.cancel_goal()
			self._result.success = success
			self._result.target_heading = self.target_heading
			self._feedback.status = " Second tacking goal expired"
			rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
			return

		self.target_heading = self.tacking_client.get_result().target_heading
		success = True
		self._result.success = success
		self._result.target_heading = self.target_heading
		self._feedback.status = " Completed Layline Action. "
		rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
		self._as.set_succeeded(self._result)
		
if __name__ == '__main__':
	rospy.init_node('layline_action')
	server = LaylineAction(rospy.get_name())
	rospy.spin()
