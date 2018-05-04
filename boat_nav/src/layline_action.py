#! /usr/bin/env python
import actionlib
import boat_msgs.msg
import rospy
import math
from boat_msgs.msg import BoatState, Point, TackingAction, TackingGoal, GPS
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
		self.min_speed = rospy.get_param('/boat/nav/min_tacking_speed')
		self.layline = rospy.get_param('/boat/nav/layline')
		self.target_heading = 0
		self.new_target = False
		self.boat_speed = 0
		self.ane_reading = 0
		self.apparent_wind_heading = 0
		self.wind_coming = 0
		self.compass = 0
		self.sub_ane = rospy.Subscriber('anemometer', Float32, self.anemometer_callback)
		self.target_heading_sub = rospy.Subscriber('target_heading', Float32, self.target_heading_callback)
		self.target_sub = rospy.Subscriber('target_point', Point, self.target_callback)
		self.sub_heading = rospy.Subscriber('compass', Float32, self.compass_callback)
		self.target_pub = rospy.Publisher('target_heading', Float32, queue_size=10)
		self.pos_sub = rospy.Subscriber('lps', Point, self.position_callback)
		self.gps_sub = rospy.Subscriber('gps_raw', GPS, self.gps_callback)
		self.rate = rospy.Rate(100)
		self.tacking_client.wait_for_server()

	def target_heading_callback(self, new_target_heading):
		self.target_heading = new_target_heading.data

	def position_callback(self, position):
		self.cur_pos = position

	def target_callback(self, new_target):
		self.new_target = True
		print "new target"

	def gps_callback(self, gps):
		self.boat_speed = gps.speed * 0.514444 # Knots to m/s

	def is_within_bounds(self, val, boundA, boundB):
		if boundA < boundB:
			val -= boundA
			boundB -= boundA
			boundA = 0
		else:
			val -= boundB
			boundA -= boundB
			boundB = 0
		if val < 0:
			val += 360
		val = val % 360
		return (boundA <= val and val <= boundB) or (boundB <= val and val <= boundA)

	def gtAngle(self, angle1, angle2):
		comp_angle = (angle2 + 180) % 360
		if angle2 >= 180:
			return not self.is_within_bounds(angle1, angle2, comp_angle)
		else:
			return self.is_within_bounds(angle1, angle2, comp_angle)
	
	def ltAngle(self, angle1, angle2):
		comp_angle = (angle2 + 180) % 360
		if angle2 >= 180:
			return self.is_within_bounds(angle1, angle2, comp_angle)
		else:
			return not self.is_within_bounds(angle1, angle2, comp_angle)

	def anemometer_callback(self, new_heading):
		self.ane_reading = new_heading.data
		self.apparent_wind_heading = (self.ane_reading + self.compass) % 360
		self.wind_coming = (self.apparent_wind_heading + 180) % 360

	def compass_callback(self, compass):
		self.compass = compass.data
		self.apparent_wind_heading = (self.ane_reading + self.compass) % 360
		self.wind_coming = (self.apparent_wind_heading + 180) % 360

	def layline_callback(self, goal):
		# helper variables
		success = False
		preempted = False
		self.new_target = False
		
		# publish info to the console for the user
		self._feedback.status = " Entered Layline Action Callback. "
		rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
		if self.gtAngle(goal.alt_tack_angle, self.wind_coming):
			tacking_direction = 1
		else:
			tacking_direction = -1
		self._feedback.status = " Tacking away from mark to hit layline. "
		rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
		new_target = self.wind_coming - tacking_direction * self.layline
		print new_target, self.wind_coming, tacking_direction
		if new_target < 0:
			new_target += 360
		new_target = new_target % 360
		self.target_heading = new_target
		self.target_pub.publish(Float32(self.target_heading))

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
		while (not hit_layline or self.boat_speed < self.min_speed) and not preempted:
			direct_heading = math.atan2(goal.target.y - self.cur_pos.y, goal.target.x - self.cur_pos.x) * 180 / math.pi
			direct_heading = (direct_heading + 360) % 360
			if goal.alt_tack_angle - goal.overshoot_angle < 0:
				lower_bound = goal.alt_tack_angle - goal.overshoot_angle + 360
			else:
				lower_bound = goal.alt_tack_angle - goal.overshoot_angle
			upper_bound = (goal.alt_tack_angle + goal.overshoot_angle) % 360
			if (tacking_direction is 1 and self.gtAngle(direct_heading, upper_bound)) or\
				(tacking_direction is -1 and self.ltAngle(direct_heading, lower_bound)):
				hit_layline = True

			if self._as.is_preempt_requested() or self.new_target:
				self._result.success = success
				self._result.target_heading = self.target_heading
				self._feedback.status = " Preempted"
				rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
				self._as.set_preempted()
				preempted = True
			self.rate.sleep()
		
		# If preempted in the loop, exit the action
		if preempted:
			return

		self._feedback.status = " Tacking towards mark after hitting layline. "
		rospy.loginfo(rospy.get_caller_id() + self._feedback.status)
		# Reverse tacking direction
		tacking_goal.direction = tacking_direction * -1
		self.target_heading = goal.alt_tack_angle
		self.target_pub.publish(Float32(self.target_heading))
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
