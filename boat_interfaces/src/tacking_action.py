#! /usr/bin/env python
import actionlib
import boat_msgs.msg
import rospy
from boat_msgs.msg import BoatState
from std_msgs.msg import Bool, Float32

class TackingAction(object):
	# create messages that are used to publish feedback/result
	_feedback = boat_msgs.msg.TackingFeedback()
	_result = boat_msgs.msg.TackingResult()

	def __init__(self, name):
		self._name = name
		self._as = actionlib.SimpleActionServer(self._name, boat_msgs.msg.TackingAction, execute_cb=self.tacking_callback, auto_start = False)
		self._as.start()
		self.layline = rospy.get_param('/boat/layline')
		self.rudder_max = rospy.get_param('boat/rudder_max')
		self.rudder_min = rospy.get_param('boat/rudder_min')
		self.rudder_pos = 90.0
		self.app_wind_shift = 0
		self.target_heading = 0
		self.init_anemometer = 0
		self.init_heading = 0
		self.cur_boat_heading = 0
		self.ane_reading = 0
		self.tacking = False
		self.state = BoatState()
		self.sub_ane = rospy.Subscriber('anemometer', Float32, self.anemometer_callback)
		self.sub_tar = rospy.Subscriber('target_heading', Float32, self.target_callback)
		self.sub_heading = rospy.Subscriber('compass', Float32, self.compass_callback)
		self.sub_state = rospy.Subscriber('boat_state', BoatState, self.boat_state_callback)
		self.rudder_pos_pub = rospy.Publisher('rudder', Float32, queue_size=10)
		self.state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
		self.pid_enable_pub = rospy.Publisher('rudder_pid/enable', Bool, queue_size=10)
		self.target_pub = rospy.Publisher('target_heading', Float32, queue_size=10)
		self.rate = rospy.Rate(100)

	def anemometer_callback(self, anemometer):
		self.ane_reading = anemometer.data

	def target_callback(self, target):
		self.target_heading = target.data

 	def compass_callback(self, heading):
		self.cur_boat_heading = heading.data

	def boat_state_callback(self, state):
		self.state = state

	def tacking_callback(self, goal):
		# helper variables
		success = False
		preempted = False
		
		self._feedback.status = "Entered Action Callback"
		
		# publish info to the console for the user
		rospy.loginfo('Tacking Action: Tacking Action Startup')
		self.state.minor = BoatState.MIN_TACKING
		self.state_pub.publish(self.state)
				
		# Make sure PID is disabled
		self.pid_enable_pub.publish(Bool(False))
		
		self.init_target = self.target_heading
		
		while not success and not preempted:
			# start executing the action
			if goal.direction == 1:
				if self.ane_reading < (180 + self.layline):
					if self.cur_boat_heading < self.init_target:
						# Nothing relevant to publish it for other than debugging and the simulator
						self.target_pub.publish(Float32(self.cur_boat_heading))
					if not self.rudder_pos == self.rudder_max:
						self.rudder_pos = self.rudder_max
						self.rudder_pos_pub.publish(Float32(self.rudder_pos))
				else:
					if self.state.major is not BoatState.MAJ_AUTONOMOUS:
						self.state.minor = BoatState.MIN_COMPLETE
					else:
						self.state.minor = BoatState.MIN_PLANNING
					self.rudder_pos = 90.0
					self.rudder_pos_pub.publish(Float32(self.rudder_pos))
					self.state_pub.publish(self.state)
					success = True
			
			elif goal.direction == -1:
				if self.ane_reading > (180 - self.layline):
					if self.cur_boat_heading > self.init_target:
						# Nothing relevant to publish it for other than debugging and the simulator
						self.target_pub.publish(Float32(self.cur_boat_heading)) 
					if not self.rudder_pos == self.rudder_min:
						self.rudder_pos = self.rudder_min
						self.rudder_pos_pub.publish(Float32(self.rudder_pos))
				else:
					if self.state.major is not BoatState.MAJ_AUTONOMOUS:
						self.state.minor = BoatState.MIN_COMPLETE
					else:
						self.state.minor = BoatState.MIN_PLANNING
					self.rudder_pos = 90.0
					self.rudder_pos_pub.publish(Float32(self.rudder_pos))
					self.state_pub.publish(self.state)
					success = True

			if self._as.is_preempt_requested():
				rospy.loginfo('Tacking Action: Preempted')
				self.rudder_pos = 90.0
				self.rudder_pos_pub.publish(Float32(self.rudder_pos))
				if self.state.major is not BoatState.MAJ_AUTONOMOUS:
					self.state.minor = BoatState.MIN_COMPLETE
				else:
					self.state.minor = BoatState.MIN_PLANNING
				self.state_pub.publish(self.state)
				self._result.success = False
				self._result.target_heading = self.target_heading
				self._feedback.status = "Preempted"
				self._as.set_preempted()
				preempted = True
			self.rate.sleep()
		  
		if success:
			self._result.success = success
			if (goal.direction == -1 and self.cur_boat_heading > self.init_target) or\
					(goal.direction == 1 and self.cur_boat_heading < self.init_target):
						# Publish and return the accurate curent heading for the navigator node to use
						self.target_pub.publish(Float32(self.cur_boat_heading))
						self.target_heading = self.cur_boat_heading
			self._result.target_heading = self.target_heading
			rospy.loginfo('Tacking Action: Success')
			self._as.set_succeeded(self._result)
		
if __name__ == '__main__':
	rospy.init_node('tacking_action')
	server = TackingAction(rospy.get_name())
	rospy.spin()
