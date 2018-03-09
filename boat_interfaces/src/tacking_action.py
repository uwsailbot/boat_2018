#! /usr/bin/env python
import rospy
import actionlib
import boat_msgs.msg 
from std_msgs.msg import Float32, Bool
from boat_msgs.msg import BoatState

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
		self.state = BoatState()
		self._sub = rospy.Subscriber('anemometer', Float32, self.anemometer_callback)
		self.rudder_pos_pub = rospy.Publisher('rudder', Float32, queue_size=10)
		self.state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
		self.pid_enable_pub = rospy.Publisher('rudder_pid/enable', Bool, queue_size=10)
		self.rate = rospy.Rate(100)

	def anemometer_callback(self, anemometer):
		self.ane_reading = anemometer.data
	  
	def tacking_callback(self, goal):
		# helper variables
		
		success = False
		preempted = False
		
		self._feedback.status = "Entered Action Callback"
		
		# publish info to the console for the user
		rospy.loginfo('Tacking Action: Tacking Action Startup')
		self.state = goal.boat_state
		self.state.minor = BoatState.MIN_TACKING
		self.state_pub.publish(self.state)
		
		# Make sure PID is disabled
		self.pid_enable_pub.publish(Bool(False))
		
		while not success and not preempted:
			# start executing the action
			if goal.direction == 1:
				if self.ane_reading < (180 + self.layline):
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
				self._feedback.status = "Preempted"
				self._as.set_preempted()
				preempted = True
			self.rate.sleep()
		  
		if success:
			self._result.success = success
			rospy.loginfo('Tacking Action: Success')
			self._as.set_succeeded(self._result)
		
if __name__ == '__main__':
	rospy.init_node('tacking_action')
	server = TackingAction(rospy.get_name())
	rospy.spin()
