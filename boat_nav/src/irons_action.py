#!/usr/bin/env python
import rospy
from actionlib import SimpleActionServer
from boat_msgs.msg import IronsAction as IronsActionMsg, IronsFeedback, IronsResult
from std_msgs.msg import Bool, Float32

class IronsAction(object):

	# create messages that are used to publish feedback/result
	_feedback = IronsFeedback()
	_result = IronsResult()

	def __init__(self, name):
		self._name = name
		self._as = SimpleActionServer(self._name, IronsActionMsg, execute_cb=self.execute, auto_start = False)
		self._as.start()
		
		self.ane_reading = 0
		self.heading = 0
		
		# Subscribers
		rospy.Subscriber('anemometer', Float32, self.anemometer_callback)
		rospy.Subscriber('compass', Float32, self.heading_callback)
		
		# Publishers
		self.target_heading_pub = rospy.Publisher('rudder_pid/setpoint', Float32, queue_size=10)
		self.pid_enable_pub = rospy.Publisher('rudder_pid/enable', Bool, queue_size=10)
		
		self.rate = rospy.Rate(100)
	
	def heading_callback(self, compass):
		self.heading = compass.data
	
	def anemometer_callback(self, anemometer):
		self.ane_reading = anemometer.data
	
	def execute(self, goal):
		self.goal = goal
		
		success = False
		preempted = False
		
		rospy.loginfo('Irons Action: Started. Navigating directly upwind.')
		self._feedback.status = "Irons Action: Started"
		
		# Make sure PID is enabled
		self.target_heading_pub.publish(self.heading)
		self.pid_enable_pub.publish(True)
		
		on_target_count = 0
		
		# Start executing the action
		while not success and not preempted:
			
			target = self.ane_reading+self.heading-180
			error = target - self.heading
			
			if abs(error) < 5:
				on_target_count += 1
			else:
				on_target_count = 0
			
			if on_target_count > 10:
				success = True
			
			self.target_heading_pub.publish(target)
			
			# If the action was preempted...
			if self._as.is_preempt_requested():
				rospy.loginfo('Irons Action: Preempted')
				# Cleanup
				self._result.success = False
				self._feedback.status = "Preempted"
				self._as.set_preempted()
				preempted = True
			
			# Sleep
			self.rate.sleep()
		
		# Once we're outside of the loop, if we succeeded...
		if success:
			self._result.success = success
			rospy.loginfo('Irons Action: Success')
			self._as.set_succeeded(self._result)


if __name__ == '__main__':
	rospy.init_node('irons_action')
	server = IronsAction(rospy.get_name())
	rospy.spin()
