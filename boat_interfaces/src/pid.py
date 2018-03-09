#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Bool
import time

pid_is_enabled = False
kp = -1.0
kd = 0.0
ki = 0.0
output_pub = rospy.Publisher('rudder_pid/output', Float32, queue_size=10)
setpoint = 0
pid_input = 0
max_range = (rospy.get_param('/boat/rudder_max') - rospy.get_param('/boat/rudder_min')) / 2.0


# If the boat state topic changes, update local boat state
def input_callback(pid):
	global pid_input
	global setpoint
	global kp
	global max_range
	global pid_is_enabled
	if pid_is_enabled:
		pid_input = pid.data
		e = setpoint - pid_input
		if e > 180:
			e -= 360
		elif e < -180:
			e += 360
		prop= kp * e
		# TODO: Add integral and derivative
		integral = 0 
		der = 0
		output = Float32()
		output.data = integral + prop + der
		if output.data > 60:
			output.data = 60
		elif output.data < -60:
			output.data = -60
		output_pub.publish(output)
	

# If the wind heading topic changes, update local wind heading
def setpoint_callback(setpt):
	global setpoint
	setpoint = setpt.data

def enable_callback(is_enabled):
	global pid_is_enabled
	pid_is_enabled = is_enabled.data

def listener():
	# Setup subscribers
	rospy.init_node('pid_node')
	rospy.Subscriber('rudder_pid/input', Float32, input_callback)
	rospy.Subscriber('rudder_pid/setpoint', Float32, setpoint_callback)
	rospy.Subscriber('rudder_pid/enable', Bool, enable_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
