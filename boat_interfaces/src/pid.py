#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import time

pid_is_enabled = False
kp = -1.0
kd = 0.0
ki = 0.0
output_pub = rospy.Publisher('rudder_pid/output', Float64, queue_size=10)
setpoint = 0
pid_input = 0
max_range = 60


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
		print prop, setpoint, pid_input
		# TODO: Add integral and derivative
		integral = 0 
		der = 0
		output = Float64()
		output.data = integral + prop + der
		if output.data > 60:
			output.data = 60
		elif output.data < -60:
			output.data = -60
		output_pub.publish(output)
	

# If the wind heading topic changes, update local wind heading
def setpoint_callback(pid):
	global setpoint
	setpoint = pid.data

def enable_callback(pid):
	global pid_is_enabled
	pid_is_enabled = pid.data

# Determine whether the specified value is between boundA and boundB.
# Note that the order of boundA and boundB do not matter, either can be the upper or lower bound
def is_within_bounds(val, boundA, boundB):
	return (boundA < val and val < boundB) or (boundB < val and val < boundA)


# Conform an input angle to range (-180, 180)
def conform_angle(val):
	return (val + 180) % 360 - 180

def listener():
	# Setup subscribers
	rospy.init_node('pid_node', anonymous=True)
	rospy.Subscriber('rudder_pid/input', Float64, input_callback)
	rospy.Subscriber('rudder_pid/setpoint', Float64, setpoint_callback)
	rospy.Subscriber('rudder_pid/enable', Bool, enable_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
