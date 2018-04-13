#!/usr/bin/env python
import rospy
from boat_msgs.msg import BoatState, GPS
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32


# Declare global variables needed for the node
rudder_pos = 90  # What the rudder position is (0-180)
cur_boat_heading = 0
ane_reading = 0
wind_heading = 0
state = BoatState()
pid_is_enabled = False
rudder_max = rospy.get_param('boat/interfaces/rudder_max')
rudder_min = rospy.get_param('boat/interfaces/rudder_min')

# Declare the publishers for the node
rudder_pos_pub = rospy.Publisher('rudder', Float32, queue_size=10)
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)

pid_input_pub = rospy.Publisher('rudder_pid/input', Float32, queue_size=10)
pid_enable_pub = rospy.Publisher('rudder_pid/enable', Bool, queue_size=10)
pid_setpoint_pub = rospy.Publisher('rudder_pid/setpoint', Float32, queue_size=10)


# If the boat state topic changes, update local boat state
def boat_state_callback(new_state):
	global state
	global rudder_pos
	global pid_is_enabled
	state = new_state
	
	# If disabled or auto has completed its path, then stop the boat
	if state.major is BoatState.MAJ_DISABLED or (state.major is BoatState.MAJ_AUTONOMOUS and state.minor is BoatState.MIN_COMPLETE):
		rudder_pos = 90
		rudder_pos_pub.publish(Float32(rudder_pos))
		if pid_is_enabled:
			pid_is_enabled = False
			pid_enable_pub.publish(Bool(False))
			rospy.loginfo(rospy.get_caller_id() + " Disabling rudder PID")


# If the wind heading topic changes, update local wind heading
def anemometer_callback(anemometer):
	global ane_reading
	ane_reading = anemometer.data

def enable_callback(pid):
	global pid_is_enabled
	pid_is_enabled = pid.data

# If the gps topic changes, update the pid controller's input value
def compass_callback(compass):
	global cur_boat_heading
	global wind_heading
	global pid_input_pub
	global ane_reading
	
	wind_heading = (ane_reading + compass.data) % 360
	
	cur_boat_heading = compass.data
	heading_msg = Float32(conform_angle(cur_boat_heading))
	pid_input_pub.publish(heading_msg)
	# Don't loginfo here, this would be somewhat redundant as this callback just parses and republishes the same information


# Whenever the PID controller calculates a new effort, apply it to the rudder position
def pid_callback(output):
	global rudder_pos
	
	rudder_pos =  90.0 + output.data 
	rudder_pos_pub.publish(Float32(rudder_pos))
	#rospy.loginfo(rospy.get_caller_id() + " Rudder PID output pos: %f", rudder_pos)


# If the target boat heading topic changes, update rudder PID setpoint
def target_heading_callback(target_heading):
	global wind_heading
	global state
	global pid_is_enabled
	global rudder_pos
	
	# Perhaps a worthwhile check, but not really super important because this callback will never be called if these conditions are not met in path_planning_node
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	# If the PID isn't enabled, activate it
	if not pid_is_enabled:
		pid_is_enabled = True
		pid_enable_pub.publish(Bool(True))
		rospy.loginfo(rospy.get_caller_id() + " Enabling rudder PID")
	
	# We have a new valid setpoint, therefore output it	
	#rospy.loginfo(rospy.get_caller_id() + " New rudder setpoint: %f", target_heading.data)

	pid_setpoint = Float32(conform_angle(target_heading.data))
	pid_setpoint_pub.publish(pid_setpoint)


def joy_callback(controller):
	global rudder_pos
	global state
	global pid_is_enabled
	global rudder_max
	global rudder_min
	
	# Make sure we're in RC control mode
	if state.major is not BoatState.MAJ_RC or state.minor is BoatState.MIN_TACKING:
		return

		
	# Make sure the PID is off
	if pid_is_enabled:
		pid_is_enabled = False
		pid_enable_pub.publish(Bool(False))
		rospy.loginfo(rospy.get_caller_id() + " Disabling rudder PID")
	
	
	# If the boat is not currently tacking, then setup a message to send to the /rudder topic
	rudder_pos_old = rudder_pos
	position_msg = Float32()
	
	# Set the rudder position
	position_msg.data = (90 - ((rudder_max-rudder_min)/2.0* controller.axes[0]))
	
	# Only publish if the change in rudder angle is greater than 5
	if abs(position_msg.data - rudder_pos_old) > 5:
		rudder_pos_pub.publish(position_msg)
		rospy.loginfo(rospy.get_caller_id() + " Read value: %f", controller.axes[0])
		rudder_pos = position_msg.data

# Conform an input angle to range (-180, 180)
def conform_angle(val):
	return (val + 180) % 360 - 180

def listener():
	# Setup subscribers
	rospy.init_node('rudder_node')
	rospy.Subscriber('joy', Joy, joy_callback)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	rospy.Subscriber('target_heading', Float32, target_heading_callback)
	rospy.Subscriber('compass', Float32, compass_callback)
	rospy.Subscriber('rudder_pid/output', Float32, pid_callback)
	rospy.Subscriber('rudder_pid/enable', Bool, enable_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
