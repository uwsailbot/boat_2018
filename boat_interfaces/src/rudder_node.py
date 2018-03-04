#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from boat_msgs.msg import BoatState
from boat_msgs.msg import GPS
import time

# Declare global variables needed for the node
rudder_pos = 90  # What the rudder position is (0-180)
cur_boat_heading = 0
ane_reading = 0
wind_heading = 0
state = BoatState()
pid_is_enabled = False
tacking_direction = 0
layline = 30

# Declare the publishers for the node
rudder_pos_pub = rospy.Publisher('rudder', Float32, queue_size=10)
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)

pid_input_pub = rospy.Publisher('rudder_pid/input', Float64, queue_size=10)
pid_enable_pub = rospy.Publisher('rudder_pid/enable', Bool, queue_size=10)
pid_setpoint_pub = rospy.Publisher('rudder_pid/setpoint', Float64, queue_size=10)


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
	global tacking_direction
	global rudder_pos
	global state
	
	ane_reading = anemometer.data
	
	# No need to check tacking directions if we are in autonomous or not tacking
	if state.major is not BoatState.MAJ_RC or state.minor is not BoatState.MIN_TACKING:
		return
	
	# Based on direction of tack, keep the rudder turned while the boat crosses wind and passes 30 		
	# degrees to the other side, then set the rudder back to 90
	if tacking_direction == 1:
		if ane_reading < (180 + layline):
			if not rudder_pos == 150.0:
				rudder_pos = 150.0
				rudder_pos_pub.publish(Float32(rudder_pos))
			rate.sleep()
		else:
			state.minor = BoatState.MIN_COMPLETE
			tacking_direction = 0
			rudder_pos = 90.0
			rudder_pos_pub.publish(Float32(rudder_pos))
			boat_state_pub.publish(state)
	
	
	elif tacking_direction == -1:
		if ane_reading > (180 - layline):
			if not rudder_pos == 30.0:
				rudder_pos_msg.data = 30.0
				rudder_pos_pub.publish(Float32(rudder_pos))
			#print ane_reading, anemometer.data
		else:
			state.minor = BoatState.MIN_COMPLETE
			tacking_direction = 0
			rudder_pos = 90.0
			rudder_pos_pub.publish(Float32(rudder_pos))
			boat_state_pub.publish(state)
	
	#rate = rospy.Rate(10)
	#rate.sleep()


# If the gps topic changes, update the pid controller's input value
def compass_callback(compass):
	global cur_boat_heading
	global wind_heading
	
	wind_heading = (ane_reading + compass.data) % 360
	
	cur_boat_heading = compass.data
	heading_msg = Float64((cur_boat_heading+180)%360-180)
	pid_input_pub.publish(heading_msg)
	# Don't loginfo here, this would be somewhat redundant as this callback just parses and republishes the same information


# Whenever the PID controller calculates a new effort, apply it to the rudder position
def pid_callback(output):
	global rudder_pos
	
	rudder_pos =  90.0 + output.data 
	rudder_pos_pub.publish(Float32(rudder_pos))
	rospy.loginfo(rospy.get_caller_id() + " Rudder PID output pos: %f", rudder_pos)


# If the target boat heading topic changes, update rudder PID setpoint
def target_heading_callback(target_heading):
	global wind_heading
	global state
	global cur_boat_heading
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
	rospy.loginfo(rospy.get_caller_id() + " New rudder setpoint: %f", target_heading.data)

	# If the current heading and the new heading are on opposite sides of the wind, we need to tack
#	opp_wind = (wind_heading+180)%360
#	if (is_within_bounds(target_heading.data, wind_heading, opp_wind) and not is_within_bounds(cur_boat_heading, wind_heading, opp_wind)) or\
#		(is_within_bounds(cur_boat_heading, wind_heading, opp_wind) and not is_within_bounds(target_heading.data, wind_heading, opp_wind)):

	# Decide whether we need to tack through the wind
	if (abs(target_heading.data - cur_boat_heading)) > 180:
		boat_dir = -1 # Clockwise
		print "clockwise"
	else:
		boat_dir = 1 # Counter-clockwise
		print "cc"
	
	wind_coming = (wind_heading + 180) % 360 # Which direction the wind is coming from

	if (boat_dir is -1 and not is_within_bounds(wind_coming, cur_boat_heading, target_heading.data)) or\
		(boat_dir is 1 and is_within_bounds(wind_coming, cur_boat_heading, target_heading.data)):
		print wind_coming, cur_boat_heading, target_heading.data, boat_dir
		state.minor = BoatState.MIN_TACKING
		boat_state_pub.publish(state)
		rospy.loginfo(rospy.get_caller_id() + " Boat State = 'Autonomous - Tacking'")
		
		pid_setpoint = Float64(conform_angle(target_heading.data))
		pid_setpoint_pub.publish(pid_setpoint)
		
		# TODO: Find a good tolerance. We don't need to wait for the boat to be completely on target, but it needs to be well past the wind
		while abs(target_heading.data-cur_boat_heading) < 10:
			pass
			
		state.minor = BoatState.MIN_PLANNING
		boat_state_pub.publish(state)
		rospy.loginfo(rospy.get_caller_id() + " Boat State = 'Autonomous - Planning'")
	
	# Otherwise, we don't need to tack, so simply update the controller's setpoint
	else:
		pid_setpoint = Float64(conform_angle(target_heading.data))
		pid_setpoint_pub.publish(pid_setpoint)


def joy_callback(controller):
	global rudder_pos
	global state
	global tacking_direction
	global pid_is_enabled
	
	# Make sure we're in RC control mode
	if state.major is not BoatState.MAJ_RC:
		return
	
	# x is pressed then request a tack
	if controller.buttons[0] and tacking_direction == 0:
		rospy.loginfo(rospy.get_caller_id() + "Tack requested.")
		state.minor = BoatState.MIN_TACKING
		
		# If a tack is requested, figure out which side we are tacking and set the rudder accordingly
		if ane_reading > 180:
			rudder_pos = 30.0
			tacking_direction = -1
		else:
			rudder_pos = 150.0
			tacking_direction = 1
		
		rudder_pos_pub.publish(Float32(rudder_pos))
		boat_state_pub.publish(state)
	
	# o is pressed, therefore cancelling a previously requested tack
	elif controller.buttons[2] and not tacking_direction == 0:
		# Reset rudder and change tacking state
		tacking_direction = 0
		rospy.loginfo(rospy.get_caller_id() + "Tack cancelled")
		state.minor = BoatState.MIN_COMPLETE
		rudder_pos = 90.0
		rudder_pos_pub.publish(Float32(rudder_pos))
		boat_state_pub.publish(state)
	# Whenever the boat is in RC mode, except for RC - Tacking...
	if state.minor is not BoatState.MIN_TACKING:
		
		# Make sure the PID is off
		if pid_is_enabled:
			pid_is_enabled = False
			pid_enable_pub.publish(Bool(False))
			rospy.loginfo(rospy.get_caller_id() + " Disabling rudder PID")
		
		
		# If the boat is not currently tacking, then setup a message to send to the /rudder topic
		rudder_pos_old = rudder_pos
		position_msg = Float32()
		
		# Set the rudder position to be a min of 30 and max of 150
		position_msg.data = (90 - (60 * controller.axes[0]))
		
		# Only publish if the change in rudder angle is greater than 5
		if abs(position_msg.data - rudder_pos_old) > 5:
			rudder_pos_pub.publish(position_msg)
			rospy.loginfo(rospy.get_caller_id() + " Read value: %f", controller.axes[0])
			rudder_pos = position_msg.data


# Determine whether the specified value is between boundA and boundB.
# Note that the order of boundA and boundB do not matter, either can be the upper or lower bound
def is_within_bounds(val, boundA, boundB):
	return (boundA < val and val < boundB) or (boundB < val and val < boundA)


# Conform an input angle to range (-180, 180)
def conform_angle(val):
	return (val + 180) % 360 - 180

def listener():
	# Setup subscribers
	rospy.init_node('rudder_node', anonymous=True)
	rospy.Subscriber('joy', Joy, joy_callback)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	rospy.Subscriber('target_heading', Float32, target_heading_callback)
	rospy.Subscriber('compass', Float32, compass_callback)
	rospy.Subscriber('rudder_pid/output', Float64, pid_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
