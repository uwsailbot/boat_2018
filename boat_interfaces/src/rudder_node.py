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
wind_heading = 0
state = BoatState()
pid_is_enabled = False
tack_request = False
tacking_direction = 0
wind_dir = 0

# Declare the publishers for the node
rudder_pos_pub = rospy.Publisher('rudder', Float32, queue_size=10)
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)

pid_input_pub = rospy.Publisher('rudder_pid/input', Float64, queue_size=10)
pid_enable_pub = rospy.Publisher('rudder_pid/enable', Bool, queue_size=10)
pid_setpoint_pub = rospy.Publisher('rudder_pid/setpoint', Float64, queue_size=10)


# If the boat state topic changes, update local boat state
def boat_state_callback(new_state):
	global state
	state = new_state
	
	
# If the wind heading topic changes, update local wind heading
def wind_callback(heading):
	global wind_heading
	global boat_state_pub
	global rudder_pos_pub
	global tacking_direction
	global rudder_pos
	global state

	rate = rospy.Rate(10)
	rudder_pos_msg = Float32()
	wind_heading = heading.data

	# Based on direction of tack, keep the rudder turned while the boat crosses wind and passes 30 		# degrees to the other side, then set the rudder back to 90
	if tacking_direction == 1:
		if direction.data > 150:
			if not rudder_pos == 150.0:
				rudder_pos_msg.data = 150.0
				rudder_pos = rudder_pos_msg.data
				pub_rudder.publish(rudder_pos_msg)
			rate.sleep()
		else:
			state.minor = BoatState.MIN_COMPLETE
			tacking_direction = 0
			rudder_pos_msg.data = 90.0
			rudder_pos_pub.publish(rudder_pos_msg)
			boat_state_pub.publish(state)


	elif tacking_direction == -1:
		if direction.data < 210:
			if not rudder_pos == 30.0:
				rudder_pos_msg.data = 30.0
				rudder_pos = rudder_pos_msg.data
				rudder_pos_pub.publish(rudder_pos_msg)
			rate.sleep()
		else:
			state.minor = BoatState.MIN_COMPLETE
			tacking_direction = 0
			rudder_pos_msg.data = 90.0
			rudder_pos_pub.publish(rudder_pos_msg)
			boat_state_pub.publish(state)

	rate.sleep()


# If the gps topic changes, update the pid controller's input value
def gps_callback(gps):
	global cur_boat_heading
	
	cur_boat_heading = gps.track
	heading_msg = Float64()
	heading_msg.data = cur_boat_heading
	pid_input_pub.publish(heading_msg)
	# Don't loginfo here, this would be somewhat redundant as this callback just parses and republishes the same information


# Whenever the PID controller calculates a new effort, apply it to the rudder position
def pid_callback(output):
	global rudder_pos
	
	rudder_pos = output
	position_msg = Float32()
	position_msg.data = rudder_pos
	rudder_pos_pub.publish(position_msg)
	rospy.loginfo(rospy.get_caller_id() + " PID Rudder pos: %f", rudder_pos)
	
	
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
		pid_state = Bool()
		pid_state.data = True
		pid_enable_pub.publish(pid_state)
		rospy.loginfo(rospy.get_caller_id() + " Enabling rudder PID")
	
	# We have a new valid setpoint, therefore output it	
	rospy.loginfo(rospy.get_caller_id() + " New rudder setpoint: %f", target_heading)

	# If the current heading and the new heading are on opposite sides of the wind, we need to tack
	opp_wind = (wind_heading+180)%360
	if (is_in_bounds(target_heading, wind_heading, opp_wind) and not is_in_bounds(cur_boat_heading, wind_heading, opp_wind)) or\
		(is_in_bounds(cur_boat_heading, wind_heading, opp_wind) and not is_in_bounds(target_heading, wind_heading, opp_wind)):
		
		state.minor = BoatState.MIN_TACKING
		boat_state_pub.publish(state)
		rospy.loginfo(rospy.get_caller_id() + " Boat State = 'Autonomous - Tacking'")
		
		pid_setpoint_pub.publish(target_heading)
		# TODO: Find a good tolerance. We don't need to wait for the boat to be completely on target, but it needs to be well past the wind
		while abs(target_heading-cur_boat_heading) < 10:
			pass
			
		state.minor = BoatState.MIN_PLANNING
		boat_state_pub.publish(state)		
		rospy.loginfo(rospy.get_caller_id() + " Boat State = 'Autonomous - Planning'")
	
	# Otherwise, we don't need to tack, so simply update the controller's setpoint
	else:		
		pid_setpoint_pub.publish(target_heading)



def joy_callback(controller):
	global rudder_pos
	global rudder_pos_pub
	global state
	global tacking_direction
	global boat_state_pub
	global wind_heading
	global pid_is_enabled

	# Make sure we're in RC control mode
	if state.major is not BoatState.MAJ_RC:
		return

	# x is pressed then request a tack
	if controller.buttons[0] and tacking_direction == 0:
		rospy.loginfo(rospy.get_caller_id() + "Tack requested.")
		state.minor = BoatState.MIN_TACKING

		# If a tack is requested, figure out which side we are tacking and set the rudder accordingly
		if wind_dir < 180:
			rudder_pos_pub.publish(30.0)
			rudder_pos = 30.0
			tacking_direction = -1
		else:
			rudder_pos_pub.publish(150.0)
			rudder_pos = 150.0
			tacking_direction = 1

		boat_state_pub.publish(state)

	# o is pressed, therefore cancelling a previously requested tack
	elif controller.buttons[2] and not tacking_direction == 0:
		# Reset rudder and change tacking state
		tacking_direction = 0
		rospy.loginfo(rospy.get_caller_id() + "Tack cancelled")
		state.minor = BoatState.MIN_COMPLETE
		rudder_pos_msg = Float32()
		rudder_pos_msg.data = 90.0
		rudder_pos_pub.publish(rudder_pos_msg)
		boat_state_pub.publish(state)
		rudder_pos = rudder_pos_msg.data

	# Whenever the boat is in RC mode, except for RC - Tacking...
	if state.minor is not BoatState.MIN_TACKING:
	
		# Make sure the PID is off
		if pid_is_enabled:
			pid_is_enabled = False
			pid_state = Bool()
			pid_state.data = False
			pid_enable_pub.publish(pid_state)
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
	
	
def listener():
	# Setup subscribers
	rospy.init_node('rudder_node', anonymous=True)
	rospy.Subscriber('joy', Joy, joy_callback)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('anemometer', Float32, wind_callback)
	rospy.Subscriber('target_heading', Float32, target_heading_callback)
	rospy.Subscriber('gps_raw', GPS, gps_callback)
	rospy.Subscriber('rudder_pid/output', Float64, pid_callback)
	rospy.spin()
	

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
