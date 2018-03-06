#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from boat_msgs.msg import BoatState
	  
import time

ane_reading = 0
layline = rospy.get_param('/boat/layline')  # Closest angle we can sail to the wind
winch_min = rospy.get_param('/boat/winch_min')
winch_max = rospy.get_param('/boat/winch_max')
winch_pub = rospy.Publisher('winch', Int32, queue_size=10)
state = BoatState()
winch_pos = winch_max


def state_callback(new_state):
	global state
	global winch_pos
	global winch_pub
	global winch_max

	state = new_state

	if state.major is BoatState.MAJ_DISABLED or (state.major is BoatState.MAJ_AUTONOMOUS and state.minor is BoatState.MIN_COMPLETE):
		winch_pos = winch_max # Pull sails in all the way
		winch_pub.publish(winch_pos)


def joy_callback(controller):
	global winch_pos
	global state
	global winch_pub
	global winch_min
	global winch_max
	
	winch_range = winch_max - winch_min
	rate = rospy.Rate(100)

	# If we are not in autonomous mode, then use controller to set the sail
	if state.major is BoatState.MAJ_RC:
		# Use D-Pad
		if controller.axes[6] < 0:
			winch_pos = winch_min + winch_range / 2.0 # 50% in, Beam Reach
		elif controller.axes[6] > 0:
			winch_pos = winch_min + winch_range * 3.0 / 2 # 75% in, Broad Reach
		elif controller.axes[7] < 0:
			winch_pos = winch_min # 0% in, Run
		elif controller.axes[7] > 0:
			winch_pos = winch_max # 100% in, Close Hauled
		# Use Right Joystick, up to pull in, down to let out
		# TODO: Increase the sleep on decrease the request value depending how fast the actual winch can pull in
		elif abs(controller.axes[4]) >= 0.1: 
			winch_pos += 50 * controller.axes[4]
			if winch_pos > winch_max:
				winch_pos = winch_max
			elif winch_pos < winch_min:
				winch_pos = winch_min

		winch_pub.publish(Int32(winch_pos))
	rate.sleep()


def anemometer_callback(anemometer):
	global winch_pos
	global state
	global ane_reading
	global winch_pub
	global winch_max
	global winch_min
	global layline

	ane_reading = anemometer.data
	wind_rel_boat = ane_reading - 180 # set midpoint to zero, right side < 0,left side >0

	# If we are in autonomous mode, set the sail based on the wind direction given by the anemometer
	if state.major is BoatState.MAJ_AUTONOMOUS and state.minor is not BoatState.MIN_COMPLETE:
		if abs(wind_rel_boat) <= layline:
			new_position = winch_max
			
		else:
			new_position = winch_max - ((winch_max - winch_min) / (180 - layline) * (abs(wind_rel_boat)-layline))

		# If the change in sail position is significant, then publish a new position
		if abs(new_position - winch_pos) > 50:
			winch_pos = new_position
			winch_pub.publish(Int32(winch_pos))
			rospy.loginfo(rospy.get_caller_id() + " Autonomy Request: %f", winch_pos)

def listener():
	# Setup subscribers
	rospy.init_node('winch',)
	rospy.Subscriber('joy', Joy, joy_callback)
	rospy.Subscriber('boat_state', BoatState, state_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
