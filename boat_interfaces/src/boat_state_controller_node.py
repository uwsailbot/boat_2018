#!/usr/bin/env python
import actionlib
import rospy
import time
from boat_msgs.msg import BoatState, TackingAction, TackingGoal, Joy
from std_msgs.msg import Float32

state = BoatState()
tacking_direction = 0
ane_reading = 0
pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
client = actionlib.SimpleActionClient('tacking_action', TackingAction)

def state_callback(new_state):
	global state
	state = new_state

# If the wind heading topic changes, update local wind heading
def anemometer_callback(anemometer):
	global ane_reading
	ane_reading = anemometer.data

def joy_callback(controller):
	global pub
	global state
	global tacking_direction
	rate = rospy.Rate(100)
	# If switch A is in the up position, set auto
	if controller.switch_a == Joy.SWITCH_UP and state.major is not BoatState.MAJ_AUTONOMOUS:
		state.major = BoatState.MAJ_AUTONOMOUS
		state.minor = BoatState.MIN_COMPLETE
		#state.challenge = BoatState.CHA_NAV
		pub.publish(state)
	
	# If switch A is in the neutral position, set rc
	elif controller.switch_a == Joy.SWITCH_MIDDLE and state.major is not BoatState.MAJ_RC:
		state.major = BoatState.MAJ_RC
		state.minor = BoatState.MIN_COMPLETE
		state.challenge = BoatState.CHA_NAV
		pub.publish(state)
	
	# If switch A is in the down position, set disabled
	elif controller.switch_a == Joy.SWITCH_DOWN and state.major is not BoatState.MAJ_DISABLED:
		state.major = BoatState.MAJ_DISABLED
		state.minor = BoatState.MIN_COMPLETE
		state.challenge = BoatState.CHA_NAV
		pub.publish(state)

	# Use VR nob to switch challenge modes
	if controller.vr < 200 and state.challenge is not BoatState.CHA_STATION:
		state.challenge = BoatState.CHA_STATION
		pub.publish(state)
	elif controller.vr >= 200 and controller.vr < 400 and state.challenge is not BoatState.CHA_LONG:
		state.challenge = BoatState.CHA_LONG
		pub.publish(state)
	elif controller.vr >= 400 and controller.vr < 600 and state.challenge is not BoatState.CHA_NAV:
		state.challenge = BoatState.CHA_NAV
		pub.publish(state)
	elif controller.vr >= 600 and controller.vr < 800 and state.challenge is not BoatState.CHA_SEARCH:
		state.challenge = BoatState.CHA_SEARCH
		pub.publish(state)
	elif controller.vr >= 800 and state.challenge is not BoatState.CHA_AVOID:
		state.challenge = BoatState.CHA_AVOID
		pub.publish(state)

	rate.sleep()

def listener():
	# Setup subscribers
	rospy.init_node('boat_state_controller_node')
	rospy.Subscriber('joy', Joy, joy_callback)
	rospy.Subscriber('boat_state', BoatState, state_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	client.wait_for_server()
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
