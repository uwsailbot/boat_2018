#!/usr/bin/env python
import actionlib
import rospy
import time
from boat_msgs.msg import BoatState, TackingAction, TackingGoal
from sensor_msgs.msg import Joy
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
	
	# If R1 is pushed but L1 and PS aren't, set mode = autonomous
	if not controller.buttons[4] and controller.buttons[5] and not controller.buttons[8] and state.major != BoatState.MAJ_AUTONOMOUS:
		state.major = BoatState.MAJ_AUTONOMOUS
		state.minor = BoatState.MIN_COMPLETE
		#state.challenge = BoatState.CHA_NAV
		pub.publish(state)
	
	# If L1 is pushed but R1 and PS aren't, set mode = RC
	elif controller.buttons[4] and not controller.buttons[5] and not controller.buttons[8] and state.major != BoatState.MAJ_RC:
		state.major = BoatState.MAJ_RC
		state.minor = BoatState.MIN_COMPLETE
		state.challenge = BoatState.CHA_NAV
		pub.publish(state)
	
	# If PS is pushed but L1 and R1 aren't, set mode = disabled
	elif not controller.buttons[4] and not controller.buttons[5] and controller.buttons[8] and state.major != BoatState.MAJ_DISABLED:
		state.major = BoatState.MAJ_DISABLED
		state.minor = BoatState.MIN_COMPLETE
		state.challenge = BoatState.CHA_NAV
		pub.publish(state)

	# If PS is pushed but L1 and R1 aren't, set mode = disabled
	elif controller.buttons[1] and not controller.buttons[3]:
		state.challenge = (state.challenge - 1) % 5
		pub.publish(state)

	# If PS is pushed but L1 and R1 aren't, set mode = disabled
	elif not controller.buttons[1] and controller.buttons[3]:
		state.challenge = (state.challenge + 1) % 5
		pub.publish(state)

	# x is pressed then request a tack
	if controller.buttons[0] and state.major is BoatState.MAJ_RC:
		rospy.loginfo(rospy.get_caller_id() + "Tack requested.")
		# If a tack is requested, figure out which side we are tacking and set the rudder accordingly
		if ane_reading > 180:
			tacking_direction = -1
			goal = TackingGoal(direction = tacking_direction)
		else:
			tacking_direction = 1
			goal = TackingGoal(direction = tacking_direction)
		client.send_goal(goal)

	
	# o is pressed, therefore cancelling a previously requested tack
	elif controller.buttons[2] and state.major is BoatState.MAJ_RC:
		# Reset rudder and change tacking state
		tacking_direction = 0
		rospy.loginfo(rospy.get_caller_id() + "Tack cancelled")
		client.cancel_goal()
	
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
