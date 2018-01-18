#!/usr/bin/env python
import rospy
import roslaunch
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from boat_msgs.msg import BoatState
import time

tack_request = False
tacking_direction = 0
pub_state = rospy.Publisher('boat_state', BoatState, queue_size=10)
pub_rudder = rospy.Publisher('rudder', Float32, queue_size=10)
rudder_pos = 90.0
wind_dir = 0
state = 0

# New state callback
def state_callback(new_state):
    global state
    state = new_state


# New joystick reading callback
def joy_callback(controller):
    global tacking_direction
    global pub_state
    global pub_rudder
    global rudder_pos
    global wind_dir
    global state

	# Make sure we're in RC control mode
    if state.major is not BoatState.MAJ_RC:
        return

    # x is pressed then request a tack
    if controller.buttons[0] and tacking_direction == 0:
        rospy.loginfo(rospy.get_caller_id() + "Tack requested.")
        state.minor = BoatState.MIN_TACKING

        # If a tack is requested, figure out which side we are tacking and set the rudder accordingly
        if wind_dir < 180:
            pub_rudder.publish(30.0)
            rudder_pos = 30.0
            tacking_direction = -1
        else:
            pub_rudder.publish(150.0)
            rudder_pos = 150.0
            tacking_direction = 1

        pub_state.publish(state)

    # o is pressed, therefore cancelling a previously requested tack
    elif controller.buttons[2] and not tacking_direction == 0:
        # Reset rudder and change tacking state
        tacking_direction = 0
        rospy.loginfo(rospy.get_caller_id() + "Tack cancelled")
        state.minor = BoatState.MIN_COMPLETE
        rudder_pos_msg = Float32()
        rudder_pos_msg.data = 90.0
        pub_rudder.publish(rudder_pos_msg)
        pub_state.publish(state)
        rudder_pos = rudder_pos_msg.data


def anemometer_callback(wind_direction):
    global pub_state
    global pub_rudder
    global tacking_direction
    global rudder_pos
    global wind_dir
    global state

    rate = rospy.Rate(10)
    rudder_pos_msg = Float32()
    wind_dir = wind_direction.data

    # Based on direction of tack, keep the rudder turned while the boat crosses wind and passes 30 degrees to the
    # other side, then set the rudder back to 90
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
            pub_rudder.publish(rudder_pos_msg)
            pub_state.publish(state)


    elif tacking_direction == -1:
        if direction.data < 210:
            if not rudder_pos == 30.0:
                rudder_pos_msg.data = 30.0
                rudder_pos = rudder_pos_msg.data
                pub_rudder.publish(rudder_pos_msg)
            rate.sleep()
        else:
            state.minor = BoatState.MIN_COMPLETE
            tacking_direction = 0
            rudder_pos_msg.data = 90.0
            pub_rudder.publish(rudder_pos_msg)
            pub_state.publish(state)

    rate.sleep()


def listener():
    rospy.init_node('joy_to_tack', anonymous=True)
    rospy.Subscriber('boat_state', BoatState, state_callback)
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.Subscriber('anemometer', Float32, anemometer_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
