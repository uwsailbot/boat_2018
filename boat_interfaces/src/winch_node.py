#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from boat_msgs.msg import BoatState
      
import time

wind_dir = 0
request = "Hold"
winch_pos = 0
max_angle = 30  # Closest angle we can sail to the wind
pub = rospy.Publisher('winch', Int32, queue_size=10)
state = BoatState()


def state_callback(new_state):
    global state
    state = new_state


def joy_callback(controller):
    global winch_pos
    global request
    global state
    global pub

    rate = rospy.Rate(100)

    # If we are not in autonomous mode, then use the Dpad to set the sail position
    if state.major is BoatState.MAJ_RC:
        if controller.axes[6] < 0:
            request = "Broad"
            winch_pos = 1500  # 1.5 turns
        elif controller.axes[6] > 0:
            request = "Beam"
            winch_pos = 1100  # Three turns
        elif controller.axes[7] < 0:
            request = "Run"
            winch_pos = 1600  # Fully out
        elif controller.axes[7] > 0:
            request = "Close"
            winch_pos = 0  # Five Turns
        else:
            request = "Hold"
    pub.publish(winch_pos)
    rate.sleep()


def anemometer_callback(wind_direction):
    global winch_pos
    global state
    global wind_dir
    global pub

    wind_dir = wind_direction.data
    rate = rospy.Rate(10)

    # If we are in autonomous mode, set the sail based on the wind direction given by the anemometer
    if state.major is BoatState.MAJ_AUTONOMOUS:
        if wind_dir < (180 + max_angle) and wind_dir > (180 - max_angle):
            new_position = 0
        elif wind_dir >= (180 - max_angle):
            new_position = (600 / (180 - max_angle)) * abs(wind_dir-(max_angle + 180)) + 1000
        else:
            new_position = (600 / (180 - max_angle)) * abs(wind_dir-(180 - max_angle)) + 1000

        # If the change in sail position is significant, then publish a new position
        if abs(new_position - winch_pos) > 100:
            winch_pos = new_position
            pub.publish(winch_pos)
            rospy.loginfo(rospy.get_caller_id() + " Autonomy Request: %f", winch_pos)

    rate.sleep()


def listener():
    # Setup subscribers
    rospy.init_node('joy_to_winch', anonymous=True)
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.Subscriber('boat_state', BoatState, state_callback)
    rospy.Subscriber('anemometer', Float32, anemometer_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
