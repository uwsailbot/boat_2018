#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from boat_msgs.msg import BoatState

state = BoatState()
      
import time
pub = rospy.Publisher('boat_state', BoatState, queue_size=10)

def state_callback(new_state):
    global state
    state = new_state

def joy_callback(controller):
    global pub
    global state
    rate = rospy.Rate(10)

    # If R1 is pushed but L1 and PS aren't, set mode = autonomous
    if not controller.buttons[4] and controller.buttons[5] and not controller.buttons[8] and state.major != BoatState.MAJ_AUTONOMOUS:
        state.major = BoatState.MAJ_AUTONOMOUS
        pub.publish(state)
        
    # If L1 is pushed but R1 and PS aren't, set mode = RC
    elif controller.buttons[4] and not controller.buttons[5] and not controller.buttons[8] and state.major != BoatState.MAJ_RC:
        state.major = BoatState.MAJ_RC
        pub.publish(state)
        
    # If PS is pushed but L1 and R1 aren't, set mode = disabled
    elif not controller.buttons[4] and not controller.buttons[5] and controller.buttons[8] and state.major != BoatState.MAJ_DISABLED:
        state.major = BoatState.MAJ_DISABLED
        pub.publish(state)
    
    rate.sleep()

def listener():
    # Setup subscribers
    rospy.init_node('controller_node', anonymous=True)
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.Subscriber('boat_state', BoatState, state_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
