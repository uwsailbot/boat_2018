#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Float32
from boat_utilities import angles

pid_is_enabled = False
kp = 1.0
kd = 0.0
ki = 0.0
output_pub = rospy.Publisher('output', Float32, queue_size=10)
setpoint = 0

# TODO: Modify this to make the PID node generalizable and usable for multiple controllers
max_range = (rospy.get_param('/boat/interfaces/rudder_max') -
             rospy.get_param('/boat/interfaces/rudder_min')) / 2.0


# If the boat state topic changes, update local boat state
def input_callback(input):

    if pid_is_enabled:
        e = angles.normalize_signed(setpoint - input.data)
        prop = kp * e
        # TODO: Add integral and derivative
        integral = 0
        der = 0
        output = Float32()
        output.data = integral + prop + der
        if output.data > 60:
            output.data = 60
        elif output.data < -60:
            output.data = -60
        output_pub.publish(output)


# If the wind heading topic changes, update local wind heading
def setpoint_callback(setpt):
    global setpoint
    setpoint = setpt.data


def enable_callback(is_enabled):
    global pid_is_enabled
    pid_is_enabled = is_enabled.data


def listener():
    # Setup subscribers
    rospy.init_node('pid_node')
    rospy.Subscriber('input', Float32, input_callback)
    rospy.Subscriber('setpoint', Float32, setpoint_callback)
    rospy.Subscriber('enable', Bool, enable_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
