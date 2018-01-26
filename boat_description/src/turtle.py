#!/usr/bin/env python

# Adapted from package teleop_tiwst_keyboard https://github.com/ros-teleop/teleop_twist_keyboard

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from boat_msgs.msg import BoatState
from std_msgs.msg import Float32
from math import pow,atan2,sqrt, cos, radians

rudder = 0.0
boat_dir = 0.0
boat_vel = 0.0
wind_dir = 0.0
wind_vel = 1.0 # m/s?
vel_msg = Twist()
pose = Pose()

vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)   

rospy.wait_for_service('turtle1/teleport_absolute')
teleport = rospy.ServiceProxy('turtle1/teleport_absolute',TeleportAbsolute) 


def calculate():
    global rudder
    global wind_dir
    global wind_vel
    global boat_dir
    global boat_vel
    
    
    print wind_dir
    boat_vel = cos(radians(wind_dir - boat_dir)) * wind_vel;


    vel_msg.linear.x = boat_vel
    vel_msg.angular.z = rudder/90 - 1
    
    vel_pub.publish(vel_msg)
    
    
def wind_callback(wind):
    global wind_dir
    wind_dir = wind.data
    calculate()
    
def rudder_callback(new_rudder):
    global rudder
    rudder = new_rudder.data
    calculate()

#def pos_callback(new_pose):
#    global pose
#    print pose
#    
#    if(pose.x > 8.5):
#        teleport(0.5, 8.5-pose.y, 0)
#    elif(pose.x < -8.5):
#        teleport(8.2, 8.5-pose.y, 0)
#        
#    if(pose.y > 8.5):
#        teleport(8.5-pose.x, 0.5, 0)
#        print "c"
#    elif(pose.y < -8.5):
#        teleport(8.5-pose.x, 8.2, 0)

    

# Initialize the node
def listener():

    rospy.init_node('turtle_controller', anonymous=True)

    rospy.Subscriber('/turtle1/pose', Pose, pos_callback)
    rospy.Subscriber('anemometer', Float32, wind_callback)
    rospy.Subscriber('rudder', Float32, rudder_callback)

    
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
