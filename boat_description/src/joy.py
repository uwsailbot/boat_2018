#!/usr/bin/env python

# Adapted from package teleop_tiwst_keyboard https://github.com/ros-teleop/teleop_twist_keyboard

import rospy
import sys, select, termios, tty
from sensor_msgs.msg import Joy
from boat_msgs.msg import BoatState
import time

msg = """
Reading from the keyboard  and Publishing to Joy!
---------------------------
Moving the rudder:
a: move rudder left
d: move rudder right
Any other key recenters the rudder

1 sets RC Mode
2 sets Auto Mode

t: tack
g: cancel tack

To change the angle at which the rudder moves,
Update the movement factor:
w: increase + 5 degrees
x: decrease - 5 degrees

Default: rudder actuates at 45 degrees, limited to 60 max


CTRL-C to quit
"""

rudderBinding = {
		'a':-1,
		'd':1
		}
autoBinding = {
		'1':0,
		'2':1
		 }

angleBinding={
		'w':5,
		'x':-5
	      }

tackBinding={
		't':1,
		'g':0
	      }


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	pub = rospy.Publisher('joy', Joy, queue_size = 1)
	rospy.init_node('keyboard_node')
	rate = rospy.Rate(100)
	status = 0
	rudder_pos = 90
	auto = 0
	tack = 0
	actuating_angle = 45

	try:
		print msg
		while(1):
			key = getKey()

			# One of the rudder pos keys is hit, update rudder position
			if key in rudderBinding.keys():
				rudder_pos = 90 + rudderBinding[key]*actuating_angle
				if rudder_pos < 30:
					rudder_pos = 30
				elif rudder_pos > 150:
					rudder_pos = 150

			# One of the state setting keys hit, update state
			elif key in autoBinding.keys():
				auto = autoBinding[key]

			# One of the tacking keys hit, update tacking state
			elif key in tackBinding.keys():
				tack = tackBinding[key]

			# One of the angle keys hit, update max angle
			elif key in angleBinding.keys():
				actuating_angle = actuating_angle + angleBinding[key]
				rudder_pos = 90
				print "Actuating Angle: %s, Rudder Reset" % (actuating_angle)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				rudder_pos = 90
				# Control C quit
				if (key == '\x03'):
					break
				
			# Create new joy message to mimc requested rudder values
			joy = Joy()
			joy_axes = [((90 - rudder_pos)/60.0), 0,0,0,0,0,0,0]
			joy_buttons = [tack,0,(not tack),0,(not auto),auto,0,0,0,0,0]
			joy.axes = joy_axes
			joy.buttons = joy_buttons
			pub.publish(joy)
			rate.sleep()

	except rospy.ROSInterruptException:
		pass
	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

