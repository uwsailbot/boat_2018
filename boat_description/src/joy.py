#!/usr/bin/env python

# Adapted from package teleop_tiwst_keyboard https://github.com/ros-teleop/teleop_twist_keyboard

import rospy
import sys, select, termios, tty
from sensor_msgs.msg import Joy
from boat_msgs.msg import BoatState
from std_msgs.msg import Float32
import time

CURSOR_UP_ONE = '\x1b[1A'
ERASE_LINE = '\x1b[2K'

msg = """
Reading from the keyboard and Publishing to Joy!
---------------------------
Moving the rudder:
a: move rudder left
d: move rudder right
Any other key recenters the rudder

1 sets RC Mode
2 sets Auto Mode

t: tack
g: cancel tack

o: shift wind left 5 degrees
p: shift wind right 5 degrees

Change rudder limits:
w: increase + 5 degrees
x: decrease - 5 degrees
By default rudder actuates at 45 degrees, limited to 60 max

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

windBinding={
		'p':5,
		'o':-5
		  }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def writeLine(rudder, tacking, angle, auto, wind):
	sys.stdout.write(CURSOR_UP_ONE)
	sys.stdout.write(ERASE_LINE)
	sys.stdout.write(CURSOR_UP_ONE)
	sys.stdout.write(ERASE_LINE)
	print "Limits: %s, Tacking: %s, AutoMode: %s" % (angle, tacking, auto)
	print "Rudder: %s, Wind: %s" % (rudder, wind)

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	joy_pub = rospy.Publisher('joy', Joy, queue_size = 1)
	wind_pub = rospy.Publisher('anemometer', Float32, queue_size = 1)
	rudder_pub = rospy.Publisher('rudder', Float32, queue_size = 1)
	rospy.init_node('keyboard_node')
	rate = rospy.Rate(100)
	
	status = 0
	rudder_pos = 90
	auto = 0
	tack_prev_request = 0
	tack = 0
	cancel = 0
	cancel_prev_request = 0
	actuating_angle = 45
	anemometer = 0

	try:
		print msg
		while(1):
			key = getKey()

			# One of the rudder pos keys is hit, update rudder position
			if key in rudderBinding.keys():
				rudder_pos = 90 + rudderBinding[key]*actuating_angle
				writeLine(rudder_pos, tack, actuating_angle, auto, anemometer)

			# One of the state setting keys hit, update state
			elif key in autoBinding.keys():
				auto = autoBinding[key]
				writeLine(rudder_pos, tack, actuating_angle, auto, anemometer)

			# One of the tacking keys hit, update tacking state
			elif key in tackBinding.keys():
				if tackBinding[key]:
					tack = True
					tack_prev_request = True
					cancel = False
				else:
					tack = False
					cancel = True
					cancel_prev_request = True
				rudder_pos = 90
				writeLine(rudder_pos, tack, actuating_angle, auto, anemometer)

			# One of the angle keys hit, update max angle
			elif key in angleBinding.keys():
				actuating_angle = actuating_angle + angleBinding[key]
				if actuating_angle < 5:
					actuating_angle = 5
				elif actuating_angle > 60:
					actuating_angle = 60
				rudder_pos = 90
				
				if (status == 14):
					print msg
				status = (status + 1) % 15
				writeLine(rudder_pos, tack, actuating_angle, auto, anemometer)

			# One of the wind direction keys was hit
			elif key in windBinding.keys():
				anemometer = anemometer + windBinding[key]
				if anemometer >= 360:
					anemometer = anemometer - 360
				elif anemometer < 0:
					anemometer = anemometer + 360
				writeLine(rudder_pos, tack, actuating_angle, auto, anemometer)

			else:
				rudder_pos = 90
				writeLine(rudder_pos, tack, actuating_angle, auto, anemometer)
				# Control C quit
				if (key == '\x03'):
					break
				
			# Publish new joy message to mimic requested values
			joy = Joy()
			joy_axes = [((90 - rudder_pos)/60.0), 0,0,0,0,0,0,0]
			joy_buttons = [tack,0,cancel,0,(not auto),auto,0,0,0,0,0]
			joy.axes = joy_axes
			joy.buttons = joy_buttons
			joy_pub.publish(joy)

			# Publish new anemometer message to relay requested wind data
			wind = Float32()
			wind.data = anemometer
			wind_pub.publish(wind)
			
			# Publish rudder pos
			rudder = Float32()
			rudder.data = rudder_pos
			rudder_pub.publish(rudder)

			# Reset state buttons
			if tack_prev_request:
				tack = False
			elif cancel_prev_request:
				cancel = False

			rate.sleep()

	except rospy.ROSInterruptException:
		pass
	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

