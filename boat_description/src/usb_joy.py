#!/usr/bin/env python
import rospy
import pygame
from sensor_msgs.msg import Joy
import sys
from struct import *
import time
import serial

def start():
	# Setup publisher for the /joy topic
	pub = rospy.Publisher('joy', Joy, queue_size=10)
	rospy.init_node('usb_joy', anonymous=True)
	rate = rospy.Rate(100)
	
	pygame.init()
	pygame.joystick.init()
	
	if pygame.joystick.get_count() < 1:
		print "No joysticks found"
		exit(0)
		
	
	stick = pygame.joystick.Joystick(0)
	stick.init()
	print stick.get_numaxes()
	
	while not rospy.is_shutdown() and pygame.joystick.get_count() > 0:
		pygame.event.pump()
		
		
		# Axes
		# 0: LX (+Right)
		# 1: LY (+Down)
		# 2: LT
		# 3: RX (+Right)
		# 4: RY (+Down)
		# 5: RT
		
		# Buttons
		# 0: X
		# 1: O
		# 2: Triangle
		# 3: Square
		# 4: LB
		# 5: RB
		# 6:
		# 7: 
		# 8: Share
		# 9: Options
		# 10: PlayStation
		# 11: L3
		# 12: R3
		
		# POV
		# 0[0]: X (+Right)
		# 0[1]: Y (+Up)
		
		axes = []
		axes.append(stick.get_axis(0))
		axes.append(-stick.get_axis(1))
		axes.append((stick.get_axis(2)+1)/2)
		axes.append(stick.get_axis(3))
		axes.append(-stick.get_axis(4))
		axes.append((stick.get_axis(5)+1)/2)
		axes.append(stick.get_hat(0)[0])
		axes.append(stick.get_hat(0)[1])
		
		buttons = []
		buttons.append(stick.get_button(0))
		buttons.append(stick.get_button(3))
		buttons.append(stick.get_button(1))
		buttons.append(stick.get_button(2))
		buttons.append(stick.get_button(4))
		buttons.append(stick.get_button(5))
		buttons.append(stick.get_button(8))
		buttons.append(stick.get_button(9))
		buttons.append(stick.get_button(10))
		buttons.append(stick.get_button(11))
		buttons.append(stick.get_button(12))
		
		# Setup a JOY message with the parsed data and publish it
		joy = Joy()
		joy.header.stamp = rospy.get_rostime()
		joy.axes = axes
		joy.buttons = buttons
		pub.publish(joy)
		rate.sleep()
	
	stick.quit()


if __name__ == '__main__':
	try: 
		start()
	except rospy.ROSInterruptException:
		pass
