#!/usr/bin/env python

import sys
from sys import argv
import pygame
import rospy
from OpenGL.GLUT import *

from simulation_data import SimulationData
from visualization_drawer import OpenGLDrawing
from hmi_interfaces import OpenGLHMI
from ros_interfaces import ROSInterfaceManager
from simulation_calculations import SimulatorCalculator
from utils import *

from boat_msgs.msg import GPS


def send_init_gps():
	gps_pub = rospy.Publisher('gps_raw', GPS, queue_size = 10)
	gps = GPS()
	gps.latitude = 0
	gps.longitude = 0
	gps_pub.publish(gps)



def init_GLUT(drawer, hmi, display_data, calculator):

	glutInit(sys.argv)
	glutInitWindowSize(display_data.win_width, display_data.win_height)
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
	display_data.win_ID = glutCreateWindow('UW Sailbot Simulator')
	
	#Setup the window with blue background
	
	glutDisplayFunc(drawer.redraw)
	glutReshapeFunc(hmi.resize)
	glutMouseFunc(hmi.mouse_handler)
	glutPassiveMotionFunc(hmi.passive_mouse_handler)
	glutMotionFunc(hmi.motion_handler)
	glutKeyboardFunc(hmi.ASCII_handler)
	glutSpecialFunc(hmi.keyboard_handler)
	glutTimerFunc(1000/30, calculator.calc, 0)
	
	drawer.load_image_resources()
	drawer.load_font_resources()
	drawer.init_sliders()
	
	glutMainLoop()


if __name__ == '__main__':
	rospy.init_node('simulator', anonymous=True, disable_signals=True)

	
	
	pygame.mixer.init()
	pygame.mixer.music.load(rel_to_abs_filepath("../media/lemme-smash.mp3"))
	
	sim_data = SimulationData()
	sim_data.should_sim_joy = not("-j" in argv or "-J" in argv)

	rim = ROSInterfaceManager(rospy=rospy, all_data=sim_data)


	hmi = OpenGLHMI(display_data=sim_data.display_data, all_data=sim_data, ros_interfaces=rim)
	drawer = OpenGLDrawing(display_data=sim_data.display_data, all_data=sim_data, hmi=hmi)
	

	sim_calc = SimulatorCalculator(rospy,all_data=sim_data, camera=hmi.camera, ros_publishers=rim.publishers)

	init_GLUT(drawer, hmi, sim_data.display_data, sim_calc)