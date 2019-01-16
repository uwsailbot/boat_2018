#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True

import threading
import math
import pygame
import rospy
import time
from enum import Enum
from sys import argv

from boat_msgs.msg import BoatState, GPS, Point, PointArray, Waypoint, WaypointArray, Joy
from boat_msgs.srv import ConvertPoint
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int32, Bool
from rosgraph_msgs.msg import Clock
from tf.transformations import quaternion_from_euler

from OpenGL.GLUT import *
from sim_io import *
from sim_ui import *

# Cheat codes
cur_input = ""
sound = False



# UI objects and UI controls stuff

cur_slider = ()
sliders = {}
show_details = False

# set camera move speed (pixels per second)
# should camera follow boat?

#raw mouse records
mouse_pos = Point()

# Resources


# Modes
class SimMode(Enum):
	DEFAULT=0
	REPLAY=1
	CONTROLLER=2

sim_mode = SimMode.DEFAULT

# Simulation data and consts
should_sim_joy = False
sim_is_running = True

boat_speed = 0 # px/s
POS_OFFSET = rospy.get_param('/boat/nav/pos_offset')
WINCH_MIN = rospy.get_param('/boat/interfaces/winch_min')
WINCH_MAX = rospy.get_param('/boat/interfaces/winch_max')
wind_speed = 0

display_path = True

prev_path_time = 0
reset_origin_on_next_gps=False
gps_publish_interval = 0.5
gps_last_published = 0


# ROS data
wind_speed = 0





# =*=*=*=*=*=*=*=*=*=*=*=*= ROS Publishers & Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=

waypoint_pub = rospy.Publisher('waypoints_raw', WaypointArray, queue_size = 10)
wind_pub = rospy.Publisher('anemometer', Float32, queue_size = 10)
gps_pub = rospy.Publisher('gps_raw', GPS, queue_size = 10)
orientation_pub = rospy.Publisher('imu/data', Imu, queue_size = 10)
joy_pub = rospy.Publisher('joy', Joy, queue_size = 10)
square_pub = rospy.Publisher('bounding_box', PointArray, queue_size = 10)
search_area_pub = rospy.Publisher('search_area', PointArray, queue_size = 10)
vision_pub = rospy.Publisher('vision', PointArray, queue_size = 10)



def update_gps(force = False):
	global gps_last_published
	if not force and (time.time() - gps_last_published)*speed < gps_publish_interval:
		return
	gps_last_published = time.time()

	gps = GPS()
	gps.status = GPS.STATUS_FIX
	# simulate position of gps at back of boat
	pos_with_offset = Point(pos.x-POS_OFFSET*cosd(heading), pos.y-POS_OFFSET*sind(heading))
	coords = to_gps(pos_with_offset)
	gps.latitude = coords.y
	gps.longitude = coords.x
	gps.track = (450-heading)%360
	gps.speed = boat_speed * 1.94384 # m/s to KNOTS
	gps_pub.publish(gps)

	orientation = quaternion_from_euler(0,0,math.radians(heading))
	imu = Imu()
	
	# Convertion because they are different types
	imu.orientation.x = orientation[0]
	imu.orientation.y = orientation[1]
	imu.orientation.z = orientation[2]
	imu.orientation.w = orientation[3]
	
	orientation_pub.publish(imu)

# TODO update replay mode for challenges

# check if point (lps) is within fov cone
def point_is_in_fov(point):
	dx = point.x - pos.x
	dy = point.y - pos.y
	# within range
	if dx*dx + dy*dy < fov_radius*fov_radius:
		# within angle
		angle = math.degrees(math.atan2(dy, dx))
		if abs(angle-heading) < fov_angle / 2:
			return True
		if abs(angle+360-heading) < fov_angle / 2:
			return True
		if abs(angle-360-heading) < fov_angle / 2:
			return True
	return False

# publish pixel coordinates for points in vision
def update_vision():
	# TODO for obstacles as well
	global vision_points_gps
	
	vision_points_gps = PointArray()
	for waypoint in waypoint_gps.points:
		if point_is_in_fov(to_lps(waypoint)) and state.challenge is not BoatState.CHA_SEARCH:
			vision_points_gps.points.append(waypoint.pt)
	
	if search_area.target is not None and point_is_in_fov(search_area.target):
		vision_points_gps.points.append(to_gps(search_area.target))
	
	vision_pub.publish(vision_points_gps)





def search_area_callback(new_search_area):
	global gps_search_area

	gps_search_area = new_search_area
	if len(gps_search_area.points) >= 1:
		search_area.center = to_lps(gps_search_area.points[0])
		if len(gps_search_area.points) >= 2:
			edge_point = to_lps(gps_search_area.points[1])
			# first point is center and second defines radius from center
			# calc radius
			dx = (search_area.center.x - edge_point.x)
			dy = (search_area.center.y - edge_point.y)
			search_area.radius = math.sqrt(dx*dx+dy*dy)
		else:
			search_area.radius = 0
	else:
		search_area.center = None


# =*=*=*=*=*=*=*=*=*=*=*=*= GLUT callbacks =*=*=*=*=*=*=*=*=*=*=*=*=





# =*=*=*=*=*=*=*=*=*=*=*=*= OpenGL Rendering =*=*=*=*=*=*=*=*=*=*=*=*=


# =*=*=*=*=*=*=*=*=*=*=*=*= Physics =*=*=*=*=*=*=*=*=*=*=*=*=







# =*=*=*=*=*=*=*=*=*=*=*=*= Initialization =*=*=*=*=*=*=*=*=*=*=*=*=






if __name__ == '__main__':
	should_sim_joy = not("-j" in argv or "-J" in argv)
	
	pygame.mixer.init()
	pygame.mixer.music.load(rel_to_abs_filepath("../meshes/lemme-smash.mp3"))
	
	state.major = BoatState.MAJ_DISABLED
	state.minor = BoatState.MIN_COMPLETE
	state.challenge = BoatState.CHA_NAV
	
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
	
	# Publish the origin/init pos for the gps->lps conversion
	gps = GPS()
	gps.latitude = 0
	gps.longitude = 0
	gps_pub.publish(gps)
	
	init_GLUT()
