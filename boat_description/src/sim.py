#!/usr/bin/env python
import sys
sys.dont_write_bytecode = True

import math
import pygame
import rospy
import time
from enum import Enum
from boat_msgs.msg import BoatState, GPS, Point, PointArray, Waypoint, WaypointArray
from boat_msgs.srv import ConvertPoint
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float32, Int32, Bool
from rosgraph_msgs.msg import Clock
from tf.transformations import quaternion_from_euler
from nav_mgs.msg import Odometry
#from OpenGL.GL import *
from OpenGL.GLUT import *
from sys import argv
from sim_io import *
from sim_ui import *

# Cheat codes
cur_input = ""
sound = False

# OpenGL data
win_ID = 0
win_width = 720
win_height = 480

# UI objects and UI controls stuff
camera = Camera(0,0,10, win_width, win_height)
cur_slider = ()
sliders = {}
show_details = False
gridsize = 10
# set camera move speed (pixels per second)
camera_move_speed = 100
camera_velocity = Point(0, 0)
# should camera follow boat?
follow_boat = False


# Resources
compass_img = ()
compass_pointer_img = ()
boat_imgs = {}
rudder_imgs = {}
sail_imgs = {}
cur_boat_img = ()
cur_rudder_img = ()
cur_sail_img = ()
open_sans_font = ()

# Modes
class SimMode(Enum):
	DEFAULT=0
	REPLAY=1
	CONTROLLER=2

sim_mode = SimMode.DEFAULT

# Simulation data and consts
should_sim_joy = False
sim_is_running = True
speed = 10
pre_pause_speed = 10
pause = False
clock = 0
last_time = -1
boat_speed = 0 # px/s
layline = rospy.get_param('/boat/nav/layline')
winch_min = rospy.get_param('/boat/interfaces/winch_min')
winch_max = rospy.get_param('/boat/interfaces/winch_max')
wind_speed = 0
speed_graph = {0 : 0}
display_path = True
path = PointArray()
prev_path_time = 0
fov_radius = 15
fov_angle = 60
vision_points_gps = PointArray()

# ROS data
wind_heading = 270
wind_speed = 0
ane_reading = 0
pos = Point(0,0)
heading = 270
target_heading = 270
state = BoatState()
rudder_pos = 90
winch_pos = 2000
waypoint_gps = WaypointArray()
joy = Joy()
joy.axes = [0]*8
joy.buttons = [0]*11
rudder_output = 0
rudder_input = 0
rudder_setpoint = 0
rudder_enable = False
replay_gps_raw = GPS()
obstacle_points = PointArray()
target_point = Waypoint()
gps_bounding_box = PointArray()


# =*=*=*=*=*=*=*=*=*=*=*=*= ROS Publishers & Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=

waypoint_pub = rospy.Publisher('waypoints_raw', WaypointArray, queue_size = 10)
wind_pub = rospy.Publisher('anemometer', Float32, queue_size = 10)
gps_pub = rospy.Publisher('gps_raw', GPS, queue_size = 10)
orientation_pub = rospy.Publisher('imu/data', Imu, queue_size = 10)
joy_pub = rospy.Publisher('joy', Joy, queue_size = 10)
square_pub = rospy.Publisher('bounding_box', PointArray, queue_size = 10)
clock_pub = rospy.Publisher('clock', Clock, queue_size = 1)
vision_pub = rospy.Publisher('vision', PointArray, queue_size = 10)
to_gps = rospy.ServiceProxy('lps_to_gps', ConvertPoint)
to_lps = rospy.ServiceProxy('gps_to_lps', ConvertPoint)


def update_gps():
	gps = GPS()
	gps.status = GPS.STATUS_FIX
	coords = to_gps(pos)
	gps.latitude = coords.pt.y
	gps.longitude = coords.pt.x
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
		if angle < 0:
			angle += 360
		if abs(angle-heading) < fov_angle / 2:
			return True
	return False

# publish pixel coordinates for points in vision
def update_vision():
	# TODO for obstacles as well
	global vision_points_gps
	
	vision_points_gps = PointArray()
	for waypoint in waypoint_gps.points:
		lps = to_lps(waypoint.pt).pt
		if point_is_in_fov(lps):
			vision_points_gps.points.append(waypoint.pt)
	
	vision_pub.publish(vision_points_gps)

def update_wind():
	global wind_heading
	global heading
	global ane_reading
	global boat_speed
	
	apparent_wind = calc_direction(calc_apparent_wind(wind_heading, boat_speed, heading))
	ane_reading = (apparent_wind - heading) % 360
	if ane_reading < 0:
		ane_reading = ane_reading + 360
	
	wind_pub.publish(Float32(ane_reading))


def boat_state_callback(newState):
	global state
	state = newState
	if state.major is not BoatState.MAJ_DISABLED and pause:
		pause_sim()
	
def rudder_callback(pos):
	global rudder_pos
	rudder_pos = pos.data


def winch_callback(pos):
	global winch_pos
	winch_pos = pos.data


# Update the raw waypoints
def waypoints_callback(newPoints):
	global waypoint_gps
	
	# Whenever the number of waypoints is decremented by one and we are in maximum meme state, smash becky
	if (len(waypoint_gps.points)-1) is len(newPoints.points) and sound and cur_boat_img is boat_imgs["mars"]:
		pygame.mixer.music.play()
	
	waypoint_gps = newPoints


def target_heading_callback(angle):
	global target_heading
	target_heading = angle.data

def lps_callback(lps):
	global pos
	
	if sim_mode is SimMode.REPLAY or sim_mode is SimMode.CONTROLLER:
		pos = lps

def odom_callback(odom):
	global pos
	
	if sim_mode is SimMode.REPLAY or sim_mode is SimMode.CONTROLLER:
		pos = Point(odom.pose.pose.position.x, odom.pose.pose.position.y)

def compass_callback(compass):
	global heading
	
	if sim_mode is SimMode.REPLAY or sim_mode is SimMode.CONTROLLER:
		heading = compass.data

def anemometer_callback(anemometer):
	global ane_reading
	ane_reading = anemometer.data

def mock_true_wind_callback(anemometer):
	global wind_heading
	wind_heading = anemometer.data

def rudder_output_callback(float32):
	global rudder_output
	rudder_output = float32.data

def rudder_input_callback(float32):
	global rudder_input
	rudder_input = float32.data

def rudder_setpoint_callback(float32):
	global rudder_setpoint
	rudder_setpoint = float32.data

def rudder_enable_callback(enabled):
	global rudder_enable
	rudder_enable = enabled.data

def gps_raw_callback(gps_raw):
	global replay_gps_raw
	replay_gps_raw = gps_raw
	
def obstacles_callback(obstacles):
	global obstacle_points
	obstacle_points = obstacles

def bounding_box_callback(box):
	global gps_bounding_box
	gps_bounding_box = box

def target_point_callback(target_pt):
	global target_point
	target_point = target_pt

def vision_callback(new_vision_points_gps):
	global vision_points_gps
	vision_points_gps = new_vision_points_gps

# =*=*=*=*=*=*=*=*=*=*=*=*= GLUT callbacks =*=*=*=*=*=*=*=*=*=*=*=*=

# Window resize callback
def resize(width, height):
	global win_width
	global win_height
	win_width = width
	win_height = height
	glutInitWindowSize(win_width, win_height)
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	glOrtho(0.0, win_width, 0.0, win_height, -1, 1)
	glMatrixMode(GL_MODELVIEW)
	glLoadIdentity()
	camera.resize(win_width, win_height)


# Handler for mouse presses
def mouse_handler(button, mouse_state, x, y):
	global waypoint_gps
	global sliders
	global cur_slider
	global gps_bounding_box
	
	if mouse_state != GLUT_DOWN:
		cur_slider = ()
		return

	# If status panels are being clicked on
	if (x <= 180 and show_details) or (x >= win_width - 120) and button == GLUT_LEFT_BUTTON:
		for key in sliders:
			if sliders[key].contains(x,y, win_height):
				cur_slider = sliders[key]
				cur_slider.handle_mouse(x,y)
		return

	if button == GLUT_RIGHT_BUTTON:
		if sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER:
			waypoint_gps = WaypointArray()
			gps_bounding_box = PointArray()
			waypoint_pub.publish(waypoint_gps)
			square_pub.publish(gps_bounding_box)

	elif cur_slider is () and (sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER) and state.challenge is not BoatState.CHA_STATION and (button == GLUT_LEFT_BUTTON or button == GLUT_MIDDLE_BUTTON):
		newPt = Point()
		(lps_x,lps_y) = camera.screen_to_lps(x,y)
		newPt.x = lps_x
		newPt.y = lps_y
		if button == GLUT_LEFT_BUTTON:
			coords = Waypoint(to_gps(newPt).pt, Waypoint.TYPE_INTERSECT)
		elif button == GLUT_MIDDLE_BUTTON:
			coords = Waypoint(to_gps(newPt).pt, Waypoint.TYPE_ROUND)
		waypoint_gps.points.append(coords)
		
		waypoint_pub.publish(waypoint_gps)

	elif cur_slider is () and (sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER) and state.challenge is BoatState.CHA_STATION and button == GLUT_LEFT_BUTTON:
		newPt = Point()
		(lps_x,lps_y) = camera.screen_to_lps(x,y)
		newPt.x = lps_x
		newPt.y = lps_y
		coords = to_gps(newPt).pt
		
		for p in gps_bounding_box.points:
			l = to_lps(p).pt
			if math.hypot(l.y - newPt.y, l.x-newPt.x) < 15:
				print "Distance between buoys is too small, must be at least 15m"
				return
		# Reset if we were gonna add to a list of 4 points already
		if len(gps_bounding_box.points) == 4:
			gps_bounding_box = PointArray()
		
		gps_bounding_box.points.append(coords)
		square_pub.publish(gps_bounding_box)
		
		waypoint_pub.publish(waypoint_gps)

	elif (button == 3 or button == 4) and mouse_state == GLUT_DOWN:
		
		if not follow_boat:
			# used to keep mouse pointing at same lps position when zooming
			(mouse_x, mouse_y) = camera.screen_to_lps(x,y)
		
		if button == 3:
			camera.scale *= 1.04
			if camera.scale > 100:
				camera.scale = 100
		else:
			camera.scale *= 0.96
			if camera.scale < 0.1:
				camera.scale = 0.1
		
		if not follow_boat:
			# keep mouse pointing at same lps position when zooming
			(new_mouse_x, new_mouse_y) = camera.screen_to_lps(x,y)
			camera.x -= new_mouse_x - mouse_x
			camera.y -= new_mouse_y - mouse_y

# Handler for mouse position
def passive_mouse_handler(x,y):
	global camera_velocity
	
	# if mouse is on right info panel, don't move camera
	if x > win_width-120:
		camera_velocity.x = 0
		camera_velocity.y = 0
		return
	
	# set camera_velocity when mouse is near edge of screen or edge of info display panel
	if (x < 55 and x > 5 and not show_details) or (x < 180+50 and x > 180 and show_details):
		camera_velocity.x = -camera_move_speed / camera.scale
	elif x > win_width-120-50:
		camera_velocity.x = camera_move_speed / camera.scale
	else:
		camera_velocity.x = 0
	# The > 5 makes sure that when we scroll off the window, the camera stops moving
	if y < 55 and y > 5:
		camera_velocity.y = camera_move_speed / camera.scale
	elif y > win_height-55 and y < win_height-5:
		camera_velocity.y = -camera_move_speed / camera.scale
	else:
		camera_velocity.y = 0	

# Handler for mouse dragging
def motion_handler(x,y):
	if cur_slider is not ():
		cur_slider.handle_mouse(x,y)


# Handler for all key presses that cannot be represented by an ASCII code
def keyboard_handler(key, mousex, mousey):
	global joy
	if key == GLUT_KEY_LEFT and should_sim_joy:
		joy.axes[0] = max(joy.axes[0]-0.1, -1)
		joy_pub.publish(joy)
	elif key == GLUT_KEY_RIGHT and should_sim_joy:
		joy.axes[0] = min(joy.axes[0]+0.1, 1)
		joy_pub.publish(joy)
	elif key == GLUT_KEY_UP and should_sim_joy:
		joy.axes[4] = 1
		joy_pub.publish(joy)
		joy.axes[4] = 0
		joy_pub.publish(joy)
	elif key == GLUT_KEY_DOWN and should_sim_joy:
		joy.axes[4] = -1
		joy_pub.publish(joy)
		joy.axes[4] = 0
		joy_pub.publish(joy)


# Handler for all key presses that can be represented by an ASCII code
def ASCII_handler(key, mousex, mousey):
	global speed
	global joy
	global sim_is_running
	global cur_input
	global cur_boat_img
	global cur_rudder_img
	global cur_sail_img
	global sound
	global wind_heading
	global sim_mode
	global show_details
	global display_path
	global path
	global follow_boat
	
	# Handle cheat codes
	cur_input += key;
	valid = False
	for code in boat_imgs:
		if code == cur_input:
			cur_boat_img = boat_imgs[code]
			cur_rudder_img = rudder_imgs[code]
			cur_sail_img = sail_imgs[code]
			valid = False
			break
		if code.startswith(cur_input):
			valid = True
	if not valid:
		cur_input = ""
	
	if key is chr(27):
		sim_is_running = False
	elif key is 'p':
		pause_sim()
	elif key is 'm':
		if sim_mode is SimMode.DEFAULT:
			sim_mode = SimMode.REPLAY
		elif sim_mode is SimMode.REPLAY:
			sim_mode = SimMode.CONTROLLER
		else:
			sim_mode = SimMode.DEFAULT
		print 'Changed sim mode, is now', sim_mode
	elif key is 'i':
		show_details = not show_details
	elif key is 'c':
		path = PointArray()
		display_path = not display_path
	elif key is 'y':
		follow_boat = not follow_boat
	elif key is '0':
		sound = not sound
	
	elif key is 'x' and (sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER):
		(lps_x,lps_y) = camera.screen_to_lps(mousex,mousey)
		coords = Waypoint(to_gps(Point(lps_x, lps_y)).pt, Waypoint.TYPE_ROUND)
		waypoint_gps.points.append(coords)
		waypoint_pub.publish(waypoint_gps)
	
	if sim_mode is SimMode.DEFAULT or sim_mode is SimMode.CONTROLLER:
		if key is '1' and should_sim_joy:
			joy.buttons[4] = 1
			joy.buttons[5] = 0
			joy.buttons[8] = 0
			joy_pub.publish(joy)
		elif key is '2' and should_sim_joy:
			joy.buttons[4] = 0
			joy.buttons[5] = 1
			joy.buttons[8] = 0
			joy_pub.publish(joy)
		elif key is '3' and should_sim_joy:
			joy.buttons[4] = 0
			joy.buttons[5] = 0
			joy.buttons[8] = 1
			joy_pub.publish(joy)
		elif key is '4' and should_sim_joy:
			joy.buttons[1] = 1
			joy.buttons[3] = 0
			joy_pub.publish(joy)
			joy.buttons[1] = 0
			joy_pub.publish(joy)
		elif key is '5' and should_sim_joy:
			joy.buttons[1] = 0
			joy.buttons[3] = 1
			joy_pub.publish(joy)
			joy.buttons[3] = 0
			joy_pub.publish(joy)
		elif key is 't' and should_sim_joy:
			joy.buttons[0] = 1
			joy.buttons[2] = 0
			joy.axes[0] = 0
			joy_pub.publish(joy)
			joy.buttons[0] = 0
			joy_pub.publish(joy)
		elif key is 'g' and should_sim_joy:
			joy.buttons[2] = 1
			joy.buttons[0] = 0
			joy_pub.publish(joy)
			joy.buttons[2] = 0
			joy_pub.publish(joy)
		elif key is 'q':
			joy.axes[0] = 0
			joy_pub.publish(joy)
		elif key is 'a':
			wind_heading += 5
			wind_heading = wind_heading % 360
		elif key is 'd':
			wind_heading -= 5
			if wind_heading < 0:
				wind_heading += 360
		elif key is ' ':
			pos.x = 0
			pos.y = 0
			camera.x = 0
			camera.y = 0
			camera.scale = 10
			path = PointArray()
			update_gps()
			


# Main display rendering callback
def redraw():
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
	
	glViewport(0, 0, win_width, win_height)
	
	# Render stuff
	draw_grid()
	if state.challenge is BoatState.CHA_STATION:
		draw_bounding_box()	
	if display_path:
		draw_path()
	draw_fov()
	draw_target_point()
	draw_waypoints()
	draw_waypoints_in_fov()
	draw_obstacles()
	draw_boat()
	draw_target_heading_arrow()
	draw_status()
	if show_details:
		draw_detailed_status()
	
	glutSwapBuffers()


# =*=*=*=*=*=*=*=*=*=*=*=*= OpenGL Rendering =*=*=*=*=*=*=*=*=*=*=*=*=

# Draw all of the waypoint as red dots
def draw_waypoints():
	glPushMatrix()
	
	for gps in waypoint_gps.points:
		
		if gps.type is Waypoint.TYPE_ROUND:
			glColor3f(1,0.5,0)
		else:
			glColor3f(1,0,0)
		
		
		p = to_lps(gps.pt).pt
		(x,y) = camera.lps_to_screen(p.x, p.y)
		draw_circle(0.5 * camera.scale,x,y)
	
	glPopMatrix()

def draw_bounding_box():
	glPushMatrix()

	glColor3f(0,1,0)
	for p in gps_bounding_box.points:
		l = to_lps(p).pt
		(x,y) = camera.lps_to_screen(l.x, l.y)
		draw_circle(0.5 * camera.scale,x,y)

	if len(gps_bounding_box.points) == 4:
		glLineWidth(1.0)
		glBegin(GL_LINES)
		for i in range(0, 4):
			a = to_lps(gps_bounding_box.points[i]).pt
			b = to_lps(gps_bounding_box.points[(i+1)%4]).pt
			(x1, y1) = camera.lps_to_screen(a.x, a.y)
			(x2, y2) = camera.lps_to_screen(b.x, b.y)
			glVertex2f(x1, y1)
			glVertex2f(x2, y2)
		glEnd()
		
	glPopMatrix()
	

def draw_target_point():
	if target_point is not () and state.major is BoatState.MAJ_AUTONOMOUS and state.minor is not BoatState.MIN_COMPLETE:
		glColor3f(1,1,1)
		(x,y) = camera.lps_to_screen(target_point.pt.x, target_point.pt.y)
		draw_circle(0.7 * camera.scale, x, y)

def draw_obstacles():
	glPushMatrix()
	
	glColor3f(0.2, 0.2, 0.2)
	for p in obstacle_points.points:
		(x,y) = camera.lps_to_screen(p.x, p.y)
		draw_circle(0.5 * camera.scale,x,y)
	
	glPopMatrix()


# Draw boat's target heading as an arrow centered at (x, y) pointing in the target heading's dirction
def draw_target_heading_arrow():
	if state.major is not BoatState.MAJ_AUTONOMOUS:
		return
	glPushMatrix()
	
	(boat_x, boat_y) = camera.lps_to_screen(pos.x, pos.y)
	
	glTranslatef(boat_x, boat_y, 0)
	glRotatef(target_heading, 0, 0, 1)
	
	tip_radius = 30
	arrow_height = 10
	arrow_base = 5
	
	glColor3f(255, 255, 255)
	glBegin(GL_POLYGON)
	glVertex2f(tip_radius, 0)
	glVertex2f(tip_radius - arrow_base, arrow_base/2)
	glVertex2f(tip_radius - arrow_base, -arrow_base/2)
	glEnd()
	
	glPopMatrix()

def draw_fov():
	(boat_x, boat_y) = camera.lps_to_screen(pos.x, pos.y)
	
	# calculate points for drawing fov cone (facing up)
	resolution = 5
	angle_step = float(fov_angle) / resolution
	cone_points = PointArray()
	cone_points.points.append(Point(0, 0))
	for i in range(-resolution, resolution + 1):
		if i is not 0:
			angle = i * angle_step
			x = math.sin(math.radians(angle/2)) * fov_radius * camera.scale
			y = math.cos(math.radians(angle/2)) * fov_radius * camera.scale
			cone_points.points.append(Point(x,y))
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
	glEnable(GL_BLEND)
	glColor4f(245/255.0, 150/255.0, 25/255.0, 0.1)
	
	glPushMatrix()
	glTranslatef(boat_x, boat_y, 0)
	glRotatef(heading-90, 0, 0, 1)

 	glBegin(GL_POLYGON)
	for point in cone_points.points:
		glVertex2f(point.x,point.y)
	glEnd()

	glColor4f(245/255.0, 150/255.0, 25/255.0, 0.8)
	glBegin(GL_LINE_LOOP)
	for point in cone_points.points:
		glVertex2f(point.x,point.y)
	glEnd()
	glPopMatrix()

	glDisable(GL_BLEND)


def draw_waypoints_in_fov():
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
	glEnable(GL_BLEND)
	glColor4f(245/255.0, 200/255.0, 5/255.0, 0.3)
	#print(vision_points_lps.points)
	for point in vision_points_gps.points:
		point = to_lps(point).pt
		(x,y) = camera.lps_to_screen(point.x, point.y)
		draw_circle(0.8 * camera.scale, x, y)

	glDisable(GL_BLEND)


# Draw the right-hand 'status' panel and all of its data
def draw_status():
	glPushMatrix()
	
	# Control positions of stuff
	pos_offset = win_height*0.62
	state_offset = win_height*0.47
	boat_offset = win_height*0.25
	
	# Draw the box
	panel_width = 120
	glColor3f(1.0, 1.0, 1.0)
	glBegin(GL_QUADS)
	glVertex2f(win_width,win_height)
	glVertex2f(win_width, 0)
	glVertex2f(win_width-panel_width,0)
	glVertex2f(win_width-panel_width,win_height)
	glEnd()
	
	# Set font
	set_font(open_sans_font)
	
	# Draw the wind readout
	glColor3f(0.0, 0.0, 0.0)
	draw_text("Wind: " + str(wind_heading), win_width-60, win_height-20, 'center')
	draw_wind_arrow(win_width-60,win_height-65)
	
	# Draw wind speed text
	draw_text("Wind speed ", win_width-60, win_height-125, 'center')
	sliders["Wind speed"].resize(win_width-100, win_height-160)
	sliders["Wind speed"].draw()
	
	# Draw the boat pos
	glColor3f(0.0, 0.0, 0.0)
	draw_text("X: %.1f m" % pos.x, win_width-60, pos_offset, 'center')
	draw_text("Y: %.1f m" % pos.y, win_width-60, pos_offset-15, 'center')
	draw_text("Spd: %.1f m/s" % boat_speed, win_width-60, pos_offset-30, 'center')
	draw_text("Head: %.1f" % heading, win_width-60, pos_offset-45, 'center')
	
	# Draw the boat state
	major = ""
	if state.major is BoatState.MAJ_RC:
		major = "RC"
	elif state.major is BoatState.MAJ_AUTONOMOUS:
		major = "Auto"
	elif state.major is BoatState.MAJ_DISABLED:
		major = "Disabled"

	challenge = ""
	if state.challenge is BoatState.CHA_STATION:
		challenge = "Station"
	elif state.challenge is BoatState.CHA_NAV:
		challenge = "Navigation"
	elif state.challenge is BoatState.CHA_LONG:
		challenge = "Long Distance"
	elif state.challenge is BoatState.CHA_AVOID:
		challenge = "Avoidance"
	elif state.challenge is BoatState.CHA_SEARCH:
		challenge = "Search"
	
	minor = ""
	if state.minor is BoatState.MIN_COMPLETE:
		minor = "Complete"
	elif state.minor is BoatState.MIN_PLANNING:
		minor = "Planning"
	elif state.minor is BoatState.MIN_TACKING:
		minor = "Tacking"

	draw_text("State", win_width-60, state_offset, 'center', 24)
	draw_text("M: " + major, win_width-60, state_offset-20, 'center')
	draw_text("m: " + minor, win_width-60, state_offset-35, 'center')
	draw_text("C: " + challenge, win_width-60, state_offset-50, 'center')
	
	# Draw the boat diagram
	draw_status_boat(win_width-60, boat_offset)
	
	# Draw the rudder and winch pos
	draw_text("Winch: %d" % winch_pos, win_width-60, boat_offset-35, 'center')
	draw_text("Rudder: %.1f" % rudder_pos, win_width-60, boat_offset-50, 'center')
	
	# Draw the simulation speed
	draw_text("Sim speed ", win_width-60, 50, 'center')
	sliders["Sim speed"].resize(win_width-100, 20)
	sliders["Sim speed"].draw()
	draw_text("Time: %.1f" % clock + "s", win_width-60, 5, 'center')
	
	#draw_speed_graph(75, 75, 100)
	
	glPopMatrix()

# Draw another panel to extend the status panel with more details and readings
def draw_detailed_status():
	global ane_reading
	global rudder_output
	global rudder_input
	global rudder_setpoint
	global rudder_enable
	global replay_gps_raw

	glPushMatrix()
	
	# width		
	panel_width = 180
		
	# Control position of stuff
	rudder_offset = 0.8
	gps_offset = 0.5
	
	# Draw the box
	glColor3f(1.0, 1.0, 1.0)
	glBegin(GL_QUADS)
	glVertex2f(0, win_height)
	glVertex2f(0, 0)
	glVertex2f(panel_width, 0)
	glVertex2f(panel_width, win_height)
	glEnd()
	
	# Set font
	set_font(open_sans_font)
	
	# Draw anemometer reading
	draw_text("Anemometer: %.1f" % ane_reading, panel_width/2, win_height-30, 'center')
	
	# Draw rudder pid values
	draw_text("Rudder PID:", panel_width/2, win_height*rudder_offset, 'center', 18)
	draw_text("output: %.1f" % rudder_output, panel_width/2, win_height*rudder_offset-20, 'center')
	draw_text("input: %.1f" % rudder_input, panel_width/2, win_height*rudder_offset-35, 'center')
	draw_text("setpoint: %.1f" % rudder_setpoint, panel_width/2, win_height*rudder_offset-50, 'center')
	draw_text("enable: %s" % rudder_enable, panel_width/2, win_height*rudder_offset-65, 'center')
	
	# Draw gps_raw values:
	draw_text("GPS Raw:", panel_width/2, win_height*gps_offset, 'center', 18)
	draw_text("status: %f" % replay_gps_raw.status, panel_width/2, win_height*gps_offset-20, 'center')
	draw_text("satellites_used: %.1f" % replay_gps_raw.satellites_used, panel_width/2, win_height*gps_offset-35, 'center')
	draw_text("lat: %.0000001f" % replay_gps_raw.latitude, panel_width/2, win_height*gps_offset-50, 'center')
	draw_text("long: %.0000001f" % replay_gps_raw.longitude, panel_width/2, win_height*gps_offset-65, 'center')
	draw_text("alt: %.0000001f" % replay_gps_raw.altitude, panel_width/2, win_height*gps_offset-80, 'center')
	draw_text("track: %.0000001f" % replay_gps_raw.track, panel_width/2, win_height*gps_offset-95, 'center')
	draw_text("speed: %.001f" % replay_gps_raw.speed, panel_width/2, win_height*gps_offset-110, 'center')
	draw_text("hdop: %.01f" % replay_gps_raw.hdop, panel_width/2, win_height*gps_offset-125, 'center')
	
	# Draw mode
	draw_text("Sim Mode:", panel_width/2, 40, 'center', 18)
	draw_text(str(sim_mode), panel_width/2, 20, 'center') 
	glPopMatrix()


# Draw the arrow for the wind direction centered on (x, y)
def draw_wind_arrow(x,y):
	draw_image(compass_img, (x, y), 0, (70,70))
	draw_image(compass_pointer_img, (x, y), wind_heading-90, (7,40))

# Draw grid
def draw_grid():
	screen_gridsize = int(gridsize * camera.scale)
	if screen_gridsize == 0:
		screen_gridsize = 1
	x = int(win_width/2.0 - camera.x * camera.scale) % screen_gridsize
	y = int(win_height/2.0 - camera.y * camera.scale) % screen_gridsize

	glColor3f(30/255.0,118/255.0,110/255.0)
	
	for i in range(0, int(win_width/screen_gridsize)+1):
		glBegin(GL_QUADS)
		glVertex2f(x+i*screen_gridsize,win_height)
		glVertex2f(x+i*screen_gridsize,0)
		glVertex2f(x+i*screen_gridsize+1,0)
		glVertex2f(x+i*screen_gridsize+1,win_height)
		glEnd()

	for i in range(0, int(win_height/screen_gridsize)+1):
		glBegin(GL_QUADS)
		glVertex2f(win_width, y+i*screen_gridsize)
		glVertex2f(0, y+i*screen_gridsize)
		glVertex2f(0, y+i*screen_gridsize+1)
		glVertex2f(win_width, y+i*screen_gridsize+1)
		glEnd()

# Draw the boat on the water
def draw_boat():
	(x, y) = camera.lps_to_screen(pos.x, pos.y)

	#draw rudder
	glPushMatrix()
	glTranslatef(x, y, 0)
	glRotatef(heading-90, 0, 0, 1)
	draw_image(
		cur_rudder_img[0], # texture id
		(0, (-cur_boat_img[1][1]/2+cur_rudder_img[1][1]*0.1)*camera.scale), # local y coord of rudder, moves to end of boat 
		rudder_pos-90, # rudder pos
		(cur_rudder_img[1][0]*camera.scale,cur_rudder_img[1][1]*camera.scale)) # rudder visual size
	glPopMatrix()
	
	#draw boat
	draw_image(
		cur_boat_img[0],
		(x, y),
		heading-90,
		(cur_boat_img[1][0]*camera.scale, cur_boat_img[1][1]*camera.scale))
	
	#draw sail
	sail_angle = 90 * float(winch_max - winch_pos)/(winch_max - winch_min)
	if ane_reading <= 180:
		sail_angle = -sail_angle
	glPushMatrix()
	glTranslatef(x, y, 0)
	glRotatef(heading-90, 0, 0, 1)
	draw_image(
		cur_sail_img[0],
		(0.1*camera.scale, 0.5*camera.scale),
		sail_angle,
		(cur_sail_img[1][0]*camera.scale, cur_sail_img[1][1]*camera.scale))
	glPopMatrix()



# Draw the rudder diagram centered on (x, y)
def draw_status_boat(x, y):
	# draw boat
	scale = 60.0/cur_boat_img[1][1]
	draw_image(cur_boat_img[0], (x, y+15), 0, (cur_boat_img[1][0]*scale, cur_boat_img[1][1]*scale))
	
	# draw rudder
	rudder_scale = 26.0/cur_rudder_img[1][1]
	draw_image(
		cur_rudder_img[0],
		(x, y-10),
		rudder_pos-90,
		(cur_rudder_img[1][0]*rudder_scale, cur_rudder_img[1][1]*rudder_scale))
	
	sail_scale = 42.0/cur_sail_img[1][1]
	sail_angle = 90 * float(winch_max - winch_pos)/(winch_max - winch_min)
	draw_image(
		cur_sail_img[0],
		(x+1, y+20),
		sail_angle,
		(cur_sail_img[1][0]*sail_scale, cur_sail_img[1][1]*sail_scale))


def draw_speed_graph(x, y, size):
	glPushMatrix()
	glTranslatef(x,y,0)
	
	glColor3f(1.0, 1.0, 1.0)
	glBegin(GL_LINES)
	glVertex2f(-size/2, 0)
	glVertex2f(size/2, 0)
	glVertex2f(0, -size/2)
	glVertex2f(0, size/2)
	glEnd()
	
	glColor3f(0.2, 0.2, 0.2)
	for head, speed in speed_graph.items():
		radius = speed/20 * size
		ang = math.radians(head-90)
		draw_circle(2, math.cos(ang)*radius, math.sin(ang)*radius)
	
	glPopMatrix()

def draw_path():
	glPushMatrix()
	glColor3f(1.0, 1.0, 1.0)
	glBegin(GL_LINES)
	for i in range(0, len(path.points)-1):
		(x, y) = camera.lps_to_screen(path.points[i].x, path.points[i].y)
		(x_n, y_n) = camera.lps_to_screen(path.points[i+1].x, path.points[i+1].y)
		glVertex2f(x, y)
		glVertex2f(x_n ,y_n)
	glEnd()
	glPopMatrix()
	

# =*=*=*=*=*=*=*=*=*=*=*=*= Physics =*=*=*=*=*=*=*=*=*=*=*=*=

def calc_apparent_wind(true_wind, boat_speed, boat_heading):
	# Use constant wind speed of 8 m/s
	x = wind_speed*math.cos(math.radians(true_wind))
	x += boat_speed*math.cos(math.radians(boat_heading + 180))
	y = wind_speed*math.sin(math.radians(true_wind))
	y += boat_speed*math.sin(math.radians(boat_heading + 180))
	return (x, y)


def calc_direction(v):
	angle = math.degrees(math.atan2(v[1], v[0]))
	if angle < 0:
		angle += 360
	return angle


# returns -1 for port, 1 for starboard
def calc_tack(boat_heading, wind_heading): 
	diff = (wind_heading - boat_heading)
	if diff > 180:
		diff -= 360
	elif diff < -180:
		diff += 360
	
	# Dead run	
	if diff == 0:
		return 1.0
	return diff/abs(diff)

# returns heading of vector point from end of boom to mast
def calc_boom_heading(boat_heading, wind_heading, winch):
	winch_range = winch_max - winch_min
	
	tack = calc_tack(boat_heading, wind_heading)
	# Note close-hauled boom is not quite parallel with boat
	return boat_heading - tack * ((winch_max - winch) * 75/winch_range + 15)

def pause_sim():
	global pause
	global speed
	global pre_pause_speed

	pause = not pause
	if pause is True:
		pre_pause_speed = speed
		speed = 0
	else:
		speed = pre_pause_speed
	sliders["Sim speed"].change_val(speed*100)


def calc(_):
	global pos
	global heading
	global target_heading
	global last_time
	global clock
	global rudder_pos
	global layline
	global ane_reading
	global boat_speed
	global path
	global prev_path_time
	# Calculate the in-simulator time
	if(last_time == -1):
		last_time = time.time()
	real_dt = time.time() - last_time
	dt = real_dt * speed
	last_time = time.time()
	clock += dt
	time_msg = Clock()
	time_msg.clock.secs = clock
	time_msg.clock.nsecs = (clock % 1) * (10**9)
	clock_pub.publish(time_msg)

	if follow_boat:
		camera.x = pos.x
		camera.y = pos.y
	else:	
		# Move camera when mouse is near edge of screen
		# I put this in here so that camera move speed is bound to time and not fps
		camera.x += camera_velocity.x * real_dt
		camera.y += camera_velocity.y * real_dt
	
	if sim_mode is SimMode.DEFAULT:
		# Calculate other things
		tack = calc_tack(heading, wind_heading)
		boom_heading = calc_boom_heading(heading, wind_heading, winch_pos)
		boom_vector = polar_to_rect(1, boom_heading)
		boom_perp_vector = polar_to_rect(1, boom_heading + tack*90)
		app_wind = calc_apparent_wind(wind_heading, boat_speed, heading)
		heading_vector = polar_to_rect(1, heading)
		# components of wind parallel and perpendicular to sail
		wind_par = -proj(app_wind, boom_vector)
		wind_perp = proj(app_wind, boom_perp_vector)
		
		if (wind_perp < 0):
			# Sail is backwinded/luffing
			acc = 0
		else:
			# Calculate drag component (major component when on run)		
			a_perp = 0.03*wind_perp**2
			acc = a_perp * proj(boom_perp_vector, heading_vector)
			# Calculate lift (major component when on reach)
			if wind_par > 0:
				a_par = 0.03*wind_par**2
				acc += a_par * proj(boom_perp_vector, heading_vector)
		
		# Wind drag on boat (prominent when in irons)
		acc += 0.01*proj(app_wind, heading_vector)

		# Water drag
		drag = 0.25*boat_speed*abs(boat_speed)
		rudder_drag = 0.2*drag*abs(math.cos(math.radians(rudder_pos)))
		drag += rudder_drag
		boat_speed += (acc - drag)*dt
		
		#old_wind_head = ane_reading
		
		heading -= (rudder_pos-90)*0.4*boat_speed * dt
		heading %= 360
			
		# Update anemometer reading because of new heading and speed
		update_wind()
		
		
		speed_graph[int(ane_reading)%360] = boat_speed
		
		pos.x += math.cos(math.radians(heading)) * boat_speed * dt
		pos.y += math.sin(math.radians(heading)) * boat_speed * dt
		
		# Reset boat if out of bounds
		if abs(pos.x) > 10000 or abs(pos.y) > 10000:
			pos.x = 0
			pos.y = 0
			camera.x = 0
			camera.y = 0
			path = PointArray()
		
		update_vision()

		update_gps()

		# Don't let drawn path be too long 
		if display_path:
			if len(path.points) > 1000:
				path = PointArray()
			# Only add current point every half second
			if clock > 0.5 + prev_path_time:
				pt = Point()
				pt.x = pos.x
				pt.y = pos.y
				path.points.append(pt)
				prev_path_time = clock
	
	glutPostRedisplay()
	
	if sim_is_running:
		glutTimerFunc(1000/30, calc, 0)
	else:
		glutDestroyWindow(win_ID)
		glutLeaveMainLoop()


# Returns magnitude of projection of u onto v
def proj(u,v):
	v_mag = math.sqrt(v[0]**2 + v[1]**2)
	return (u[0]*v[0] + u[1]*v[1])/v_mag


# Returns (x,y), given radius and angle in degrees
def polar_to_rect(rad, ang):
	return (rad * math.cos(math.radians(ang)), rad * math.sin(math.radians(ang)))


# =*=*=*=*=*=*=*=*=*=*=*=*= Initialization =*=*=*=*=*=*=*=*=*=*=*=*=

def load_image_resources():
	global cur_boat_img
	global cur_rudder_img
	global cur_sail_img
	global compass_img
	global compass_pointer_img
	# Load all the images
	compass_img = load_image('../meshes/compass.png', (256,256))
	compass_pointer_img = load_image('../meshes/compass_pointer.png', (23,128))
	
	orig=load_image('../meshes/niceboat.png', (64,128))
	boat_imgs["orig"] = orig, (2,4)
	orig_rudder = load_image('../meshes/rudder.png', (32,64))
	rudder_imgs["orig"] = orig_rudder, (1.5,3)
	orig_sail = load_image('../meshes/sail.png', (32,64))
	sail_imgs["orig"] = orig_sail, (2,4)
	
	pirate_id=load_image('../meshes/pirate_boat.png', (39,56))
	boat_imgs["pirate"] = pirate_id, (3,4)
	# use orig rudder and sail	
	rudder_imgs["pirate"] = orig_rudder, (1.5,3)
	sail_imgs["pirate"] = orig_sail, (2,4)
	
	SPACE_X = load_image('../meshes/falcon_heavy.png', (1040,5842))
	boat_imgs["mars"] = SPACE_X, (2,10)
	# use orig rudder
	rudder_imgs["mars"] = orig_rudder, (2,3)
	roadster = load_image('../meshes/roadster.png', (128,256))
	sail_imgs["mars"] = roadster, (3,6)
	
	# Load stanard/orig boat by default
	cur_boat_img = boat_imgs["orig"]
	cur_rudder_img = rudder_imgs["orig"]
	cur_sail_img = sail_imgs["orig"]


def load_font_resources():
	global open_sans_font
	open_sans_font = load_font('../meshes/OpenSans/OpenSans-Light.ttf', 2048)
	set_font(open_sans_font)


def init_sliders():
	global sliders
	wind_speed_slider = Slider(win_width-100,win_height-160,80,25, wind_speed_slider_callback, 0, 15, 5)
	wind_speed_slider.set_color(0,0,0)
	sliders["Wind speed"] = wind_speed_slider

	sim_speed_slider = Slider(win_width-100,win_height-200,80,25, sim_speed_slider_callback, 0, 1000, 200)
	sim_speed_slider.set_color(0,0,0)
	sliders["Sim speed"] = sim_speed_slider


def wind_speed_slider_callback(value):
	global wind_speed
	wind_speed = value

def sim_speed_slider_callback(value):
	global speed
	speed = value / 100.0

def init_2D(r,g,b):
	glClearColor(r,g,b,0.0)  
	glViewport(0, 0, win_width, win_height)
	glOrtho(0.0, win_width, 0.0, win_height, -1, 1)


def init_GLUT():
	global win_ID
	global pos
	glutInit(sys.argv)
	glutInitWindowSize(win_width, win_height)
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
	win_ID = glutCreateWindow('UW Sailbot Simulator')
	
	#Setup the window with blue background
	init_2D(90/255.0,155/255.0,230/255.0)
	glutDisplayFunc(redraw)
	glutReshapeFunc(resize)
	glutMouseFunc(mouse_handler)
	glutPassiveMotionFunc(passive_mouse_handler)
	glutMotionFunc(motion_handler)
	glutKeyboardFunc(ASCII_handler)
	glutSpecialFunc(keyboard_handler)
	glutTimerFunc(1000/30, calc, 0)
	
	load_image_resources()
	load_font_resources()
	init_sliders()
	
	glutMainLoop()


def listener():
	# Setup subscribers
	rospy.init_node('visualizer', anonymous=True)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('rudder', Float32, rudder_callback)
	rospy.Subscriber('winch', Int32, winch_callback)
	rospy.Subscriber('waypoints_raw', WaypointArray, waypoints_callback)
	rospy.Subscriber('target_heading', Float32, target_heading_callback)
	
	# So we can use real wind data in simulation mode
	# Data from this will override true wind in sim
	rospy.Subscriber('mock_true_wind', Float32, mock_true_wind_callback)
	
	# subscribers for replay mode
	#rospy.Subscriber('lps', Point, lps_callback)
	rospy.Subscriber('odometry/filtered', Odometry, odom_callback)
	rospy.Subscriber('compass', Float32, compass_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	rospy.Subscriber('rudder_pid/output', Float32, rudder_output_callback)
	rospy.Subscriber('rudder_pid/input', Float32, rudder_input_callback)
	rospy.Subscriber('rudder_pid/setpoint', Float32, rudder_setpoint_callback)
	rospy.Subscriber('rudder_pid/enable', Bool, rudder_enable_callback)
	rospy.Subscriber('gps_raw', GPS, gps_raw_callback)
	rospy.Subscriber('obstacles', PointArray, obstacles_callback)
	rospy.Subscriber('target_point', Waypoint, target_point_callback)
	rospy.Subscriber('bounding_box', PointArray, bounding_box_callback)
	rospy.Subscriber('vision', PointArray, vision_callback)


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

