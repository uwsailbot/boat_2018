#!/usr/bin/env python

import rospy
import sys, select, termios, tty, os
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from boat_msgs.msg import BoatState
from boat_msgs.msg import GPS
from boat_msgs.msg import Point
from boat_msgs.msg import PointArray
from boat_msgs.srv import ConvertPoint
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler
import time
from sys import argv
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import pygame

from PIL import Image

# Cheat codes
codes = []
cur_input = ""
sound = False

# OpenGL data
win_ID = 0
win_width = 640
win_height = 480

# Resources
boat_imgs = []
rudder_imgs = []
sail_imgs = []
cur_boat_img = Image.Image()
cur_rudder_img = Image.Image()
cur_sail_img = Image.Image()

# Simulation data and consts
should_sim_joy = False
sim_is_running = True
speed = 10
clock = 0
last_time = -1
boat_speed = 4 # px/s
layline = rospy.get_param('/boat/layline')
winch_min = rospy.get_param('/boat/winch_min')
winch_max = rospy.get_param('/boat/winch_max')

# ROS data
wind_heading = 270
ane_reading = 0
pos = Point()
heading = 90
target_heading = 90
state = BoatState()
rudder_pos = 90
winch_pos = 2000
local_points = PointArray()
gps_points = PointArray()
joy = Joy()
joy.axes = [0]*8
joy.buttons = [0]*11

# =*=*=*=*=*=*=*=*=*=*=*=*= ROS Publishers & Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=

waypoint_pub = rospy.Publisher('waypoints_raw', PointArray, queue_size = 1)
wind_pub = rospy.Publisher('anemometer', Float32, queue_size = 1)
gps_pub = rospy.Publisher('gps_raw', GPS, queue_size = 1)
orientation_pub = rospy.Publisher('imu/data', Imu, queue_size = 1)
joy_pub = rospy.Publisher('joy', Joy, queue_size = 1)
to_gps = rospy.ServiceProxy('lps_to_gps', ConvertPoint)
to_lps = rospy.ServiceProxy('gps_to_lps', ConvertPoint)


def update_gps():
	gps = GPS()
	gps.status = GPS.STATUS_FIX
	coords = to_gps(pos)
	gps.latitude = coords.pt.y
	gps.longitude = coords.pt.x
	gps_pub.publish(gps)
	
	orientation = quaternion_from_euler(0,0,math.radians(heading))
	imu = Imu()
	
	# Convertion because they are different types
	imu.orientation.x = orientation[0]
	imu.orientation.y = orientation[1]
	imu.orientation.z = orientation[2]
	imu.orientation.w = orientation[3]
	
	orientation_pub.publish(imu)


def update_wind(offset):
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
	global joy
	
	# Unpush the tacking button if tacking has completed so we don't tack forever
	if state.minor is BoatState.MIN_TACKING and newState.minor is not BoatState.MIN_TACKING:
		joy.buttons[2] = 1
		joy.buttons[0] = 0
		joy_pub.publish(joy)
	
	state = newState


def rudder_callback(pos):
	global rudder_pos
	rudder_pos = pos.data


def winch_callback(pos):
	global winch_pos
	winch_pos = pos.data


# Callback to restore local coord waypoints after publishing gps coord
def waypoints_callback(newPoints):
	global local_points
	global gps_points
	global sound
	
	# Refresh GPS point list
	gps_points = newPoints
	temp_points = PointArray()

	if (len(local_points.points)-1) is len(newPoints.points) and sound and cur_boat_img is boat_imgs[2]:
		pygame.mixer.music.play()
	
	# Convert all GPS points and store them as local points to draw
	for point in gps_points.points:
		local_point = to_lps(point).pt
		temp_points.points.append(local_point)
	local_points = temp_points


def target_heading_callback(angle):
	global target_heading
	target_heading = angle.data


# =*=*=*=*=*=*=*=*=*=*=*=*= OpenGL callbacks =*=*=*=*=*=*=*=*=*=*=*=*=

# Window resize callback
def resize(width, height):
	global win_width
	global win_height
	win_width = width
	win_height = height
	glutInitWindowSize(win_width, win_height)
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	gluOrtho2D(0.0, win_width, 0.0, win_height)
	glMatrixMode(GL_MODELVIEW)
	glLoadIdentity()


# Handler for mouse presses
def mouse_handler(button, state, x, y):
	global local_points
	global gps_points
	
	if state != GLUT_DOWN:
		return
		
	if button == GLUT_RIGHT_BUTTON:
		local_points = PointArray()
		gps_points = PointArray()
	else:
		newPt = Point()
		newPt.x = x - win_width/2
		newPt.y = -y + win_height/2
		coords = to_gps(newPt).pt
		gps_points.points.append(coords)
	waypoint_pub.publish(gps_points)


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
	global sound
	
	# Handle cheat codes
	cur_input += key;
	valid = False
	for code in codes:
		if code == cur_input:
			cur_boat_img = boat_imgs[codes.index(code)]
			#cur_rudder_img = rudder_imgs[codes.index(code)]
			#cur_sail_img = sail_imgs[codes.index(code)]
			valid = False
			break
		if code.startswith(cur_input):
			valid = True
	if not valid:
		cur_input = ""
	
	if key is chr(27):
		sim_is_running = False
	elif key is '1' and should_sim_joy:
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
		update_wind(5)
	elif key is 'd':
		update_wind(-5)
	elif key is 'w':
		speed += 0.1
	elif key is 's':
		speed -= 0.1
		speed = max(speed, 0)
	elif key is '0':
		sound = not sound
	elif key is ' ':
		pos.x = 0
		pos.y = 0
		update_gps()


# Main display rendering callback
def redraw():
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
	
	glViewport(0, 0, win_width, win_height)
	
	# Render stuff
	draw_waypoints()
	draw_boat()
	draw_target_heading_arrow()
	draw_status()

	glutSwapBuffers()


# =*=*=*=*=*=*=*=*=*=*=*=*= OpenGL Rendering =*=*=*=*=*=*=*=*=*=*=*=*=

def draw_pixel_rgb_i(x, y, rgb):
	glColor3f(rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0)
	glVertex2f(x,y)

def draw_pixel_rgba_i(x, y, rgba):
	glColor4f(rgba[0]/255.0, rgba[1]/255.0, rgba[2]/255.0, rgba[3]/255.0)
	glVertex2f(x,y)

def draw_image(image, x, y, angle):	
	glEnable(GL_TEXTURE_2D)
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
	glEnable(GL_BLEND)
	glPushMatrix()
	
	glTranslatef(x, y, 0)
	glRotatef(angle, 0, 0, 1)
	
	im_width, im_height = image.size
	im_data = image.getdata()
	x_offset = -im_width/2
	y_offset = -im_height/2
	
	glBegin(GL_POINTS)
	if image.mode == 'RGB':
		for i in range(0, im_width*im_height):
			draw_pixel_rgb_i(i%im_width+x_offset, i/im_width+y_offset, im_data[i])
	else:
		for i in range(0, im_width*im_height):
			draw_pixel_rgba_i(i%im_width+x_offset, i/im_width+y_offset, im_data[i])
	glEnd()
	
	glPopMatrix()
	glDisable(GL_TEXTURE_2D)
	glDisable(GL_BLEND)
	
# Render a circle centered at (x,y) with radius r
def draw_circle(r, x, y, quality=300):
	glBegin(GL_POLYGON)
	for i in range(0, quality):
		angle = 2 * math.pi * i / float(quality)
		curx = x + math.cos(angle) * r
		cury = y + math.sin(angle) * r
		glVertex2f(curx,cury)
	glEnd()

# Render the specified text with bottom left corner at (x,y)
def draw_text(text, x, y, h = 15.0):
	glPushMatrix()
	
	# Scale and translate the text as required
	scale = h/120.0
	glTranslatef(x,y,0)
	glScalef(scale, scale, 0.0)
	
	# Loop to display character by character
	for c in text:
		glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
	
	glPopMatrix()


# Draw all of the waypoint as red dots
def draw_waypoints():
	glPushMatrix()
	
	if len(local_points.points) > 0 and state.major is BoatState.MAJ_AUTONOMOUS:
		glColor3f(1,1,1)
		first_point = local_points.points[0]
		draw_circle(7,first_point.x + win_width/2.0, first_point.y + win_height/2.0)

	glColor3f(1,0,0)
	for p in local_points.points:
		x = p.x + win_width/2.0
		y = p.y + win_height/2.0
		draw_circle(5,x,y)
	
	glPopMatrix()
	

# Draw boat's target heading as an arrow centered at (x, y) pointing in the target heading's dirction
def draw_target_heading_arrow():
	if state.major is not BoatState.MAJ_AUTONOMOUS:
		return
	glPushMatrix()
	
	boat_x = pos.x + win_width/2.0
	boat_y = pos.y + win_height/2.0
	
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


# Draw the right-hand 'status' panel and all of its data
def draw_status():
	glPushMatrix()
	
	# Draw the box
	glColor3f(1.0, 1.0, 1.0)
	glBegin(GL_QUADS)
	glVertex2f(win_width,win_height)
	glVertex2f(win_width, 0)
	glVertex2f(win_width-120,0)
	glVertex2f(win_width-120,win_height)
	glEnd()
	
	# Draw the wind readout
	glColor3f(0.0, 0.0, 0.0)
	draw_text("Wind: " + str(wind_heading), win_width-110, win_height-20)
	draw_wind_arrow(win_width-60,win_height-50)
	
	# Draw the boat pos
	glColor3f(0.0, 0.0, 0.0)
	draw_text("X: %.1f" % pos.x, win_width-110, win_height*0.75)
	draw_text("Y: %.1f" % pos.y, win_width-110, win_height*0.75-15)
	draw_text("Spd: %.1f" % boat_speed, win_width-110, win_height*0.75-30)
	draw_text("Head: %.1f" % heading, win_width-110, win_height*0.75-45)
	
	# Draw the boat state
	major = ""
	if state.major is BoatState.MAJ_RC:
		major = "RC"
	elif state.major is BoatState.MAJ_AUTONOMOUS:
		major = "Auto"
	elif state.major is BoatState.MAJ_DISABLED:
		major = "Disabled"
		
	minor = ""
	if state.minor is BoatState.MIN_COMPLETE:
		minor = "Complete"
	elif state.minor is BoatState.MIN_PLANNING:
		minor = "Planning"
	elif state.minor is BoatState.MIN_TACKING:
		minor = "Tacking"
	draw_text("State", win_width-85, win_height*0.56, 24)
	draw_text("M: " + major, win_width-110, win_height*0.56-15)
	draw_text("m: " + minor, win_width-110, win_height*0.56-30)
	
	# Draw the rudder and winch pos
	draw_text("Rudder: %.1f" % rudder_pos, win_width-110, win_height*0.40)
	draw_text("Winch: %d" % winch_pos, win_width-110, win_height*0.40 - 15)
	
	# Draw the rudder diagram
	draw_rudder(win_width-55, win_height*0.2)
	
	# Draw the simulation speed
	draw_text("Spd: %.f%%" % (speed*100), win_width-110, 30)
	draw_text("Time: %.1f" % clock + "s", win_width-110, 15)
	
	glPopMatrix()


# Draw the arrow for the wind direction centered on (x, y)
def draw_wind_arrow(x,y):
	glPushMatrix()
	
	glTranslatef(x,y,0)
	glScalef(0.3,0.3,0)
	glColor3f(0.0, 1.0, 0.0)
	draw_circle(65,0,0)
	
	glRotatef(wind_heading-90,0,0,1)
	glTranslatef(0,35,0)
	
	glColor3f(1.0, 0.0, 0.0)
	glBegin(GL_QUADS)
	glVertex2f(-10,0)
	glVertex2f(-10,-100)
	glVertex2f(10,-100)
	glVertex2f(10,0)
	glEnd()
	
	glBegin(GL_TRIANGLES)
	glVertex2f(-20,0)
	glVertex2f(0,30)
	glVertex2f(20,0)
	glEnd()
	
	glPopMatrix()

# Draw the boat on the water
def draw_boat():
	x = pos.x + win_width/2.0
	y = pos.y + win_height/2.0
	draw_image(cur_boat_img, x, y, heading-90)

# Draw the rudder diagram centered on (x, y)
def draw_rudder(x, y):
	glPushMatrix()
	
	glTranslatef(x, y, 0)
	
	glColor3f(1.0, 0.5, 0.0)
	glBegin(GL_POLYGON)
	glVertex2f(-10,0)
	glVertex2f(-7,40)
	glVertex2f(0,60)
	glVertex2f(7,40)
	glVertex2f(10,0)
	glEnd()
	
	glRotatef(rudder_pos-90, 0, 0, 1)
	glColor3f(0.5,0.2,0.2)
	glBegin(GL_POLYGON)
	glVertex2f(-4,10)
	glVertex2f(-4,-20)
	glVertex2f(4,-20)
	glVertex2f(4,10)
	glEnd()
	
	glPopMatrix()


# =*=*=*=*=*=*=*=*=*=*=*=*= Physics =*=*=*=*=*=*=*=*=*=*=*=*=
def calc_apparent_wind(true_wind, boat_speed, boat_heading):
	# Use constant wind speed of 8 m/s
	x = 8*math.cos(math.radians(true_wind))
	x += boat_speed*math.cos(math.radians(boat_heading + 180))
	y = 8*math.sin(math.radians(true_wind))
	y += boat_speed*math.sin(math.radians(boat_heading + 180))
	return (x, y)

def calc_direction(v):
	angle = math.degrees(math.atan2(v[1], v[0]))
	if angle < 0:
		angle += 360
	return angle

# returns 1 for port, -1 for starboard
def calc_tack(boat_heading, wind_heading): 
	diff = (wind_heading - boat_heading)
	if diff > 180:
		diff -= 360
	elif diff < -180:
		diff += 360

	return diff/abs(diff)

# returns heading of vector point from end of boom to mast
def calc_boom_heading(boat_heading, wind_heading, winch):
	global winch_min
	global winch_max
	winch_range = winch_max - winch_min

	tack = calc_tack(boat_heading, wind_heading)
	# Note close-hauled boom is not quite parallel with boat
	return boat_heading + tack * ((winch_max - winch) * 75/winch_range + 15)
	

def calc(_):
	global pos
	global heading
	global target_heading
	global local_points
	global last_time
	global clock
	global rudder_pos
	global layline
	global ane_reading
	global boat_speed
	
	# Calculate the in-simulator time
	if(last_time == -1):
		last_time = time.time()
	dt = (time.time() - last_time) * speed
	last_time = time.time()
	clock += dt
	
	dh = wind_heading - heading
	while dh > 180:
		dh -= 360
	while dh < -180:
		dh += 360
	dh = max(0,(abs(dh)-25))/150
	
	boat_speed += 5 * (1-dh) * dt
	boat_speed -= math.copysign(0.04*boat_speed*boat_speed, boat_speed)
	boat_speed = min(boat_speed, 10)
	boat_speed = max(boat_speed, -10)
	
	if(state.major != BoatState.MAJ_DISABLED):
		heading -= (rudder_pos-90)*0.1 
		heading %= 360
		
		# Update anemometer reading because of new heading
		update_wind(0)
		
		# Our laylines are set further out than the boat will actually hit irons at, so physics wise the laylines are actually at laylines-TOL, which
		# is where it should hit irons
		TOL = 5

		# Outside of laylines, speed works normally
		#if ane_reading >= (180+layline-TOL) or ane_reading <= (180 - layline+TOL):
		pos.x += math.cos(math.radians(heading)) * boat_speed * dt
		pos.y += math.sin(math.radians(heading)) * boat_speed * dt
	
	update_gps()
	glutPostRedisplay()
	
	if sim_is_running:
		glutTimerFunc(1000/30, calc, 0)
	else:
		glutDestroyWindow(win_ID)
		exit(0)

# Returns (x,y), given radius and angle in degrees
def polar_to_rect(rad, ang):
	return (rad * math.cos(math.radians(ang)), rad * math.sin(math.radians(ang)))



# =*=*=*=*=*=*=*=*=*=*=*=*= Initialization =*=*=*=*=*=*=*=*=*=*=*=*=

def init_2D(r,g,b):
	glClearColor(r,g,b,0.0)  
	glViewport(0, 0, win_width, win_height)
	gluOrtho2D(0.0, win_width, 0.0, win_height)


def init_GL():
	global win_ID
	global pos
	glutInit(sys.argv)
	glutInitWindowSize(win_width, win_height)
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
	win_ID = glutCreateWindow('Simulator')
	
	#Setup the window with blue background
	init_2D(66/255.0,134/255.0,244/255.0)
	glutDisplayFunc(redraw)
	glutReshapeFunc(resize)
	glutMouseFunc(mouse_handler)
	glutKeyboardFunc(ASCII_handler)
	glutSpecialFunc(keyboard_handler)
	glutTimerFunc(1000/30, calc, 0)
	glutMainLoop()

def rel_to_abs_filepath(filepath):
	abs_filepath = os.path.dirname(os.path.realpath(__file__))
	while filepath.startswith('../'):
		filepath_arr = filepath.split('/')
		filepath_arr.pop(0)
		filepath = ''.join(['/'+str(s) for s in filepath_arr])[1:]
		abs_filepath_arr = abs_filepath.split('/')
		abs_filepath_arr = abs_filepath_arr[:-1]
		abs_filepath =''.join(['/'+str(s) for s in abs_filepath_arr])[1:]
	return abs_filepath + '/' + filepath
	
def load_image(filepath, size):
	# loads and returns an image
	abs_filepath = rel_to_abs_filepath(filepath)
	im = Image.open(abs_filepath)
	im = im.transpose(Image.FLIP_TOP_BOTTOM)
	im = im.resize(size, Image.NEAREST)
	return im

def listener():
	# Setup subscribers
	rospy.init_node('visualizer', anonymous=True)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('rudder', Float32, rudder_callback)
	rospy.Subscriber('winch', Int32, winch_callback)
	rospy.Subscriber('waypoints_raw', PointArray, waypoints_callback)
	rospy.Subscriber('target_heading', Float32, target_heading_callback)

if __name__ == '__main__':
	should_sim_joy = not("-j" in argv or "-J" in argv)
	
	# Load all the images
	codes.append("orig")
	boat_imgs.append(load_image('../meshes/niceboat.png', (24,48)))
	codes.append("pirate")
	boat_imgs.append(load_image('../meshes/pirate_boat.png', (36,48)))
	codes.append("mars")
	boat_imgs.append(load_image('../meshes/falcon_heavy.png', (1040/55,5842/55)))
	cur_boat_img = boat_imgs[0]
	pygame.mixer.init()
	pygame.mixer.music.load(rel_to_abs_filepath("../meshes/lemme-smash.mp3"))

	try:
		listener()
	except rospy.ROSInterruptException:
		pass
	
	init_GL()

