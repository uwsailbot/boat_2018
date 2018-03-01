#!/usr/bin/env python

import rospy
import sys, select, termios, tty
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from boat_msgs.msg import BoatState
from boat_msgs.msg import GPS
from boat_msgs.msg import Point
from boat_msgs.msg import PointArray
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler
import time
import sys
from sys import argv
import numpy
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

windowID = 0
winWidth = 640
winHeight = 480

speed = 10
clock = 0
lastTime = -1
boatSpeed = 4 # px/s
RADIUS = 6378137 # Radius of earth, in meters


simJoy = False

wind_heading = 90
pos = Point()
heading = 90
state = BoatState()
rudderPos = 90
winchPos = 0
points = PointArray()
gps_points = PointArray()
joy = Joy()
joy.axes = [0]*8
joy.buttons = [0]*11


# =*=*=*=*=*=*=*=*=*=*=*=*= ROS Publishers & Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=

waypoint_pub = rospy.Publisher('waypoints_raw', PointArray, queue_size = 1)
wind_pub = rospy.Publisher('anemometer', Float32, queue_size = 1)
gps_pub = rospy.Publisher('gps_raw', GPS, queue_size = 1)
orientation_pub = rospy.Publisher('imu/data', Imu, queue_size = 1)
joy_pub = rospy.Publisher('/joy', Joy, queue_size = 1)


def updateGPS():
    gps = GPS()
    gps.status = GPS.STATUS_FIX

    # Vaguely uncertain of this math https://sciencing.com/convert-xy-coordinates-longitude-latitude-8449009.html
    #gps.longitude = numpy.arctan2(pos.y, pos.x)
    #print(gps.longitude)
    #gps.latitude = numpy.arccos((pos.y/RADIUS) / numpy.sin(gps.longitude))
    coords = local_to_gps(pos)
    gps.latitude = coords.x
    gps.longitude = coords.y
    gps_pub.publish(gps)

    orientation = quaternion_from_euler(0,0,numpy.radians(heading))
    imu = Imu()
    
    # Convertion because they are different types
    imu.orientation.x = orientation[0]
    imu.orientation.y = orientation[1]
    imu.orientation.z = orientation[2]
    imu.orientation.w = orientation[3]

    orientation_pub.publish(imu)
 
def updateWind(offset):
    global wind_heading
    global heading

    # Publish new anemometer message to relay requested wind data
    wind_heading = wind_heading + offset
    if wind_heading >= 360:
        wind_heading = wind_heading - 360
    elif wind_heading < 0:
        wind_heading = wind_heading + 360
    
    ane_reading = Float32()
    ane_reading.data = (wind_heading - heading) % 360
    if ane_reading.data < 0:
        ane_reading.data + 360
    wind_pub.publish(ane_reading)

def boat_state_callback(newState):
    global state
    state = newState
    
def rudder_callback(pos):
    global rudderPos
    rudderPos = pos.data
    
def winch_callback(pos):
    global winchPos
    winchPos = pos.data
    
# Callback to restore local coord waypoints after publishing gps coord
def waypoints_callback(newPoints):
    global points
    points = newPoints

# =*=*=*=*=*=*=*=*=*=*=*=*= OpenGL callbacks =*=*=*=*=*=*=*=*=*=*=*=*=

def resize(width, height):
    global winWidth
    global winHeight
    winWidth = width
    winHeight = height
    glutInitWindowSize(winWidth, winHeight)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluOrtho2D(0.0, winWidth, 0.0, winHeight)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def mouseHandler(button, state, x, y):
    global points
    global gps_points
    
    if state != GLUT_DOWN:
        return
        
    if button == GLUT_RIGHT_BUTTON:
        points = PointArray()
        gps_points = PointArray()
    else:
        newPt = Point()
        newPt.x = x - winWidth/2
        newPt.y = -y + winHeight/2
        print newPt
        coords = local_to_gps(newPt)
        print coords
        gps_points.points.append(coords)
    waypoint_pub.publish(gps_points)

def keyboardHandler(key, mousex, mousey):
    global joy
    if key == GLUT_KEY_LEFT and simJoy:
        joy.axes[0] = max(joy.axes[0]-0.1, -1)
        joy_pub.publish(joy)
    elif key == GLUT_KEY_RIGHT and simJoy:
        joy.axes[0] = min(joy.axes[0]+0.1, 1)
        joy_pub.publish(joy)
    elif key == GLUT_KEY_UP or key == GLUT_KEY_DOWN and simJoy:
        joy.axes[0] = 0
        joy_pub.publish(joy)
    
def ASCIIHandler(key, mousex, mousey):
    global speed
    global joy

    if key is chr(27):
        glutDestroyWindow(windowID)
        exit(0)
    elif key is '1' and simJoy:
        joy.buttons[4] = 1
        joy.buttons[5] = 0
        joy.buttons[8] = 0
        joy_pub.publish(joy)
    elif key is '2' and simJoy:
        joy.buttons[4] = 0
        joy.buttons[5] = 1
        joy.buttons[8] = 0
        joy_pub.publish(joy)
    elif key is '3' and simJoy:
        joy.buttons[4] = 0
        joy.buttons[5] = 0
        joy.buttons[8] = 1
        joy_pub.publish(joy)
    elif key is 'a':
        updateWind(5)
    elif key is 'd':
        updateWind(-5)
    elif key is 'w':
        speed += 0.1
    elif key is 's':
        speed -= 0.1
        speed = max(speed, 0)
    elif key is ' ':
        pos.x = 0
        pos.y = 0
        updateGPS()
        
        
def redraw():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glViewport(0, 0, winWidth, winHeight)
   
    # Render stuff
    drawPoints()
    drawBoat()
    drawStatus()

    glutSwapBuffers()


# =*=*=*=*=*=*=*=*=*=*=*=*= OpenGL Rendering =*=*=*=*=*=*=*=*=*=*=*=*=

# Render a circle centered at (x,y) with radius r
def drawCircle(r, x, y):

    glBegin(GL_POLYGON)
    for i in range(0,300):
        angle = 2 * numpy.pi * i / 300.0
        curx = x + numpy.cos(angle) * r
        cury = y + numpy.sin(angle) * r
        glVertex2f(curx,cury)
    glEnd()

# Render the specified text with bottom left corner at (x,y)
def drawText(text, x, y, h = 15.0):
    glPushMatrix()

    # set the position of the text in the window using the x and y coordinates
    glTranslatef(x,y,0)
    
    f = h/120.0
    glScalef(f,f, 0.0)

    # loop to display character by character
    for c in text:
        glutStrokeCharacter(GLUT_STROKE_ROMAN, ord(c))
    
    glPopMatrix()
        
        
def drawPoints():
    glPushMatrix()

    for p in points.points:
    
        x = p.x + winWidth/2.0
        y = p.y + winHeight/2.0
    
        glColor3f(1,0,0)
        drawCircle(5,x,y)
    
    glPopMatrix()

def drawStatus():
    glPushMatrix()
    
    # Draw the box
    glColor3f(1.0, 1.0, 1.0)
    glBegin(GL_QUADS)
    glVertex2f(winWidth,winHeight)
    glVertex2f(winWidth, 0)
    glVertex2f(winWidth-120,0)
    glVertex2f(winWidth-120,winHeight)
    glEnd()
    
    # Draw the wind readout
    glColor3f(0.0, 0.0, 0.0)
    drawText("Wind: " + str(wind_heading), winWidth-110, winHeight-20)
    drawWindArrow(winWidth-60,winHeight-50)
    
    # Draw the boat pos
    glColor3f(0.0, 0.0, 0.0)
    drawText("X: %.1f" % pos.x, winWidth-110, winHeight*0.75)
    drawText("Y: %.1f" % pos.y, winWidth-110, winHeight*0.75-15)
    drawText("Spd: %.1f" % boatSpeed, winWidth-110, winHeight*0.75-30)
    drawText("Head: %.1f" % heading, winWidth-110, winHeight*0.75-45)
    
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
    drawText("State", winWidth-85, winHeight*0.56, 24)
    drawText("M: " + major, winWidth-110, winHeight*0.56-15)
    drawText("m: " + minor, winWidth-110, winHeight*0.56-30)
    
    drawText("Rudder: %.1f" % rudderPos, winWidth-110, winHeight*0.40)
    drawText("Winch %.1d" % winchPos, winWidth-110, winHeight*0.40 - 15)
    
    drawRudder(winWidth-55, winHeight*0.2)
    
    drawText("Spd: %.f%%" % (speed*100), winWidth-110, 30)
    drawText("Time: %.1f" % clock + "s", winWidth-110, 15)
    
    glPopMatrix()

def drawWindArrow(x,y):
    glPushMatrix()
    
    glTranslatef(x,y,0)
    glScalef(0.3,0.3,0)
    glColor3f(0.0, 1.0, 0.0)
    drawCircle(65,0,0)
    
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
    
def drawBoat():
    glPushMatrix()
    
    x = pos.x + winWidth/2.0
    y = pos.y + winHeight/2.0
    
    glTranslatef(x, y, 0)    
    glRotatef(heading-90, 0, 0, 1)
    
    glColor3f(1.0, 215/225.0, 145/225.0)
    glBegin(GL_POLYGON)
    glVertex2f(-5,0)
    glVertex2f(0,15)
    glVertex2f(5,0)
    glVertex2f(0,-5)
    glEnd()
    
    glPopMatrix()
    
def drawRudder(x, y):
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
    
    glRotatef(rudderPos-90, 0, 0, 1)
    glColor3f(0.5,0.2,0.2)
    glBegin(GL_POLYGON)
    glVertex2f(-4,10)
    glVertex2f(-4,-20)
    glVertex2f(4,-20)
    glVertex2f(4,10)
    glEnd()
    
    glPopMatrix()


# =*=*=*=*=*=*=*=*=*=*=*=*= Physics =*=*=*=*=*=*=*=*=*=*=*=*=
def calc(running):

    global pos
    global heading
    global points
    global lastTime
    global clock
    global rudderPos

    if(lastTime == -1):
        lastTime = time.time()

    dt = (time.time() - lastTime) * speed
    lastTime = time.time()
    clock += dt
    
    
    # This all to be removed since it is handled by the rest of the boat
    #if(state.major == BoatState.MAJ_AUTONOMOUS):
    #    if len(points.points) >= 1:
    #        y = points.points[0].y
    #        x = points.points[0].x
    #        
    #        # This is broken but whatever it's getting deleted anyways
    #        h1 = heading
    #        h2 = (numpy.degrees(numpy.arctan2(y-pos.y, x-pos.x))+360)%360
    #        dh = h1 - h2
    #        
    #        if(dh > 0.1):
    #            rudderPos = 120
    #        elif(dh < -0.1):
    #            rudderPos = 60
    #        else:
    #            rudderPos = 90
    #        
    #        if numpy.sqrt((pos.x-x)*(pos.x-x)+(pos.y-y)*(pos.y-y)) < 3:
    #            points.points.remove(points.points[0])
    #            waypoint_pub.publish(points)
    #    else:
    #        update_boat_state(BoatState.MAJ_DISABLED)

        
    if(state.major != BoatState.MAJ_DISABLED):
        heading -= (rudderPos-90)*0.1
        heading %= 360
        pos.x += numpy.cos(numpy.radians(heading)) * boatSpeed * dt
        pos.y += numpy.sin(numpy.radians(heading)) * boatSpeed * dt
        
    updateGPS()
    glutPostRedisplay()
    
    if running:
       glutTimerFunc(1000/30, calc, 1)

def local_to_gps(pos):
    gps = Point()
    gps.x = numpy.degrees(pos.y/RADIUS)
    print gps.x
    gps.y = numpy.degrees((pos.x/RADIUS) / numpy.cos(numpy.radians(gps.x)))
    return gps


# =*=*=*=*=*=*=*=*=*=*=*=*= Initialization =*=*=*=*=*=*=*=*=*=*=*=*=

def init2D(r,g,b):
    glClearColor(r,g,b,0.0)  
    glViewport(0, 0, winWidth, winHeight)
    gluOrtho2D(0.0, winWidth, 0.0, winHeight)

def initGL():
    global windowID
    glutInit(sys.argv)
    glutInitWindowSize(winWidth, winHeight)
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
    windowID = glutCreateWindow('Simulator')
    
    #Setup the window with blue background
    init2D(66/255.0,134/255.0,244/255.0)
    glutDisplayFunc(redraw)
    glutReshapeFunc(resize)
    glutMouseFunc(mouseHandler)
    glutKeyboardFunc(ASCIIHandler)
    glutSpecialFunc(keyboardHandler)
    glutTimerFunc(1000/30, calc, 1)
    glutMainLoop()

def listener():
	# Setup subscribers
	rospy.init_node('visualizer', anonymous=True)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('rudder', Float32, rudder_callback)
	rospy.Subscriber('winch', Int32, winch_callback)
	rospy.Subscriber('waypoints', PointArray, waypoints_callback)

if __name__ == '__main__':
    simJoy = not("-j" in argv or "-J" in argv)
    
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    
    initGL()