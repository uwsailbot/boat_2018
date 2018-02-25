#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32
from boat_msgs.msg import Point
from boat_msgs.msg import PointArray
from boat_msgs.msg import GPS
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
import math

# Declare global variables needed for the node
origin_lps = Point()
RADIUS = 6378137 # Radius of earth, in meters
ane_reading = 0

# Declare the publishers for the node
gps_pub = rospy.Publisher('odometry_navsatfix', NavSatFix, queue_size=10)
lps_pub = rospy.Publisher('lps', Point, queue_size=10)
waypoints_pub = rospy.Publisher('waypoints', PointArray, queue_size=10)
wind_heading_pub = rospy.Publisher('wind_heading', Float32, queue_size=10)

# Convert the current boat location from gps to lps
# Convert GPS to NavSatFix msg for filtering
def gps_callback(gps):
	global gps_pub
	global lps_pub

	gps_parsed = NavSatFix()
	gps_parsed.header = gps.header
	gps_parsed.status = gps.status
	gps_parsed.service = NavSatStatus.SERVICE_GPS
	gps_parsed.latitude = gps.latitude
	gps_parsed.longitude = gps.longitude
	gps_parsed.altitude = gps.altitude
		
	# TODO: Add covariance?
	gps_pub.publish(gps_parsed)
	local = gpsToLps(getCoords(gps))
	
	# Commenting this out so that we don't spam the output
	rospy.loginfo(rospy.get_caller_id() + " Long: %f, Lat: %f --- X: %f, Y: %f", getCoords(gps).x, getCoords(gps).y, local.x, local.y)
	lps_pub.publish(local)

# Convert the list of waypoints from gps to lps
def waypoints_callback(waypoints_raw):
	global waypoints_pub
	waypoints = PointArray()

	for curpoint in waypoints_raw.points:
		waypoints.points.append(gpsToLps(curpoint))
		
	rospy.loginfo(rospy.get_caller_id() + " Converted waypoints to local positioning system")
	waypoints_pub.publish(waypoints)

def anemometer_callback(anemometer):
	global ane_reading
	ane_reading = anemometer.data

def compass_callback(compass):
	global ane_reading
	global wind_heading_pub

	wind_reading = Float32()
	wind_reading.data = (ane_reading + 180) % 360
	wind_heading.data = (ane_reading + compass.data) % 360
	wind_heading_pub.publish(wind_heading)
	
# Convert from gps to lps
def gpsToLps(coords):
	global RADIUS
	global origin_lps
	local = Point()
	local.x = RADIUS * cosd(coords.y) * math.radians(coords.x) - origin_lps.x
	local.y = RADIUS * math.radians(coords.y) - origin_lps.y
	return local
	
# Extract longitude and latitude from boat_nav.msg.GPS
def getCoords(gps):
	coords = Point()
	coords.x = gps.longitude
	coords.y = gps.latitude
	return coords

def cosd(angle):
	return math.cos(math.radians(angle))

def sind(angle):
	return math.sin(math.radians(angle))

# Initialize the node
def listener():
	global origin_lps
    
	rospy.init_node('sensor_parser')
    
    # setup the origin
	origin_coords = rospy.wait_for_message('gps_raw', GPS)
	origin_lps = gpsToLps(getCoords(origin_coords))
	rospy.loginfo("Got origin: x:%f y:%f", origin_lps.x, origin_lps.y)
    
    
	rospy.Subscriber('gps_raw', GPS, gps_callback)
	rospy.Subscriber('waypoints_raw', PointArray, waypoints_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	
	# TODO: Change to whichever compass topic is more accurate
	rospy.Subscriber('compass_mag', Float32, compass_callback)
	rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
