#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32
from boat_msgs.msg import Point
from boat_msgs.msg import PointArray
from boat_msgs.msg import GPS
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import Imu
from boat_msgs.srv import ConvertPoint, ConvertPointResponse
from tf.transformations import euler_from_quaternion
import math

# Declare global variables needed for the node
origin_lps = Point()
RADIUS = rospy.get_param('/boat/radius')

# Declare the publishers for the node
gps_pub = rospy.Publisher('odometry_navsatfix', NavSatFix, queue_size=10)
lps_pub = rospy.Publisher('lps', Point, queue_size=10)
compass_pub = rospy.Publisher('compass', Float32, queue_size=10)

# =*=*=*=*=*=*=*=*=*=*=*=*= Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=

# Convert the current boat location from gps to lps
# Convert GPS to NavSatFix msg for filtering
def gps_callback(gps):
	global gps_pub
	global lps_pub
	
	gps_parsed = NavSatFix()
	gps_parsed.header = gps.header
	gps_parsed.status.status = gps.status
	gps_parsed.status.service = NavSatStatus.SERVICE_GPS
	gps_parsed.latitude = gps.latitude
	gps_parsed.longitude = gps.longitude
	gps_parsed.altitude = gps.altitude
	
	# TODO: Add covariance?
	gps_pub.publish(gps_parsed)
	local = to_lps(get_coords(gps))
	
	# Commenting this out so that we don't spam the output
	#rospy.loginfo(rospy.get_caller_id() + " Long: %f, Lat: %f --- X: %f, Y: %f", get_coords(gps).x, get_coords(gps).y, local.x, local.y)
	lps_pub.publish(local)


def orientation_callback(imu):
	global compass_pub
	
	explicit_quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
	roll, pitch, yaw = euler_from_quaternion(explicit_quat)
	
	yaw = math.degrees(yaw) % 360
	
	heading = Float32()
	heading.data = yaw
	compass_pub.publish(heading)


# =*=*=*=*=*=*=*=*=*=*=*=*= Services =*=*=*=*=*=*=*=*=*=*=*=*=

# Convert from gps to lps
def gps_to_lps_srv(req):
	res = ConvertPointResponse()
	res.pt = to_lps(req.pt)
	
	#rospy.loginfo(rospy.get_caller_id() + " Request: GPS (long: %.1f, lat: %.1f)", req.pt.x, req.pt.y)   
	#rospy.loginfo(rospy.get_caller_id() + " Response: LPS (x: %.f, y: %.f)", res.pt.x, res.pt.y)
	return res


# Convert from lps to gps
def lps_to_gps_srv(req):
	res = ConvertPointResponse()
	res.pt = to_gps(req.pt)
	
	#rospy.loginfo(rospy.get_caller_id() + " Request: LPS (x: %.f, y: %.f)", req.pt.x, req.pt.y)
	#rospy.loginfo(rospy.get_caller_id() + " Response: GPS (long: %.1f, lat: %.1f)", res.pt.x, res.pt.y)
	return res


# =*=*=*=*=*=*=*=*=*=*=*=*= Local =*=*=*=*=*=*=*=*=*=*=*=*=

# Convert from gps to lps
def to_lps(gps):
	global origin_lps
	local = Point()
	local.x = RADIUS * cosd(gps.y) * math.radians(gps.x) - origin_lps.x
	local.y = RADIUS * math.radians(gps.y) - origin_lps.y
	return local


# Convert from gps to lps
def to_gps(local):
	global origin_lps
	gps = Point()
	
	gps.y = math.degrees((local.y + origin_lps.y)/RADIUS)
	gps.x = math.degrees((local.x + origin_lps.x)/(RADIUS * cosd(gps.y)))
	return gps


# Extract longitude and latitude from boat_nav.msg.GPS
def get_coords(gps):
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
	
	# Setup first so that simulator can send the origin point
	srv1 = rospy.Service('lps_to_gps', ConvertPoint, lps_to_gps_srv)
	
	# setup the origin
	origin_coords = rospy.wait_for_message('gps_raw', GPS)
	origin_lps = to_lps(get_coords(origin_coords))
	rospy.loginfo("Got origin: x:%f y:%f", origin_lps.x, origin_lps.y)
	
	rospy.Subscriber('imu/data', Imu, orientation_callback)
	rospy.Subscriber('gps_raw', GPS, gps_callback)
	srv2 = rospy.Service('gps_to_lps', ConvertPoint, gps_to_lps_srv)
	rospy.spin()


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
