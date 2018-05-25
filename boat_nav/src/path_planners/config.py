#!/usr/bin/env python
import threading
import math
import rospy
from boat_msgs.msg import BoatState, Point, PointArray, Waypoint, WaypointArray
from boat_msgs.srv import ConvertPoint
from std_msgs.msg import Float32

BUOY_TOL = rospy.get_param('/boat/planner/buoy_tol')

_boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
_waypoints_pub = rospy.Publisher('waypoints_raw', WaypointArray, queue_size=10)
_target_pub = rospy.Publisher('target_point', Waypoint, queue_size=10)

# Declare global variables needed for the node
class Globals:
	state = BoatState()
	waypoints = []
	target_waypoint = Waypoint()
	
	@staticmethod
	def set_minor_state(minor):
		Globals.state.minor = minor
		_boat_state_pub.publish(Globals.state)
	
	@staticmethod
	def clear_waypoints():
		Globals.update_waypoints([])
	
	@staticmethod
	def update_waypoints(new_pts):
		Globals.waypoints = new_pts
		_waypoints_pub.publish(Globals.waypoints)
	
	@staticmethod
	def publish_target():
		
		waypoints = Globals.waypoints
		target_waypoint = Globals.target_waypoint
		cur_pos = Data.cur_pos
		
		if target_waypoint.type is Waypoint.TYPE_ROUND and target_waypoint in waypoints and waypoints.index(target_waypoint) < len(waypoints)-1:
			
			r = 3/111319.492188 # meters to coords
			k = 1.5
			
			next = waypoints[waypoints.index(target_waypoint)+1]
			theta_boat = math.atan2(Services.to_gps(cur_pos).y - target_waypoint.pt.y, Services.to_gps(cur_pos).x - target_waypoint.pt.x)
			theta_next = math.atan2(next.pt.y - target_waypoint.pt.y, next.pt.x - target_waypoint.pt.x)
			d_theta = (theta_boat - theta_next + 4 * math.pi) % (2 * math.pi)
			angle = theta_next + k*(d_theta - math.pi) / 2
			if d_theta < math.pi:
				angle -= math.pi / 2
			else:
				angle += math.pi / 2
			
			roundPt = Point(target_waypoint.pt.x + math.cos(angle)*r, target_waypoint.pt.y + math.sin(angle)*r)
			Globals.target_waypoint = Waypoint(roundPt, Waypoint.TYPE_ROUND)
		
		_target_pub.publish(Globals.target_waypoint)
		##rospy.loginfo(rospy.get_caller_id() + " New target waypoint: (long: %.2f, lat: %.2f) or (x: %.f, y: %.f)", point.x, point.y, local.x, local.y)
	

# Declare boat-related variables
class Data:
	cur_pos = Point()
	wind_coming = 0
	cur_boat_heading = 0
	ane_reading = 0

# Declare the services for the node
class Services:
	_to_gps_srv = ()
	_to_lps_srv = ()
	_to_gps_lock = threading.Lock()
	_to_lps_lock = threading.Lock()
	
	@staticmethod
	def to_gps(p):
		with Services._to_gps_lock:
			if type(p) is Point:
				return Services._to_gps_srv(p).pt
			elif type(p) is Waypoint:
				return Services._to_gps_srv(p.pt).pt
			else:
				raise ValueError("p is of invalid type " + str(type(p)) +", must be either Point or Waypoint")
	
	@staticmethod
	def to_lps(p):
		with Services._to_lps_lock:
			if type(p) is Point:
				return Services._to_lps_srv(p).pt
			elif type(p) is Waypoint:
				return Services._to_lps_srv(p.pt).pt
			else:
				raise ValueError("p is of invalid type " + str(type(p)) +", must be either Point or Waypoint")


# =*=*=*=*=*=*=*=*=*=*=*=*= Math Helper Funcs =*=*=*=*=*=*=*=*=*=*=*=*=

def boat_reached_target():
	return is_within_dist(Data.cur_pos, Services.to_lps(Globals.target_waypoint), BUOY_TOL)


# Determine if the dist between two points is within the specified tolerance
def is_within_dist(p1, p2, dist):
	a = math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2)
	return math.sqrt(a) < dist


# =*=*=*=*=*=*=*=*=*=*=*=*= Private =*=*=*=*=*=*=*=*=*=*=*=*=

def _anemometer_callback(anemometer):
	Data.ane_reading = anemometer.data
	_calc_wind_coming()


def _compass_callback(compass):
	Data.cur_boat_heading = compass.data
	_calc_wind_coming()


def _calc_wind_coming():
	Data.new_wind_heading = (Data.ane_reading + Data.cur_boat_heading) % 360
	Data.wind_coming = (Data.new_wind_heading + 180) % 360


def initialize():
	
	rospy.wait_for_service('gps_to_lps')
	rospy.wait_for_service('lps_to_gps')
	Services._to_lps_srv = rospy.ServiceProxy('gps_to_lps', ConvertPoint)
	Services._to_gps_srv = rospy.ServiceProxy('lps_to_gps', ConvertPoint)
	
	rospy.Subscriber('anemometer', Float32, _anemometer_callback)
	rospy.Subscriber('compass', Float32, _compass_callback)


