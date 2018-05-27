#!/usr/bin/env python
import threading
import math
import rospy
from abc import ABCMeta, abstractmethod
from boat_msgs.msg import BoatState, Point, PointArray, Waypoint, WaypointArray
from boat_msgs.srv import ConvertPoint
from std_msgs.msg import Float32

BUOY_TOL = rospy.get_param('/boat/planner/buoy_tol')

_boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
_waypoints_pub = rospy.Publisher('waypoints_raw', WaypointArray, queue_size=10)
_target_pub = rospy.Publisher('target_point', Waypoint, queue_size=10)


# Abstract base class for all Planners
# All implementations should be organized in the same order as this class
# That is:
#    - Class Variables
#    - Constructor
#    - Main Behaviour
#    - Callbacks
#    - Setters
#    - Utilities
#
# Implementations should also use the @overrides (@overrides.overrides) decorator
# to ensure proper inheritence on the setup() and planner() methods
#
class Planner:
	__metaclass__ = ABCMeta
	
	# Static variables shared in in all implementations
	state = BoatState()
	waypoints = []
	target_waypoint = Waypoint()
	cur_pos = Point()
	wind_coming = 0
	cur_boat_heading = 0
	ane_reading = 0
	
	def __init__(self):
		pass
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Main Behaviour =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	@abstractmethod
	def setup(self):
		NotImplementedError("Class %s doesn't implement setup()" % (self.__class__.__name__))
	
	
	@abstractmethod
	def planner(self):
		NotImplementedError("Class %s doesn't implement planner()" % (self.__class__.__name__))
	
	
	def traverse_waypoints_planner(self):
		
		waypoints = self.waypoints
		
		# If the list of waypoints is not empty 
		if len(waypoints) > 0:
		
			# If the boat is close enough to the waypoint, start navigating towards the next waypoint in the path
			if self.boat_reached_target():
				rospy.loginfo(rospy.get_caller_id() + " Reached intermediate waypoint (lat: %.2f, long: %.2f)", waypoints[0].pt.y, waypoints[0].pt.x)
			
				del waypoints[0]
				self.update_waypoints(waypoints)
				self.publish_target(waypoints[0])
		
		# If there are no waypoints left to navigate to, exit
		else:
			self.set_minor_state(BoatState.MIN_COMPLETE)
			rospy.loginfo(rospy.get_caller_id() + " No waypoints left. Boat State = 'Autonomous - Complete'")
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	@staticmethod
	def _anemometer_callback(anemometer):
		Planner.ane_reading = anemometer.data
		Planner._calc_wind_coming()
	
	
	@staticmethod
	def _compass_callback(compass):
		Planner.cur_boat_heading = compass.data
		Planner._calc_wind_coming()
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Setters =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	@staticmethod
	def set_minor_state(minor):
		Planner.state.minor = minor
		_boat_state_pub.publish(Planner.state)
	
	@staticmethod
	def clear_waypoints():
		Planner.update_waypoints([])
	
	@staticmethod
	def update_waypoints(new_pts):
		Planner.waypoints = new_pts
		_waypoints_pub.publish(Planner.waypoints)
	
	@staticmethod
	def publish_target(*argv):
		assert len(argv) is 0 or len(argv) is 1, "Invalid number of arguments, expected 1"
		
		if len(argv) is 1:
			Planner.target_waypoint = argv[0]
		
		waypoints = Planner.waypoints
		target_waypoint = Planner.target_waypoint
		cur_pos = Planner.cur_pos
		
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
			Planner.target_waypoint = Waypoint(roundPt, Waypoint.TYPE_ROUND)
		
		_target_pub.publish(Planner.target_waypoint)
		##rospy.loginfo(rospy.get_caller_id() + " New target waypoint: (long: %.2f, lat: %.2f) or (x: %.f, y: %.f)", point.x, point.y, local.x, local.y)
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Utility Functions =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	@staticmethod
	def _calc_wind_coming():
		Planner.new_wind_heading = (Planner.ane_reading + Planner.cur_boat_heading) % 360
		Planner.wind_coming = (Planner.new_wind_heading + 180) % 360
	
	
	def boat_reached_target(self):
		return self.is_within_dist(self.cur_pos, Services.to_lps(self.target_waypoint), BUOY_TOL)
	
	
	# Determine if the dist between two points is within the specified tolerance
	@staticmethod
	def is_within_dist(p1, p2, dist):
		a = math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2)
		return math.sqrt(a) < dist


# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Services =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

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


# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Initialize the class once =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
# TODO: Make this less jank
def initialize():
	rospy.wait_for_service('gps_to_lps')
	rospy.wait_for_service('lps_to_gps')
	Services._to_lps_srv = rospy.ServiceProxy('gps_to_lps', ConvertPoint)
	Services._to_gps_srv = rospy.ServiceProxy('lps_to_gps', ConvertPoint)
	
	rospy.Subscriber('anemometer', Float32, Planner._anemometer_callback)
	rospy.Subscriber('compass', Float32, Planner._compass_callback)

