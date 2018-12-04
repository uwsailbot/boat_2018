#!/usr/bin/env python
import rospy
import threading
import math
from abc import ABCMeta, abstractmethod
from boat_msgs.msg import BoatState, Point, Waypoint, WaypointArray
from boat_msgs.srv import ConvertPoint
from std_msgs.msg import Float32
from boat_utilities import angles, points, units

BUOY_TOL = rospy.get_param('/boat/planner/buoy_tol')
ROUND_DIST = rospy.get_param('/boat/planner/round_dist')
ROUND_FACTOR = rospy.get_param('/boat/planner/round_factor')

_boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
_waypoints_pub = rospy.Publisher('waypoints_raw', WaypointArray, queue_size=10)
_target_pub = rospy.Publisher('target_point', Waypoint, queue_size=10)


class Planner:
    """Abstract base class for all Planners.

	All implementations should be organized in the same order as this class
	That is:
		- Class Variables
		- Constructor
		- Main Behaviour
		- Callbacks
		- Setters
		- Utilities

	Implementations should also use the \@overrides (\@overrides.overrides) decorator
	to ensure proper inheritance on the abstract setup() and planner() methods

	Note that implementations must implement the setup() and planner() methods
	"""
    __metaclass__ = ABCMeta

    # Static variables shared in in all implementations
    state = BoatState()
    waypoints = []
    target_waypoint = Waypoint()
    cur_pos = Point()
    wind_coming = 0
    cur_boat_heading = 0
    ane_reading = 0

    # =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Main Behaviour =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

    @abstractmethod
    def setup(self):
        """Setup the planner for the specific challenge.

		There are three times when this is called:
			1. When the BoatState.major enters MAJ_AUTONOMOUS
			2. When the BoatState.challenge enters the specified implementation's challenge
			3. When the BoatState.minor is MIN_COMPLETE or MIN_INITIALIZE and a new waypoint is added to the waypoints_raw topic
		"""
        NotImplementedError("Class %s doesn't implement setup()" % (self.__class__.__name__))

    @abstractmethod
    def planner(self):
        """Compute the next point for the boat to navigate to.

		This is called whenever the boat's position is updated, and should compute the next
		target_waypoint for the boat to navigate towards
		"""
        NotImplementedError("Class %s doesn't implement planner()" % (self.__class__.__name__))

    def _traverse_waypoints_planner(self):
        """Navigate through the waypoints on the waypoints_raw topic sequentially."""
        waypoints = self.waypoints

        # If the list of waypoints is not empty
        if len(waypoints) > 0:

            # If the boat is close enough to the waypoint, start navigating towards the next waypoint in the path
            if self._boat_reached_target():
                rospy.loginfo(
                    rospy.get_caller_id() +
                    " Reached intermediate waypoint (lat: %.2f, long: %.2f)", waypoints[0].pt.y,
                    waypoints[0].pt.x)

                del waypoints[0]
                self._update_waypoints(waypoints)
                if len(waypoints) > 0:
                    self._publish_target(waypoints[0])

        # If there are no waypoints left to navigate to, exit
        else:
            self._set_minor_state(BoatState.MIN_COMPLETE)
            rospy.loginfo(rospy.get_caller_id() +
                          " No waypoints left. Boat State = 'Autonomous - Complete'")

    # =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

    @staticmethod
    def _anemometer_callback(anemometer):
        """Callback for anemometer reading."""
        Planner.ane_reading = anemometer.data
        Planner._calc_wind_coming()

    @staticmethod
    def _compass_callback(compass):
        """Callback for compass reading."""
        Planner.cur_boat_heading = compass.data
        Planner._calc_wind_coming()

    # =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Setters =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

    @staticmethod
    def _set_minor_state(minor):
        """Set the minor state of the BoatState."""
        Planner.state.minor = minor
        _boat_state_pub.publish(Planner.state)

    @staticmethod
    def _clear_waypoints():
        """Clear all the waypoints. Equivalent to _update_waypoints([])"""
        Planner._update_waypoints([])

    @staticmethod
    def _update_waypoints(new_pts):
        """Set and publish the specified waypoints."""
        Planner.waypoints = new_pts
        _waypoints_pub.publish(Planner.waypoints)

    @staticmethod
    def _publish_target(*argv):
        """Publish the target waypoint.

		Publish the target waypoint, performing additional computation if the waypoint is
		of type Waypoint.TYPE_ROUND to ensure the boat will properly round the waypoint.

		@param argv: Optionally specify the waypoint to publish. If unspecified, the Planner.target_waypoint will be used.
		"""
        assert len(argv) is 0 or len(argv) is 1, "Invalid number of arguments, expected 1"

        if len(argv) is 1:
            Planner.target_waypoint = argv[0]

        waypoints = Planner.waypoints
        target_waypoint = Planner.target_waypoint
        cur_pos = Planner.cur_pos

        # Perform the necessary computation to allow rounding of buoys
        if (target_waypoint.type is Waypoint.TYPE_ROUND and target_waypoint in waypoints
                and waypoints.index(target_waypoint) < len(waypoints) - 1):

            r = units.to_geo_coords(ROUND_DIST)
            k = ROUND_FACTOR

            # Use the heading from the boat to the buoy and from the buoy to the next to calculate where around the target to place the waypoint.
            next = waypoints[waypoints.index(target_waypoint) + 1]
            theta_boat = math.atan2(
                Services.to_gps(cur_pos).y - target_waypoint.pt.y,
                Services.to_gps(cur_pos).x - target_waypoint.pt.x)
            theta_next = math.atan2(next.pt.y - target_waypoint.pt.y,
                                    next.pt.x - target_waypoint.pt.x)
            d_theta = (theta_boat - theta_next + 4 * math.pi) % (2 * math.pi)
            angle = theta_next + k * (d_theta - math.pi) / 2
            if d_theta < math.pi:
                angle -= math.pi / 2
            else:
                angle += math.pi / 2

            roundPt = Point(target_waypoint.pt.x + math.cos(angle) * r,
                            target_waypoint.pt.y + math.sin(angle) * r)
            Planner.target_waypoint = Waypoint(roundPt, Waypoint.TYPE_ROUND)

        # Publish the target
        _target_pub.publish(Planner.target_waypoint)

        ##rospy.loginfo(rospy.get_caller_id() + " New target waypoint: (long: %.2f, lat: %.2f) or (x: %.f, y: %.f)", point.x, point.y, local.x, local.y)

    # =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Utility Functions =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

    @staticmethod
    def _calc_wind_coming():
        """Calculate the angle of the wind."""
        Planner.new_wind_heading = angles.normalize(Planner.ane_reading + Planner.cur_boat_heading)
        Planner.wind_coming = angles.opposite(Planner.new_wind_heading)

    def _boat_reached_target(self):
        """Determine if the boat is within BUOY_TOL meters of the target_waypoint.

		@return True if the boat is within the tolerance
		"""
        return points.is_within_dist(self.cur_pos, Services.to_lps(self.target_waypoint), BUOY_TOL)


# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Services =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=


class Services:
    _to_gps_srv = ()
    _to_lps_srv = ()
    _to_gps_lock = threading.Lock()
    _to_lps_lock = threading.Lock()

    @staticmethod
    def to_gps(p):
        """Convert a point from local (meters) position system to global (coords) position system.

		@param p: The Point or Waypoint to convert
		@return The converted Point
		"""
        with Services._to_gps_lock:
            if type(p) is Point:
                return Services._to_gps_srv(p).pt
            elif type(p) is Waypoint:
                return Services._to_gps_srv(p.pt).pt
            else:
                raise ValueError("p is of invalid type " + str(type(p)) +
                                 ", must be either Point or Waypoint")

    @staticmethod
    def to_lps(p):
        """Convert a point from global (coords) position system to local (meters) position system.

		@param p: The Point or Waypoint to convert
		@return The converted Point
		"""
        with Services._to_lps_lock:
            if type(p) is Point:
                return Services._to_lps_srv(p).pt
            elif type(p) is Waypoint:
                return Services._to_lps_srv(p.pt).pt
            else:
                raise ValueError("p is of invalid type " + str(type(p)) +
                                 ", must be either Point or Waypoint")


# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Initialize the module =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=

# TODO: Make this less jank and move Services to a new module (Shared on all ros packages?)
rospy.wait_for_service('gps_to_lps')
rospy.wait_for_service('lps_to_gps')
Services._to_lps_srv = rospy.ServiceProxy('gps_to_lps', ConvertPoint)
Services._to_gps_srv = rospy.ServiceProxy('lps_to_gps', ConvertPoint)

rospy.Subscriber('anemometer', Float32, Planner._anemometer_callback)
rospy.Subscriber('compass', Float32, Planner._compass_callback)
