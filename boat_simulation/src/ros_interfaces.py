#!/usr/bin/env python

import math
import threading

from std_msgs.msg import Float32, Int32, Bool
from sensor_msgs.msg import Imu
from boat_msgs.msg import BoatState, GPS, Point, PointArray, Waypoint, WaypointArray, Joy
from boat_msgs.srv import ConvertPoint

class ROSServices():
    def __init__(self, rospy):
        self.rospy = rospy
        self.to_gps_srv = rospy.ServiceProxy('lps_to_gps', ConvertPoint)
        self.to_lps_srv = rospy.ServiceProxy('gps_to_lps', ConvertPoint)
        
        self.to_gps_lock = threading.Lock()
        self.to_lps_lock = threading.Lock()

    def to_gps(self, p):
        return self.call_service(service=self.to_gps_srv,
                                 lock=self.to_gps_lock,
                                 p=p)
    def to_lps(self, p):
        return self.call_service(service=self.to_lps_srv,
                                 lock=self.to_lps_lock,
                                 p=p)

    def call_service(self, service, lock, p):
        with lock:
            if self.rospy.is_shutdown():
                return Point()
            if type(p) is Point:
                return service(p).pt
            elif type(p) is Waypoint:
                return service(p.pt).pt
            else:
                raise ValueError("p is of invalid type " + str(type(p)) +", must be either Point or Waypoint")

class ROSPublishers():
    def __init__(self, rospy):
        self.waypoint_pub = rospy.Publisher('waypoints_raw', WaypointArray, queue_size = 10)
        self.wind_pub = rospy.Publisher('anemometer', Float32, queue_size = 10)
        self.gps_pub = rospy.Publisher('gps_raw', GPS, queue_size = 10)
        self.orientation_pub = rospy.Publisher('imu/data', Imu, queue_size = 10)
        self.joy_pub = rospy.Publisher('joy', Joy, queue_size = 10)
        self.square_pub = rospy.Publisher('bounding_box', PointArray, queue_size = 10)
        self.search_area_pub = rospy.Publisher('search_area', PointArray, queue_size = 10)
        self.vision_pub = rospy.Publisher('vision', PointArray, queue_size = 10)

class ROSSubscribers():

    def __init__(self, rospy, database, all_data, ros_services):
        self.database = database
        self.all_data = all_data
        self.services = ros_services

        # Note: all variables that these change should be initialized
        # in the Sim Data object construction

        rospy.Subscriber('boat_state', BoatState, self.boat_state_callback)
        rospy.Subscriber('rudder', Float32, self.rudder_callback)
        rospy.Subscriber('winch', Int32, self.winch_callback)
        rospy.Subscriber('waypoints_raw', WaypointArray, self.waypoints_callback)
        rospy.Subscriber('target_heading', Float32, self.target_heading_callback)
        
        # So we can use real wind data in simulation mode
        # Data from this will override true wind in sim
        rospy.Subscriber('mock_true_wind', Float32, self.mock_true_wind_callback)
        
        # subscribers for replay mode
        rospy.Subscriber('lps', Point, self.lps_callback)
        rospy.Subscriber('compass', Float32, self.compass_callback)
        rospy.Subscriber('anemometer', Float32, self.anemometer_callback)
        rospy.Subscriber('rudder_pid/output', Float32, self.rudder_output_callback)
        rospy.Subscriber('rudder_pid/input', Float32, self.rudder_input_callback)
        rospy.Subscriber('rudder_pid/setpoint', Float32, self.rudder_setpoint_callback)
        rospy.Subscriber('rudder_pid/enable', Bool, self.rudder_enable_callback)
        rospy.Subscriber('gps_raw', GPS, self.gps_raw_callback)
        rospy.Subscriber('obstacles', PointArray, self.obstacles_callback)
        rospy.Subscriber('target_point', Waypoint, self.target_point_callback)
        rospy.Subscriber('bounding_box', PointArray, self.bounding_box_callback)
        rospy.Subscriber('search_area', PointArray, self.search_area_callback)
        rospy.Subscriber('vision', PointArray, self.vision_callback)
    
        # This is only used in a callback
        self.origin_override_pub = rospy.Publisher('origin_override', Point, queue_size = 10)

    def rudder_callback(self, msg):
        self.database["rudder_pos"] = msg.data

    ''' TODO: consider using a general callback for msgs with .data
    rospy.Subscriber('rudder', Float32, self.data_Type_callback, ("rudder_pos))
    def data_type_callback(self, data_msg, args):
        field = args[0]
        self.database[field] = data_msgs.data
    '''

    
    def boat_state_callback(self, newState):
        self.database["state"] = newState
        if self.database["state"].major is not BoatState.MAJ_DISABLED and pause:
            pause_sim()

    def winch_callback(self, msg):
        self.database["winch_pos"] = msg.data


    # Update the raw waypoints
    def waypoints_callback(self, newPoints):
        self.database["waypoint_gps"] = newPoints
        
        # Whenever the number of waypoints is decremented by one and we are in maximum meme state, smash becky
        ''' TODO
        if (len(waypoint_gps.points)-1) is len(newPoints.points) and sound and cur_boat_img is boat_imgs["mars"]:
            pygame.mixer.music.play()
        '''

    def target_heading_callback(self, angle):
        self.database["taret_heading"] = angle.data

    def lps_callback(self, lps):
        if self.all_data.mode_is_replay_or_controller():
            self.database["pos"] = lps

    def compass_callback(self, compass):
        if self.all_data.mode_is_replay_or_controller():
            self.database["heading"] = compass.data

    def anemometer_callback(self, anemometer):
        self.database["ane_reading"] = anemometer.data

    def mock_true_wind_callback(self, anemometer):
        self.database["wind_heading"] = anemometer.data

    def rudder_output_callback(self, float32):
        self.database["rudder_output"] = float32.data

    def rudder_input_callback(self, float32):
        self.database["rudder_input"] = float32.data

    def rudder_setpoint_callback(self, float32):
        self.database["rudder_setpoint"] = float32.data

    def rudder_enable_callback(self, enabled):
        self.database["rudder_enable"] = enabled.data

    def gps_raw_callback(self, gps):
        self.database["replay_gps_raw"] = gps
        
        if self.all_data.reset_origin_on_next_gps:
            self.all_data.reset_origin_on_next_gps = False
            new_origin = Point()
            new_origin.x = gps.longitude
            new_origin.y = gps.latitude
            self.origin_override_pub.publish(new_origin)
        
    def obstacles_callback(self, obstacles):
        self.database["obstacle_points"] = obstacles

    def bounding_box_callback(self, box):
        self.database["gps_bounding_box"] = box


    def search_area_callback(self, new_search_area):
        self.database["gps_search_area"] = new_search_area
        gps_search_area = self.database["gps_search_area"]

        if len(gps_search_area.points) >= 1:
            self.all_data.search_area.center = self.services.to_lps(gps_search_area.points[0])
            if len(gps_search_area.points) >= 2:
                edge_point = self.services.to_lps(gps_search_area.points[1])
                # first point is center and second defines radius from center
                # calc radius
                dx = (self.all_data.search_area.center.x - edge_point.x)
                dy = (self.all_data.search_area.center.y - edge_point.y)
                self.all_data.search_area.radius = math.sqrt(dx*dx+dy*dy)
            else:
                self.all_data.search_area.radius = 0
        else:
            self.all_data.search_area.center = None

    def target_point_callback(self, target_pt):
        self.database["target_point"] = target_pt

    def vision_callback(self, new_vision_points_gps):
        self.database["vision_points_gps"] = new_vision_points_gps
        
class ROSInterfaceManager():
    def __init__(self, rospy=None, all_data=None):
        self.services = ROSServices(rospy)
        self.subscribers = ROSSubscribers(rospy, database=all_data.ros_data, all_data=all_data, ros_services=self.services)
        self.publishers = ROSPublishers(rospy)
