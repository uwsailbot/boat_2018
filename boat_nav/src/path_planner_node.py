#!/usr/bin/env python
import math
import rospy
from boat_msgs.msg import BoatState, Point, PointArray
from boat_msgs.srv import ConvertPoint

# Declare global variables needed for the node
state = BoatState()
waypoints = []
rate = 0
ane_reading = 0
buoy_tol = rospy.get_param('/boat/planner/buoy_tol')
height_to_travel = rospy.get_param('/boat/planner/station/height')
max_width = rospy.get_param('/boat/planner/station/width')
box = []
cur_pos = Point()
start_station = Point()
end_station = Point()
final_buoy_station = False
station_setup = False
not_within_box = False

# Declare the publishers for the node
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
waypoints_pub = rospy.Publisher('waypoints_raw', PointArray, queue_size=10)
target_pub = rospy.Publisher('target_point', Point, queue_size=10)

def boat_state_callback(new_state):
	global state
	global waypoints

	# For if bounding box is already present when we switch to station mode
	if new_state.challenge is BoatState.CHA_STATION and state.challenge is not BoatState.CHA_STATION and len(box) is 4:
		rospy.Timer(rospy.Duration(5*60), station_timer_callback, oneshot=True)
		print rospy.Time.now().secs
		station_setup()

	state = new_state
	
	# Move to planning state if there is a pending waypoint
	if state.major is BoatState.MAJ_AUTONOMOUS and state.minor is BoatState.MIN_COMPLETE and len(waypoints)>0:
		state.minor = BoatState.MIN_PLANNING
		boat_state_pub.publish(state)
		
	# if station, load points from list and publish them

def anemometer_callback(anemometer):
	global ane_reading
	ane_reading = anemometer.data
	
def bounding_box_callback(bounding_box):
	global box
	# Reorganize the local points to create a box when drawn, if there are four
	if len(bounding_box.points) == 4:
		x_sum = 0
		y_sum = 0
		for p in bounding_box.points:
			x_sum += p.x
			y_sum += p.y
		x_avr = x_sum / 4.0
		y_avr = y_sum / 4.0
		temp = [None]*4
		for p in bounding_box.points:
			if p is None:
				return
			if p.x < x_avr:
				if p.y < y_avr:
					temp[0] = p
				else: 
					temp[1] = p
			else:
				if p.y < y_avr:
					temp[3] = p
				else:
					temp[2] = p
		box = temp
	else:
		box = bounding_box.points

	# For if bounding box is added after we are in station mode
	if state.challenge is BoatState.CHA_STATION and len(box) is 4:
		rospy.Timer(rospy.Duration(5*60), station_timer_callback, oneshot=True)
		station_setup()

def station_timer_callback(event):
	global waypoints
	global final_buoy_station
	waypoints = []
	final_buoy_station = True
	rospy.loginfo(rospy.get_caller_id() + " Reached end of station timer")

	# Determine distance from each side of the box 
	l_dist = abs(dist_from_line(box[0], box[1]))
	t_dist = abs(dist_from_line(box[1], box[2]))
	r_dist = abs(dist_from_line(box[3], box[2]))
	b_dist = abs(dist_from_line(box[0], box[3]))

	dist_list = (l_dist, t_dist, r_dist, b_dist)
	i = dist_list.index(min(dist_list))
	
	# Use service to determine 100m dist in gps
	# Use this distance on how far to put the buoy outside the box
	tol_dist = lps_to_gps(Point(20,0)).pt.x
	cur_pos_gps = lps_to_gps(cur_pos).pt
	
	final_point = {0 : Point(box[0].x - tol_dist, cur_pos_gps.y),
				1 : Point(cur_pos_gps.x, box[1].y + tol_dist),
				2 : Point(box[3].x + tol_dist, cur_pos_gps.y),
				3 : Point(cur_pos_gps.x, box[0].y - tol_dist)
	}
	final_direction = {0 : "Left", 1: "Top", 2: "Right", 3: "Bottom"}
	rospy.loginfo(rospy.get_caller_id() + " Exiting bounding box through: " + final_direction[i])
	waypoints.append(final_point[i])
	waypoints_pub.publish(waypoints)
				
	
def dist_from_line(start_point, end_point):
	cur_pos_gps = lps_to_gps(cur_pos).pt
	if (end_point.x - start_point.x) <= 0.0001:
		return cur_pos_gps.x - end_point.x
	m = (end_point.y - start_point.y)/(end_point.x - start_point.x)
	b = start_point.y - m * start_point.x
	return (b + m * cur_pos_gps.x - cur_pos_gps.y)/(math.sqrt(1 + m*m))
	
def waypoints_callback(new_waypoint):
	global waypoints
	
	waypoints = new_waypoint.points
	if(len(waypoints) > 0):
		publish_target(waypoints[0])
	
	# If we are waiting in autonomous-complete, and a new waypoint is added, move to planning state
	if state.major is BoatState.MAJ_AUTONOMOUS and state.minor is BoatState.MIN_COMPLETE and len(waypoints)>0:
		state.minor = BoatState.MIN_PLANNING
		boat_state_pub.publish(state)

def update_waypoints():
	global waypoints
	global final_buoy_station
	if state.challenge is BoatState.CHA_STATION and not final_buoy_station:
		station_waypoint_planner()
	# If we just hit the last station keeping point, the challenge is now complete so reset to default
	elif state.challenge is BoatState.CHA_STATION and final_buoy_station:
		state.challenge = BoatState.CHA_NAV
		boat_state_pub.publish(state)
		final_buoy_station = False
		station_setup = False

	del waypoints[0]
	waypoints_pub.publish(waypoints)

def station_setup():
	global start_station
	global end_station
	global waypoints
	rospy.loginfo(rospy.get_caller_id() + " Beginning station challenge path planner routine")

	# Clear previous points
	waypoints = []
	waypoints_pub.publish(waypoints)
	target_pub.publish(Point())

	# Determine wall angles and box widths
	station_angle = math.atan2(box[3].y - box[0].y, box[3].x - box[0].x) * 180.0 / math.pi
	bottom_box_width = math.hypot(box[3].y - box[0].y, box[3].x - box[0].x)
	top_box_width = math.hypot(box[2].y - box[1].y, box[2].x - box[1].x)
	left_box_height = math.hypot(box[1].y - box[0].y, box[1].x - box[0].x)
	right_box_height = math.hypot(box[3].y - box[2].y, box[3].x - box[2].x)

	# Determine width at the height we will be travelling
	station_width = (bottom_box_width + (top_box_width-bottom_box_width) * height_to_travel) * max_width
	station_height = (left_box_height + (right_box_height-left_box_height) * ((1 - max_width) / 2.0)) * height_to_travel
	
	# Determine the two points that will alternate
	start_x = ((box[3].x + box[0].x) / 2.0) - station_width / 2.0
	y_int = box[0].y - math.tan(math.radians(station_angle)) * box[0].x
	start_y = math.tan(math.radians(station_angle)) * start_x + y_int + station_height
	start_station = Point(start_x, start_y)
	end_x = start_x + station_width
	end_y = math.tan(math.radians(station_angle)) * end_x + y_int + station_height
	end_station = Point(end_x, end_y)
	
	waypoints.append(start_station)
	waypoints_pub.publish(waypoints)
	
def station_waypoint_planner():
	global waypoints
	if not_within_box:
		x_sum = 0
		y_sum = 0
		for p in box:
			x_sum += p.x
			y_sum += p.y
		x_avr = x_sum / 4.0
		y_avr = y_sum / 4.0
		waypoints.append(Point(x_avr, y_avr))
		return
	diff_point = Point()
	diff_point.x = waypoints[0].x - start_station.x
	diff_point.y = waypoints[0].y - start_station.y
	if abs(diff_point.x) < 0.00001 and abs(diff_point.y) < 0.00001:
		waypoints.append(end_station)
	else:
		waypoints.append(start_station)

def publish_target(point):
	local = gps_to_lps(point).pt
	target_pub.publish(local)
	##rospy.loginfo(rospy.get_caller_id() + " New target waypoint: (long: %.2f, lat: %.2f) or (x: %.f, y: %.f)", point.x, point.y, local.x, local.y)

def position_callback(position):
	global waypoints
	global rate
	global cur_pos
	global not_within_box
	cur_pos = position
	rate = rospy.Rate(100)
	
	# If the boat isn't in the autonomous planning state, exit
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	# If the list of waypoints is not empty 
	if(len(waypoints) > 0):
		# If the boat is close enough to the waypoint...
		if is_within_dist(position, gps_to_lps(waypoints[0]).pt, buoy_tol):
			rospy.loginfo(rospy.get_caller_id() + " Reached intermediate waypoint (lat: %.2f, long: %.2f)", waypoints[0].y, waypoints[0].x)
			update_waypoints()
	
	# If there are no waypoints left to navigate to, exit	
	elif state.minor != BoatState.MIN_COMPLETE:
		state.minor = BoatState.MIN_COMPLETE
		boat_state_pub.publish(state)
		rospy.loginfo(rospy.get_caller_id() + " No waypoints left. Boat State = 'Autonomous - Complete'")

	# If in station keeping mode and outside of desired box, cycle to next waypoint
	if state.challenge is BoatState.CHA_STATION and len(box) is 4 and not final_buoy_station:
		if not within_box():
			not_within_box = True
			update_waypoints()
		else:
			not_within_box = False
	
	# Adjust the sleep to suit the node
	rate.sleep()

def within_box():
	l_dist = dist_from_line(box[0], box[1])
	t_dist = dist_from_line(box[1], box[2])
	r_dist = dist_from_line(box[3], box[2])
	b_dist = dist_from_line(box[0], box[3])
	
	# Determine wall angles and box widths
	
	bottom_box_width = math.hypot(box[3].y - box[0].y, box[3].x - box[0].x)
	top_box_width = math.hypot(box[2].y - box[1].y, box[2].x - box[1].x)
	left_box_height = math.hypot(box[1].y - box[0].y, box[1].x - box[0].x)
	right_box_height = math.hypot(box[3].y - box[2].y, box[3].x - box[2].x)
	width_tol = (bottom_box_width + (top_box_width-bottom_box_width) * height_to_travel) * 0.1
	height_tol =  (left_box_height + (right_box_height-left_box_height) * (1 - max_width) / 2.0) * 0.1
	
	if l_dist >= width_tol and r_dist <= -1 * width_tol and t_dist >= height_tol and b_dist <= -1 * height_tol:
		return True
	else:
		print "Not within inner box."
		return False
	

# Determine if the dist between two points is within the specified tolerance
def is_within_dist(p1, p2, dist):
	a = math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2)
	return math.sqrt(a) < dist


# Initialize the node
def initialize():
	rospy.init_node('path_planner')
	
	global gps_to_lps
	global lps_to_gps
	rospy.wait_for_service('gps_to_lps')
	rospy.wait_for_service('lps_to_gps')
	gps_to_lps = rospy.ServiceProxy('gps_to_lps', ConvertPoint)
	lps_to_gps = rospy.ServiceProxy('lps_to_gps', ConvertPoint)
	
	# If the filters work, change lps to use /odometry/filtered
	rospy.Subscriber('lps', Point, position_callback)
	rospy.Subscriber('waypoints_raw', PointArray, waypoints_callback)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('bounding_box', PointArray, bounding_box_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		initialize()
	except rospy.ROSInterruptException:
		pass

