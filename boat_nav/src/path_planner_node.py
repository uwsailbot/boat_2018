#!/usr/bin/env python
import math
import rospy
from boat_msgs.msg import BoatState, Point, PointArray, Waypoint, WaypointArray
from boat_msgs.srv import ConvertPoint
from std_msgs.msg import Float32

# Declare global variables needed for the node
state = BoatState()
waypoints = []
target_waypoint = Waypoint()
rate = 0
ane_reading = 0
buoy_tol = rospy.get_param('/boat/planner/buoy_tol')
height_to_travel = rospy.get_param('/boat/planner/station/height')
max_width = rospy.get_param('/boat/planner/station/width')
inner_box_scale = rospy.get_param('/boat/planner/station/inner_box_scale')
box = []
cur_pos = Point()
start_station = Point()
end_station = Point()
station_timeout = False
wind_coming = 0
cur_boat_heading = 0

# Declare the publishers for the node
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
waypoints_pub = rospy.Publisher('waypoints_raw', WaypointArray, queue_size=10)
target_pub = rospy.Publisher('target_point', Waypoint, queue_size=10)

def publish_target():
	global target_waypoint
	
	if target_waypoint.type is Waypoint.TYPE_ROUND and target_waypoint in waypoints and waypoints.index(target_waypoint) < len(waypoints)-1:
		
		r = 5/111319.492188 # meters to coords
		k = 1.5
		
		next = waypoints[waypoints.index(target_waypoint)+1]
		print "cur  ", target_waypoint.pt
		print "next ", next.pt
		print "boat ", lps_to_gps(cur_pos).pt
		
		theta_boat = math.atan2(lps_to_gps(cur_pos).pt.y - target_waypoint.pt.y, lps_to_gps(cur_pos).pt.x - target_waypoint.pt.x)
		theta_next = math.atan2(next.pt.y - target_waypoint.pt.y, next.pt.x - target_waypoint.pt.x)
		
		print "theta_boat", theta_boat
		print "theta_next", theta_next
		
		d_theta = (theta_boat - theta_next + 4 * math.pi) % (2 * math.pi)
		angle = theta_next + k*(d_theta - math.pi) / 2
		if d_theta < math.pi:
			angle -= math.pi / 2
		else:
			angle += math.pi / 2
		
		print angle/math.pi*180
		roundPt = Point(target_waypoint.pt.x + math.cos(angle)*r, target_waypoint.pt.y + math.sin(angle)*r)
		target_waypoint = Waypoint(roundPt, Waypoint.TYPE_ROUND)
	
	target_lps = Waypoint(gps_to_lps(target_waypoint.pt).pt, target_waypoint.type)
	target_pub.publish(target_lps)
	##rospy.loginfo(rospy.get_caller_id() + " New target waypoint: (long: %.2f, lat: %.2f) or (x: %.f, y: %.f)", point.x, point.y, local.x, local.y)


def boat_state_callback(new_state):
	global state
	global waypoints

	prev_state = state
	state = new_state
	
	# For if bounding box is already present when we switch to station mode
	if ((state.challenge is BoatState.CHA_STATION and prev_state.challenge is not BoatState.CHA_STATION) or\
		(state.major is BoatState.MAJ_AUTONOMOUS and prev_state.major is not BoatState.MAJ_AUTONOMOUS)) and len(box) is 4:
		rospy.Timer(rospy.Duration(5*60), station_timer_callback, oneshot=True)
		station_setup()
	
	# Move to planning state if there is a pending waypoint
	if state.major is BoatState.MAJ_AUTONOMOUS and state.minor is BoatState.MIN_COMPLETE and len(waypoints)>0:
		waypoints_callback(WaypointArray(waypoints))
		state.minor = BoatState.MIN_PLANNING
		boat_state_pub.publish(state)
		
	
	#TODO: if station, load points from list and publish them

def anemometer_callback(anemometer):
	global ane_reading
	ane_reading = anemometer.data
	calc_wind_coming()

def compass_callback(compass):
	global cur_boat_heading
	cur_boat_heading = compass.data
	calc_wind_coming()
	
def calc_wind_coming():
	global wind_coming
	new_wind_heading = (ane_reading + cur_boat_heading) % 360
	wind_coming = (new_wind_heading + 180) % 360
	
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
			if wind_coming > 45 and wind_coming <= 135:
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
			elif wind_coming > 135 and wind_coming <= 225:
				if p.x < x_avr:
					if p.y < y_avr:
						temp[1] = p
					else: 
						temp[2] = p
				else:
					if p.y < y_avr:
						temp[0] = p
					else:
						temp[3] = p
			elif wind_coming > 225 and wind_coming <= 315:
				if p.x < x_avr:
					if p.y < y_avr:
						temp[2] = p
					else: 
						temp[3] = p
				else:
					if p.y < y_avr:
						temp[1] = p
					else:
						temp[0] = p
			else:
				if p.x < x_avr:
					if p.y < y_avr:
						temp[3] = p
					else: 
						temp[0] = p
				else:
					if p.y < y_avr:
						temp[2] = p
					else:
						temp[1] = p
			box = temp
	else:
		box = bounding_box.points

	# For if bounding box is added after we are in station mode
	if state.challenge is BoatState.CHA_STATION and len(box) is 4:
		rospy.Timer(rospy.Duration(5*60), station_timer_callback, oneshot=True)
		station_setup()


def waypoints_callback(new_waypoint):
	global waypoints
	global target_waypoint
	waypoints = new_waypoint.points
	
	if state.major is not BoatState.MAJ_AUTONOMOUS:
		return
	
	if state.challenge is BoatState.CHA_NAV or state.challenge is BoatState.CHA_LONG:
		if len(waypoints) > 0:
			target_waypoint = waypoints[0]
			publish_target()
		
	elif state.challenge is BoatState.CHA_STATION:
		return
	
	# If we are waiting in autonomous-complete, and a new waypoint is added, move to planning state
	if state.minor is BoatState.MIN_COMPLETE and len(waypoints)>0:
		state.minor = BoatState.MIN_PLANNING
		boat_state_pub.publish(state)
		
def position_callback(position):
	global waypoints
	global rate
	global cur_pos
	cur_pos = position
	rate = rospy.Rate(100)
	
	# If the boat isn't in the autonomous planning state, exit
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	# Navigation and long-distance challenge
	if state.challenge is BoatState.CHA_NAV or state.challenge is BoatState.CHA_LONG:
		traverse_waypoints_planner()
	
	# Station keeping challenge
	elif state.challenge is BoatState.CHA_STATION:
		station_keeper_planner()
	
	# Adjust the sleep to suit the node
	rate.sleep()


# =*=*=*=*=*=*=*=*=*=*=*=*= Challenge-Specific Funcs =*=*=*=*=*=*=*=*=*=*=*=*=

def station_setup():
	global state
	global start_station
	global end_station
	global waypoints
	global target_waypoint
	rospy.loginfo(rospy.get_caller_id() + " Beginning station challenge path planner routine")

	# Clear previous points
	waypoints = []
	waypoints_pub.publish(waypoints)
	target_waypoint = Waypoint()
	publish_target()

	# Determine wall angles and box widths, as well as slope of the bottom of the box, as long as it's not vertical
	m_bottom = 0
	vert = False
	if abs(box[3].x - box[0].x) > 0.0001:
		m_bottom = (box[3].y - box[0].y)/(box[3].x - box[0].x)
	else:
		vert = True
	bottom_box_width = math.hypot(box[3].y - box[0].y, box[3].x - box[0].x)
	top_box_width = math.hypot(box[2].y - box[1].y, box[2].x - box[1].x)
	left_box_height = math.hypot(box[1].y - box[0].y, box[1].x - box[0].x)
	right_box_height = math.hypot(box[3].y - box[2].y, box[3].x - box[2].x)

	# Determine width at the height we will be traveling
	station_width = (bottom_box_width + (top_box_width-bottom_box_width) * height_to_travel) * max_width
	station_height = (left_box_height + (right_box_height-left_box_height) * ((1 - max_width) / 2.0)) * height_to_travel
	
	# Determine the two points that will alternate, first find centre point of box
	y_sum = 0
	x_sum = 0
	for p in box:
		x_sum += p.x
		y_sum += p.y
	x_avr = x_sum / 4.0
	y_avr = y_sum / 4.0
	
	y_int = box[0].y - m_bottom * box[0].x
	start_station = Point()
	end_station = Point()

	# Depending on if bottom of box (rel to wind) is on right/left/up/down side, find a parallel line to the bottom
	# Then create two points on that line at the desired height and width, and move them inside the box
	if box[0].x > x_avr and box[3].x > x_avr:
		start_station.y = ((box[3].y + box[0].y) / 2.0) - station_width / 2.0
		end_station.y = start_station.y + station_width
		if not vert:
			start_station.x = (start_station.y - y_int)/m_bottom - station_height
			end_station.x = (end_station.y - y_int)/m_bottom - station_height
		else:
			start_station.x = box[3].x - station_height
			end_station.x = box[3].x - station_height
	elif box[0].x < x_avr and box[3].x < x_avr:
		start_station.y = ((box[3].y + box[0].y) / 2.0) - station_width / 2.0
		end_station.y = start_station.y + station_width
		if not vert:
			start_station.x = (start_station.y - y_int)/m_bottom + station_height
			end_station.x = (end_station.y - y_int)/m_bottom + station_height
		else:
			start_station.x = box[3].x + station_height
			end_station.x = box[3].x + station_height
	elif box[0].y > y_avr and box[3].y > y_avr:
		start_station.x = ((box[3].x + box[0].x) / 2.0) - station_width / 2.0
		start_station.y = m_bottom * start_station.x + y_int - station_height
		end_station.x = start_station.x + station_width
		end_station.y = m_bottom * end_station.x + y_int - station_height
	else:
		start_station.x = ((box[3].x + box[0].x) / 2.0) - station_width / 2.0
		start_station.y = m_bottom * start_station.x + y_int + station_height
		end_station.x = start_station.x + station_width
		end_station.y = m_bottom * end_station.x + y_int + station_height

	target_waypoint = Waypoint(start_station, Waypoint.TYPE_INTERSECT)
	publish_target()
	
	state.minor = BoatState.MIN_PLANNING
	boat_state_pub.publish(state)

def station_timer_callback(event):
	global station_timeout
	global target_waypoint
	station_timeout = True
	rospy.loginfo(rospy.get_caller_id() + " Reached end of station timer")

	# Determine distance from each side of the box 
	l_dist = dist_from_line(box[0], box[1])
	t_dist = dist_from_line(box[1], box[2])
	r_dist = dist_from_line(box[3], box[2])
	b_dist = dist_from_line(box[0], box[3])

	dist_list = (l_dist, t_dist, r_dist, b_dist)
	i = dist_list.index(min(dist_list))
	
	# Use service to determine 12m dist in gps
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
	
	target_waypoint = Waypoint(final_point[i], Waypoint.TYPE_INTERSECT)
	publish_target()

def station_keeper_planner():
	global target_waypoint
	global station_timeout
	
	
		# If time is up and we've fully exited the box, stop
	if station_timeout:
		if is_within_dist(cur_pos, gps_to_lps(target_waypoint.pt).pt, buoy_tol):
			state.minor = BoatState.MIN_COMPLETE
			boat_state_pub.publish(state)
			station_timeout = False
			rospy.loginfo(rospy.get_caller_id() + " Exited box. Boat State = 'Autonomous - Complete'")
		return
	
	# If the box is ever invalid, stop
	if len(box) is not 4 and state.minor is not BoatState.MIN_COMPLETE:
		state.minor = BoatState.MIN_COMPLETE
		boat_state_pub.publish(state)
		rospy.loginfo(rospy.get_caller_id() + " Box is invalid. Boat State = 'Autonomous - Complete'")
		return
	
	#TODO: Make this not spam (Eg. if we're already headed for the center, don't re-publish
	if not within_box():
		x_avr = 0
		y_avr = 0
		for p in box:
			x_avr += p.x / float(len(box))
			y_avr += p.y / float(len(box))
		target_waypoint = Waypoint(Point(x_avr, y_avr), Waypoint.TYPE_INTERSECT)
		publish_target()
		return
	
	# If we've reached the set waypoint, flip around
	if is_within_dist(cur_pos, gps_to_lps(target_waypoint.pt).pt, buoy_tol):
	
		if is_within_dist(target_waypoint.pt, start_station, 0.0001):
			target_waypoint = Waypoint(end_station, Waypoint.TYPE_INTERSECT)
		else:
			target_waypoint = Waypoint(start_station, Waypoint.TYPE_INTERSECT)
		publish_target()


def traverse_waypoints_planner():
	global waypoints
	global target_waypoint
	
	# If the list of waypoints is not empty 
	if(len(waypoints) > 0):
	
		# If the boat is close enough to the waypoint, start navigating towards the next waypoint in the path
		if is_within_dist(cur_pos, gps_to_lps(target_waypoint.pt).pt, buoy_tol):
			rospy.loginfo(rospy.get_caller_id() + " Reached intermediate waypoint (lat: %.2f, long: %.2f)", waypoints[0].pt.y, waypoints[0].pt.x)
			
			if state.challenge is BoatState.CHA_LONG:
				waypoints.append(waypoints[0])
			del waypoints[0]
			waypoints_pub.publish(waypoints)
	
	# If there are no waypoints left to navigate to, exit
	else:
		state.minor = BoatState.MIN_COMPLETE
		boat_state_pub.publish(state)
		rospy.loginfo(rospy.get_caller_id() + " No waypoints left. Boat State = 'Autonomous - Complete'")


# =*=*=*=*=*=*=*=*=*=*=*=*= Math Helper Funcs =*=*=*=*=*=*=*=*=*=*=*=*=

def within_box():
	# Find centre of box
	y_sum = 0
	x_sum = 0
	for p in box:
		x_sum += p.x
		y_sum += p.y
	x_avr = x_sum / 4.0
	y_avr = y_sum / 4.0
	inner_box = [None]*4
	# Construct the inner box, using scale parameter from yaml
	for i in range(4):
		pt = Point()
		pt.y = (box[i].y - y_avr)*inner_box_scale + y_avr
		pt.x = (box[i].x - x_avr)*inner_box_scale + x_avr
		inner_box[i] = pt

	# Create horizontal vector from the cur_pos towards +x
	start_point = lps_to_gps(cur_pos).pt
	m = 0
	b = start_point.y
	intersections = 0

	# Cast the vector out of the box, if the boat is inside the box, the ray will only intersect with a side once.  If outside of the box, it 
	# will intersect an odd number of times.
	for i in range(4):
		# Make sure line is not vertical
		if abs(inner_box[(i+1)%4].x - inner_box[i].x) > 0.0001:
			m_box = (inner_box[(i+1)%4].y - inner_box[i].y)/(inner_box[(i+1)%4].x - inner_box[i].x)
			b_box = inner_box[i].y - m_box * inner_box[i].x
			
			# If the slopes are the same they will never intersect
			if abs(m - m_box) > 0.0001:
				x_int = (b_box - b)/(m_box - m)
				y_int = m_box * x_int + b_box
				# Make sure the intersection occurs within the bounds of the box, since lines continue infinitely and the intersection
				# might be out of bound
				if (x_int <= max(inner_box[i].x,inner_box[(i+1)%4].x) and x_int >= min(inner_box[i].x,inner_box[(i+1)%4].x) and x_int >= start_point.x
					and y_int <= max(inner_box[i].y,inner_box[(i+1)%4].y) and y_int >= min(inner_box[i].y,inner_box[(i+1)%4].y)):
					intersections += 1
		# Perfectly vertical line, will intersect as long as it is on the right side of the starting point and in the right y range
		else:
			if (inner_box[i].x >= start_point.x and start_point.y <= max(inner_box[i].y,inner_box[(i+1)%4].y) 
				and start_point.y >= min(inner_box[i].y,inner_box[(i+1)%4].y)):
				intersections += 1
			
	
	if (intersections % 2) is not 0:
		return True
	else:
		print "Not within inner box."
		return False
	

# Determine if the dist between two points is within the specified tolerance
def is_within_dist(p1, p2, dist):
	a = math.pow(p1.x - p2.x, 2) + math.pow(p1.y - p2.y, 2)
	return math.sqrt(a) < dist


def dist_from_line(start_point, end_point):
	cur_pos_gps = lps_to_gps(cur_pos).pt
	if (end_point.x - start_point.x) <= 0.0001:
		return cur_pos_gps.x - end_point.x
	m = (end_point.y - start_point.y)/(end_point.x - start_point.x)
	b = start_point.y - m * start_point.x
	return abs(b + m * cur_pos_gps.x - cur_pos_gps.y)/(math.sqrt(1 + m*m))


# =*=*=*=*=*=*=*=*=*=*=*=*= Initialization =*=*=*=*=*=*=*=*=*=*=*=*=

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
	rospy.Subscriber('waypoints_raw', WaypointArray, waypoints_callback)
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('bounding_box', PointArray, bounding_box_callback)
	rospy.Subscriber('compass', Float32, compass_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		initialize()
	except rospy.ROSInterruptException:
		pass

