#!/usr/bin/env python
import math
import rospy
from overrides import overrides
from boat_msgs.msg import BoatState, Point, PointArray, Waypoint
from path_planners.planner_base import Planner, Services

HEIGHT_TO_TRAVEL = rospy.get_param('/boat/planner/station/height')
MAX_WIDTH = rospy.get_param('/boat/planner/station/width')
INNER_BOX_SCALE = rospy.get_param('/boat/planner/station/inner_box_scale')

class StationPlanner(Planner):
	"""Planner implementation to stay within a location for 5 minutes, and then exit."""
	
	def __init__(self):
		self.box = []
		self.start_station = Point()
		self.end_station = Point()
		self.station_timeout = False
		
		rospy.Subscriber('bounding_box', PointArray, self._bounding_box_callback)
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Main Behaviour =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	@overrides
	def setup(self):
		
		if len(self.box) is not 4:
			return
		
		rospy.loginfo(rospy.get_caller_id() + " Beginning station challenge path planner routine")
		
		rospy.Timer(rospy.Duration(5*60), self._timer_callback, oneshot=True)
		
		# Clear previous points
		self.clear_waypoints()
		
		# Determine wall angles and box widths, as well as slope of the bottom of the box, as long as it's not vertical
		m_bottom = 0
		vert = False
		
		box = self.box
		
		if abs(box[3].x - box[0].x) > 0.0001:
			m_bottom = (box[3].y - box[0].y)/(box[3].x - box[0].x)
		else:
			vert = True
		bottom_box_width = math.hypot(box[3].y - box[0].y, box[3].x - box[0].x)
		top_box_width = math.hypot(box[2].y - box[1].y, box[2].x - box[1].x)
		left_box_height = math.hypot(box[1].y - box[0].y, box[1].x - box[0].x)
		right_box_height = math.hypot(box[3].y - box[2].y, box[3].x - box[2].x)

		# Determine width at the height we will be traveling
		station_width = (bottom_box_width + (top_box_width-bottom_box_width) * HEIGHT_TO_TRAVEL) * MAX_WIDTH
		station_height = (left_box_height + (right_box_height-left_box_height) * ((1 - MAX_WIDTH) / 2.0)) * HEIGHT_TO_TRAVEL
	
		# Determine the two points that will alternate, first find centre point of box
		y_sum = 0
		x_sum = 0
		for p in box:
			x_sum += p.x
			y_sum += p.y
		x_avr = x_sum / 4.0
		y_avr = y_sum / 4.0
	
		y_int = box[0].y - m_bottom * box[0].x
		self.start_station = start_station = Point()
		self.end_station = end_station = Point()

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
		
		self.publish_target(Waypoint(start_station, Waypoint.TYPE_INTERSECT))
		self.set_minor_state(BoatState.MIN_PLANNING)
	
	
	@overrides
	def planner(self):
		
		# If time is up and we've fully exited the box, stop
		if self.station_timeout:
			if self.boat_reached_target():
				self.set_minor_state(BoatState.MIN_COMPLETE)
				self.station_timeout = False
				rospy.loginfo(rospy.get_caller_id() + " Exited box. Boat State = 'Autonomous - Complete'")
			return
		
		# If the box is ever invalid, stop
		if len(self.box) is not 4 and self.state.minor is not BoatState.MIN_COMPLETE:
			self.set_minor_state(BoatState.MIN_COMPLETE)
			rospy.loginfo(rospy.get_caller_id() + " Box is invalid. Boat State = 'Autonomous - Complete'")
			return
		
		#TODO: Make this not spam (Eg. if we're already headed for the center, don't re-publish
		if not self._within_box():
			x_avr = 0
			y_avr = 0
			for p in self.box:
				x_avr += p.x / float(len(self.box))
				y_avr += p.y / float(len(self.box))
			self.publish_target(Waypoint(Point(x_avr, y_avr), Waypoint.TYPE_INTERSECT))
			return
		
		# If we've reached the set waypoint, flip around
		if self.boat_reached_target():
			if self.is_within_dist(self.target_waypoint.pt, self.start_station, 0.0001):
				self.publish_target(Waypoint(self.end_station, Waypoint.TYPE_INTERSECT))
			else:
				self.publish_target(Waypoint(self.start_station, Waypoint.TYPE_INTERSECT))
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	def _timer_callback(self, event):
		"""Exit the station once the 5 minutes of the challenge are complete."""
		self.station_timeout = True
		rospy.loginfo(rospy.get_caller_id() + " Reached end of station timer")
		
		box = self.box
		
		# Determine distance from each side of the box 
		l_dist = self._dist_from_line(box[0], box[1])
		t_dist = self._dist_from_line(box[1], box[2])
		r_dist = self._dist_from_line(box[3], box[2])
		b_dist = self._dist_from_line(box[0], box[3])
	
		dist_list = (l_dist, t_dist, r_dist, b_dist)
		i = dist_list.index(min(dist_list))
		
		# Use service to determine 12m dist in gps
		# Use this distance on how far to put the buoy outside the box
		tol_dist = Services.to_gps(Point(20,0)).x
		cur_pos_gps = Services.to_gps(self.cur_pos)
		
		final_point = {0 : Point(box[0].x - tol_dist, cur_pos_gps.y),
					1 : Point(cur_pos_gps.x, box[1].y + tol_dist),
					2 : Point(box[3].x + tol_dist, cur_pos_gps.y),
					3 : Point(cur_pos_gps.x, box[0].y - tol_dist)
		}
		final_direction = {0 : "Left", 1: "Top", 2: "Right", 3: "Bottom"}
		rospy.loginfo(rospy.get_caller_id() + " Exiting bounding box through: " + final_direction[i])
		
		self.publish_target(Waypoint(final_point[i], Waypoint.TYPE_INTERSECT))
	
	
	def _bounding_box_callback(self, bounding_box):
		"""Callback for station bounding box."""
		wind_coming = self.wind_coming
		
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
				self.box = temp
		else:
			self.box = bounding_box.points
		
		# For if bounding box is added after we are in station mode
		if self.state.challenge is BoatState.CHA_STATION:
			self.setup()
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Utility Functions =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	def _dist_from_line(self, start_point, end_point):
		"""Calculate the distance of the boat from the line.
		
		@param start_point: The initial point on the line
		@param end_point:The second point on the line
		@return: The distance in coordinates
		"""
		cur_pos_gps = Services.to_gps(self.cur_pos)
		if (end_point.x - start_point.x) <= 0.0001:
			return cur_pos_gps.x - end_point.x
		m = (end_point.y - start_point.y)/(end_point.x - start_point.x)
		b = start_point.y - m * start_point.x
		return abs(b + m * cur_pos_gps.x - cur_pos_gps.y)/(math.sqrt(1 + m*m))
	
	
	def _within_box(self):
		"""Determine if the boat is within the inner box.
		
		@return: True if the boat is within the box
		"""
		# Find centre of box
		y_sum = 0
		x_sum = 0
		for p in self.box:
			x_sum += p.x
			y_sum += p.y
		x_avr = x_sum / 4.0
		y_avr = y_sum / 4.0
		inner_box = [None]*4
		# Construct the inner box, using scale parameter from yaml
		for i in range(4):
			pt = Point()
			pt.y = (self.box[i].y - y_avr)*INNER_BOX_SCALE + y_avr
			pt.x = (self.box[i].x - x_avr)*INNER_BOX_SCALE + x_avr
			inner_box[i] = pt
	
		# Create horizontal vector from the cur_pos towards +x
		start_point = Services.to_gps(self.cur_pos)
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
				
		
		return (intersections % 2) is not 0

