#!/usr/bin/env python
import math
import rospy
from overrides import overrides
from boat_msgs.msg import BoatState, Point, PointArray, Waypoint
from path_planners.planner_base import Planner

class SearchPlanner(Planner):
	"""Planner implementation to perform a search algorithm to find a buoy."""
	
	def __init__(self):
		self.search_center = Point() # center of search circle
		self.search_radius = 0 # radius of search circle
		self.search_target_found = False # have we found the target
		self.search_moving_to_found_target = False # have we begun moving towards target
		
		rospy.Subscriber('search_area', PointArray, self._search_area_callback)
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Main Behaviour =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	# sets up waypoints for the search routine
	# TODO call this again if we ran through all the waypoints set without finding anything?
	@overrides
	def setup(self):
		
		if self.search_radius is 0:
			return
		
		rospy.loginfo(rospy.get_caller_id() + " Setting waypoints for search challenge routine")
	
		self.search_target_found = False
		self.search_moving_to_found_target = False
	
		# Clear previous points
		#self.clear_waypoints()
	
		# stub for now
		wind_heading = 45
		num_sweeps = 10
		sweep_width = 2*self.search_radius/num_sweeps
		#waypoints = [_get_search_pt(i,wind_heading,sweep_width,search_radius) for i in xrange(0,num_sweeps-1)]
		self.update_waypoints(self._get_expaning_square_pts(wind_heading, sweep_width))
		self.publish_target(self.waypoints[0])
		self.set_minor_state(BoatState.MIN_PLANNING)
	
	@overrides
	def planner(self):
		
		if self.search_target_found:
			# TODO: Read from vision topic and republish it as target
			self.search_moving_to_found_target = True
			pass
		
		else:
			self.traverse_waypoints_planner()
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	# receives point array from topic, first point is center, second point is on the edge of the circle
	def _search_area_callback(self, search_area):
		"""Callback for search area."""
		if len(search_area.points) < 2:
			return
		
		# take first point as center
		# take second point as point on edge of circle
		self.search_center = search_area.points[0]
		dx = (search_area.points[1].x - self.search_center.x)
		dy = (search_area.points[1].y - self.search_center.y)
		self.search_radius = math.sqrt(dx*dx+dy*dy)
		
		# Start seach routine
		if self.state.challenge is BoatState.CHA_SEARCH:
			self.setup()
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Utility Functions =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	def _get_expaning_square_pts(self, angle, width):
		"""Calculate the points required to search the given area, using an expanding square algorithm.
		
		@param angle: The heading of the incoming wind, in degrees
		@param width: The width of the search path, in meters (?)
		"""
		moves = [[1,0],[0,1],[-1,0],[0,-1]]
		direction = multiplier = 1
		x = y = 0
		out = []
		c,s = math.cos(math.radians(angle)),math.sin(math.radians(angle))
		while(math.sqrt(x*x + y*y) < self.search_radius):
			index = len(out)%4
			x += moves[index][0]*direction*multiplier*width
			y += moves[index][1]*direction*multiplier*width
			trans_x = x*c - y*s + self.search_center.x
			trans_y = y*c + x*s + self.search_center.y
			out.append(Waypoint(Point(trans_x,trans_y), Waypoint.TYPE_INTERSECT))
			if not index%2:
				multiplier += 1
		return out
	
	#def _get_search_pt(i,angle,width,radius):
	#		y = -radius + (i+1)*width
	#		x = (1-2*(i%2))*math.sqrt(radius**2 - y**2)
	#		c,s = math.cos(angle),math.sin(angle)
	#		trans_x = x*c - y*s + search_center.x
	#		trans_y = y*c - x*s + search_center.y
	#		return Point(trans_x,trans_y)
	

