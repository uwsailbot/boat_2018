#!/usr/bin/env python
import math
import rospy
from boat_msgs.msg import BoatState, Point, PointArray, Waypoint
from path_planners.config import Globals
from path_planners.nav import traverse_waypoints_planner

class SearchPlanner:
	
	def __init__(self):
		self.search_center = Point() # center of search circle
		self.search_radius = 0 # radius of search circle
		self.search_target_found = False # have we found the target
		self.search_moving_to_found_target = False # have we begun moving towards target
		
		rospy.Subscriber('search_area', PointArray, self.search_area_callback)
	
	
	# receives point array from topic, first point is center, second point is on the edge of the circle
	def search_area_callback(self, search_area):
		if len(search_area.points) < 2:
			return
		
		# take first point as center
		# take second point as point on edge of circle
		self.search_center = search_area.points[0]
		dx = (search_area.points[1].x - self.search_center.x)
		dy = (search_area.points[1].y - self.search_center.y)
		self.search_radius = math.sqrt(dx*dx+dy*dy)
		
		# Start seach routine
		if Globals.state.challenge is BoatState.CHA_SEARCH:
			self.setup()
	
	
	# sets up waypoints for the search routine
	# TODO call this once we are in search mode and search area has been setup
	# TODO call this again if we ran through all the waypoints set without finding anything?
	def setup(self):
		rospy.loginfo(rospy.get_caller_id() + " Setting waypoints for search challenge routine")
	
		self.search_target_found = False
		self.search_moving_to_found_target = False
	
		# Clear previous points
		#Globals.clear_waypoints()
	
		# stub for now
		wind_heading = 45
		num_sweeps = 10
		sweep_width = 2*self.search_radius/num_sweeps
		#waypoints = [_get_search_pt(i,wind_heading,sweep_width,search_radius) for i in xrange(0,num_sweeps-1)]
		Globals.update_waypoints(self._get_expaning_square_pts(wind_heading, sweep_width))
		
		Globals.set_minor_state(BoatState.MIN_PLANNING)
	
	def planner(self):
		
		if self.search_target_found:
			# TODO: Read from vision topic and republish it as target
			self.search_moving_to_found_target = True
			pass
		
		else:
			traverse_waypoints_planner()
	
	
	def _get_expaning_square_pts(self, angle, width):
		moves = [[1,0],[0,1],[-1,0],[0,-1]]
		direction = multiplier = 1
		x = y = 0
		out = []
		c,s = math.cos(math.radians(angle)),math.sin(math.radians(angle))
		while(math.sqrt(x*x + y*y) < self.search_radius):
			print len(out), x, y
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
	

