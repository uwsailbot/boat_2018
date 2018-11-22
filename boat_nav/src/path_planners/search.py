#!/usr/bin/env python
import rospy
import math
from overrides import overrides
from actionlib import SimpleActionClient
from boat_msgs.msg import BoatState, Point, PointArray, Waypoint, IronsAction, IronsGoal
from std_msgs.msg import Bool
from path_planners import Planner

VISION_WIDTH = rospy.get_param('/boat/vision/width')

class SearchPlanner(Planner):
	"""Planner implementation to perform a search algorithm to find a buoy."""
	
	def __init__(self):
		self.area_center = Point() # center of search circle
		self.area_radius = 0 # radius of search circle
		self.vision_target = ()
		self.started_pattern = False
		
		rospy.Subscriber('search_area', PointArray, self._search_area_callback)
		rospy.Subscriber('vision', PointArray, self._vision_callback)
		
		self.irons_client = SimpleActionClient('irons_action', IronsAction)
		self.found_pub = rospy.Publisher('found_target', Bool, queue_size = 10)
		
		self.irons_client.wait_for_server()
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Main Behaviour =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	@overrides
	def setup(self):
		self.started_pattern = False
		
		if self.area_radius is 0:
			return
		
		rospy.loginfo(rospy.get_caller_id() + " Setting waypoints for search challenge routine")
		
		self.vision_target = ()
		self.found_pub.publish(False)
		
		wind_heading = self.wind_coming
		sweep_width = VISION_WIDTH
		
		# Calculate an expanding square pattern and begin traversing the pattern
		self.update_waypoints(self._get_expaning_square_pts(wind_heading, sweep_width))
		self.publish_target(self.waypoints[0])
		self.set_minor_state(BoatState.MIN_PLANNING)
		self.started_pattern = True
	
	@overrides
	def planner(self):
		
		if not self.started_pattern:
			return
		
		# If we still haven't found the target, continue traversing the pattern
		if self.vision_target is ():
			
			if len(self.waypoints) > 0:
				self.traverse_waypoints_planner()
				
			# If we ran through all the waypoints set without finding anything, restart the planner
			else:
				self.setup()
		
		# If we've seen the target at least once, navigate towards its last seen position
		else:
			self.publish_target(Waypoint(self.vision_target, Waypoint.TYPE_INTERSECT))
			
			#Once we intercept the vision target, publish and complete
			if self.boat_reached_target(): #TODO: Add additional 'complete' criteria to ensure contact
				self.found_pub.publish(True)
				self.set_minor_state(BoatState.MIN_COMPLETE) #TODO: Add MIN_SHUTDOWN??
				
				
				self.irons_client.send_goal(IronsGoal())
				
				# Adjust time delay until the layline setup action is considered failed
				if not self.irons_client.wait_for_result(rospy.Duration(20)):
					self.irons_client.cancel_goal()

	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	# receives point array from topic, first point is center, second point is on the edge of the circle
	def _search_area_callback(self, search_area):
		"""Callback for search area.
		
		The first Point is the center of the circular area, the second is a point on the edge of the area.
		That is, the distance between the two points describes the radius of the circlular area, centered on Point 1.
		
		@param search_area: The PointArray describing the search area.
		"""
		if len(search_area.points) < 2:
			return
		
		# take first point as center
		# take second point as point on edge of circle
		self.area_center = search_area.points[0]
		dx = (search_area.points[1].x - self.area_center.x)
		dy = (search_area.points[1].y - self.area_center.y)
		self.area_radius = math.sqrt(dx*dx+dy*dy)
		
		# Start seach routine
		if self.state.challenge is BoatState.CHA_SEARCH:
			self.setup()
	
	
	def _vision_callback(self, targets):
		"""Callback for vision targets.
		
		Currently, this topic should only contain a single Point, so we do not need to determine which Point in the PointArray to sail towards
		
		@param targets: The PointArray of objects seen by the boat's vision system, including buoys and obstacles
		"""
		
		# Whenever the vision sees a new/updated target, save it
		if len(targets.points) > 0:
			
			# The first time we find a target, print it
			if self.vision_target is ():
				rospy.loginfo("Found target! (%.4f, %.4f). Navigating to target.", targets.points[0].x, targets.points[0].y)
			
			self.vision_target = targets.points[0]
	
	
	# =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*= Utility Functions =*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=
	
	def _get_expaning_square_pts(self, angle, width):
		"""Calculate the points required to search the given area, using an expanding square algorithm.
		
		The square will be oriented such that the first point will be directly downwind. This
		ensures no sides of the square will ever be upwind (All sides should be ~45 degrees to wind)
		
		@param angle: The heading of the incoming wind, in degrees.
		@param width: The width of the search path, in meters
		"""
		width = width / 111319.492188 # meters to coords
		angle += 90
		out = []
		radius = width *1.5
		increment = width * math.sqrt(2) / 4
		
		# --- past stuff 

		# Add an extra point for better startup central coverage
		# out.append(self._make_waypoint(angle-90, width/2))
		
		#x = math.cos(math.radians(angle-90))*radius
		#y = math.sin(math.radians(angle-90))*radius
		#out.append(Waypoint(Point(x,y), Waypoint.TYPE_INTERSECT))
		
		# ---

		# Set 4 points in the beginning for initial loop/square
		for i in range(0,4):
			out.append(self._make_waypoint(angle, radius))
			angle += 90
			radius += increment

		# do some sweeping lines while going downwind to cover the areas around center
		# these points are arbituarily made based on simulator and can be modified based on current needs
		out.append(self._make_waypoint(angle-90, radius*0.6))
		out.append(self._make_waypoint(angle-45, radius*0.707))
		out.append(self._make_waypoint(angle-90, radius*0.4))
		out.append(self._make_waypoint(angle-20, radius*0.8))
		out.append(self._make_waypoint(angle-90, 0))
		out.append(self._make_waypoint(angle, radius*0.9))
		angle += 90
		radius += increment

		# continue setting points until we reach outer radius of circle
		while radius < self.area_radius + width:
			out.append(self._make_waypoint(angle, radius))
			angle += 90
			radius += increment
		
		# Add extra points for better coverage at edge of circle
		radius += increment
		for i in range(0,4):
			out.append(self._make_waypoint(angle, radius))
			angle += 90

		return out
	
	def _make_waypoint(self, angle, radius):
		x = self.area_center.x + math.cos(math.radians(angle))*radius
		y = self.area_center.y + math.sin(math.radians(angle))*radius
		return Waypoint(Point(x,y), Waypoint.TYPE_INTERSECT)

	#def _get_search_pt(i,angle,width,radius):
	#		y = -radius + (i+1)*width
	#		x = (1-2*(i%2))*math.sqrt(radius**2 - y**2)
	#		c,s = math.cos(angle),math.sin(angle)
	#		trans_x = x*c - y*s + area_center.x
	#		trans_y = y*c - x*s + area_center.y
	#		return Point(trans_x,trans_y)
	

