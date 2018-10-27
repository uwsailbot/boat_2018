#!/usr/bin/env python

from enum import Enum

from camera import Camera
from boat_msgs.msg import BoatState, GPS, Point, PointArray, Waypoint, WaypointArray

class DisplayData():
	def __init__(self):
		self.win_ID = 0
		self.win_width = 720
		self.win_height = 480

		self.display_path = False
		self.show_details = False

		self.compass_img = ()
		self.compass_pointer_img = ()
		self.boat_imgs = {}
		self.rudder_imgs = {}
		self.sail_imgs = {}
		self.cur_boat_img = ()
		self.cur_rudder_img = ()
		self.cur_sail_img = ()
		self.open_sans_font = ()

		self.loaded_font = []

		self.gridsize = 10

		self.camera = Camera(0,0,10, self.win_width, self.win_height)

class SimBoatState():
	def __init__(self):
		self.state = BoatState()
		self.state.major = BoatState.MAJ_DISABLED
		self.state.minor = BoatState.MIN_COMPLETE
		self.state.challenge = BoatState.CHA_NAV


class SimMode(Enum):
	DEFAULT=0
	REPLAY=1
	CONTROLLER=2

# ROS data -- search
class SearchAreaSection:
	def __init__(self, center, length, recursive_level, search_area):
		self.center = center
		self.length = length
		self.children = []
		self.recursive_level = recursive_level
		self.full= True
		if recursive_level > 0:
			self.add_child(Point(center.x+length/4.0, center.y+length/4.0), search_area)
			self.add_child(Point(center.x+length/4.0, center.y-length/4.0), search_area)
			self.add_child(Point(center.x-length/4.0, center.y-length/4.0), search_area)
			self.add_child(Point(center.x-length/4.0, center.y+length/4.0), search_area)

	def add_child(self, child_center, search_area):
		if self.recursive_level > 1 or (child_center.x-search_area.center.x)**2+(child_center.y-search_area.center.y)**2 < search_area.radius**2:
			self.children.append(SearchAreaSection(child_center, self.length/2.0, self.recursive_level - 1, search_area))

	def update(self):
		# recursively do this so we can prune children at all levels
		if self.recursive_level > 1:
			# recursively update children
			for child in self.children:
				child.update()
		
		to_remove = []
		for child in self.children:
			if self.full and not child.full:
				self.full = False
			# remove leaf nodes who are in fov, remove non-leaf nodes who have no children
			if child.recursive_level is 0:
				if point_is_in_fov(child.center):
					to_remove.append(child)
			else:
				if len(child.children) is 0:
					to_remove.append(child)
		for child in to_remove:
			self.children.remove(child)

		# consider oneself full when it has 4 children and all its children are full
		# full means all of its non-leaf children have 4 children (are complete squares)
		if self.full:
			if len(self.children) is not 4:
				self.full = False
			else:
				for child in self.children:
					if not child.full:
						self.full = False
	
	def draw(self, camera):
		glBegin(GL_QUADS)
		self._draw(camera)
		glEnd()

	def _draw(self, camera):
		if self.recursive_level is 0 or self.full:
			glColor4f(self.recursive_level*0.2, self.recursive_level*0.2, self.recursive_level*0.2, 0.3)
			extent = self.length/2.0 * camera.scale
			(x, y) = camera.lps_to_screen(self.center.x, self.center.y)
			glVertex2f(x+extent, y-extent)
			glVertex2f(x-extent, y-extent)
			glVertex2f(x-extent, y+extent)
			glVertex2f(x+extent, y+extent)
		else:
			for child in self.children:
				child._draw(camera)
	

class SearchArea:

	def __init__(self, center, radius, target):
		self.center = center
		self.radius = radius
		self.target = target
		self.sections = None

	# cpu/mem will scale exponentially with resolution
	# use ~4 for good computers, less for poorer computers
	def setup_coverage(self, resolution=3):
		if self.center is None or self.radius is 0 or self.target is None:
			print("Tried to setup coverage check for search but search area was not completely setup (c:%s r:%s t:%s)" % (self.center, self.radius, self.target))
			return
		self.resolution = int(resolution)
		sections = []
		sections_per_half_row = int(float(self.radius)/fov_radius)+1
		self.sections_per_row = sections_per_half_row*2

		for i in range(-sections_per_half_row, sections_per_half_row):
			section_x = self.center.x + fov_radius*i + fov_radius/2.0
			for j in range(-sections_per_half_row, sections_per_half_row):
				section_y = self.center.y + fov_radius*j + fov_radius/2.0
				section = SearchAreaSection(Point(section_x, section_y), fov_radius, self.resolution, self)
				section.update() # update once to finish setting up
				sections.append(section)
		self.sections = sections
	
	def update_coverage_section(self, x_section, y_section):
		section_index = x_section*self.sections_per_row + y_section
		try:
			self.sections[section_index].update()
		except IndexError:
			pass
		
	def update_coverage(self):
		if self.center is None or self.radius is 0 or self.target is None:
			return
		dx = pos.x-self.center.x
		dy = pos.y-self.center.y
		if dx**2 + dy**2 < (self.radius+fov_radius)**2:
			if dx<0:
				dx-=fov_radius # account for int trunction of neg numbers in next line
			x_section = int(dx/fov_radius)+self.sections_per_row/2
			if dy<0:
				dy-=fov_radius # account for int trunction of neg numbers in next line
			y_section = int(dy/fov_radius)+self.sections_per_row/2

			# update current section and 8 adjacent sections
			self.update_coverage_section(x_section, y_section)
			self.update_coverage_section(x_section, y_section-1)
			self.update_coverage_section(x_section, y_section+1)
			self.update_coverage_section(x_section-1, y_section)
			self.update_coverage_section(x_section-1, y_section-1)
			self.update_coverage_section(x_section-1, y_section+1)
			self.update_coverage_section(x_section+1, y_section)
			self.update_coverage_section(x_section+1, y_section-1)
			self.update_coverage_section(x_section+1, y_section+1)


class SimulationData():
	def __init__(self):
		self.display_data = DisplayData()
		self.boat_state = SimBoatState()
		self.ros_data = {"state": BoatState(),
		                 "rudder_pos": 90,
						 "winch_pos": 2000,
						 "waypoint_gps": WaypointArray(),
						 "target_heading": 270,
						 "pos": Point(0,0),
						 "heading": 270,
						 "ane_reading": 0,
						 "wind_heading": 270,
						 "rudder_output": 0,
						 "rudder_input": 0,
						 "rudder_setpoint": 0,
						 "rudder_enable": False,
						 "replay_gps_raw": GPS(),
						 "obstable_points": PointArray(),
						 "gps_bounding_box": PointArray(),
			             "gps_search_area": PointArray(),
						 "target_point": Waypoint(),
						 "vision_points_gps": PointArray()
		                 }

		self.sim_mode = SimMode.DEFAULT
		self.reset_origin_on_next_gps = False
		self.search_area = SearchArea(None, 0, None)

		self.last_time = -1
		self.speed = 10
		self.clock = 0

		self.wind_speed = 0

	# TODO: Meaning of this
	def mode_is_replay_or_controller(self):
		return self.sim_mode is SimMode.REPLAY or self.sim_mode is SimMode.CONTROLLER

	def mode_is_default(self):
		return self.sim_mode == SimMode.DEFAULT