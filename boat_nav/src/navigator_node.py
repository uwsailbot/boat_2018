#!/usr/bin/env python
import actionlib
import math
import rospy
from boat_msgs.msg import BoatState, GPS, MaxVMGAction, MaxVMGGoal, Point, TackingAction, TackingGoal, LaylineAction, LaylineGoal
from std_msgs.msg import Float32

## Trigger for when the wind shifts significantly
new_wind = False

## Trigger for when we are navigating towards a brand new target
is_new_target = False

## The relative apparent wind heading, in degrees CCW from East
ane_reading = 0

## The true apparent wind heading, in degrees CCW from East
apparent_wind_heading = 0

## The current `boat_msgs.msg.BoatState`
state = BoatState()

## The `boat_msgs.msg.Point` describing the location to navigate to
target = Point()

## The angle to the target point, in degrees CCW from East
target_heading = 0

## The current boat heading, in degrees CCW from East
cur_boat_heading = 0

## Initial of position of the boat when a new target is added
start_pos = Point()
## Current boat position
cur_pos =  Point()

## The boat's speed, in m/s
boat_speed = 0

## The maximum VMG found, in m/s
max_vmg = 0

## Whether or not the max VMG has been found
found_max_vmg = False

## Direct heading to next mark
direct_heading = 0

## The angle within which the boat cannot go (Irons)
layline = rospy.get_param('/boat/layline')

# Declare the publishers for the node
heading_pub = rospy.Publisher('target_heading', Float32, queue_size=10)
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
max_vmg_client = actionlib.SimpleActionClient('max_vmg_action', MaxVMGAction)
tacking_client = actionlib.SimpleActionClient('tacking_action', TackingAction)
layline_client = actionlib.SimpleActionClient('layline_action', LaylineAction)



# =*=*=*=*=*=*=*=*=*=*=*=*= ROS Callbacks =*=*=*=*=*=*=*=*=*=*=*=*=


##	Callback for setting the boat state when the `/boat_state` topic is updated
#	
#	@param new_state The new `boat_msgs.msg.BoatState` to set
#	
def boat_state_callback(new_state):
	global state
	state = new_state


##	Callback for setting the apparent wind heading when the `/anemometer` topic is updated
#	
#	@param new_heading The new anemometer reading, 180 is directly infront of boat increasing CCW
#	
def anemometer_callback(new_heading):
	global ane_reading
	ane_reading = new_heading.data


##	Callback for setting the boat speed when the `/gps_raw` topic is updated
#	
#	@param gps The `boat_msgs.msg.GPS` message containing the current boat speed
#	
def gps_callback(gps):
	global boat_speed
	boat_speed = gps.speed * 0.514444 # Knots to m/s


##	Callback for setting the boat's heading when the `/compass` topic is updated.
#	
#	@param compass The new heading to set, in degrees CCW from East
#	
def compass_callback(compass):
	global apparent_wind_heading
	global new_wind
	global cur_boat_heading
	global target_heading
	
	cur_boat_heading = compass.data
	new_wind_heading = (ane_reading + compass.data) % 360
	
	# Tolerance on a wind shift to be determined
	# Only update wind heading if a significant shift is detected, because it will then replan our upwind path
	if abs(new_wind_heading - apparent_wind_heading) > 0.1 :
		new_wind = True
		apparent_wind_heading = new_wind_heading
		#print "Wind headings: ", ane_reading, (apparent_wind_heading + 180) % 360
	
	# If we are in RC, update our target heading to be the same direction as we are pointing, so the path planner 
	# will work when we switch to auto
	if state.major is not BoatState.MAJ_AUTONOMOUS:
		target_heading = cur_boat_heading


##	Callback for setting the target point when the `/target_point` topic is updated.
#	
#	@param new_target The `boat_msgs.msg.Point` to set
#	
def target_callback(new_target):
	global target
	global is_new_target
	global start_pos

	if abs(target.x - new_target.x) > 0.01 or abs(target.y - new_target.y) > 0.01:
		is_new_target = True
		target = new_target
		start_pos = cur_pos


##	Callback for setting the boat's location when the `/lps` topic is updated.
#	
#	@param position The `boat_msgs.msg.Point` to set
#
def position_callback(position):
	global cur_pos
	cur_pos = position 
	
	# If the boat isn't in the autonomous planning state, exit
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	awa_algorithm()


##  Callback for timer that is set after a tack or layline maneover, is used to make sure that ros messages are caught up on after actions are executed
#
#	@param event Timer event
#
def timer_callback(event):
	no_layline_planning = False


# =*=*=*=*=*=*=*=*=*=*=*=*= Calculations =*=*=*=*=*=*=*=*=*=*=*=*=

'''
##	Calculate the current velocity made good along the specified heading
#	
#	@param direct_heading The direct heading to the target, in degrees CCW from East
#	@return The current velocity made good
#	
def vmg(direct_heading):
	return math.cos(cur_boat_heading - direct_heading) * boat_speed
'''

##	Calculate the global maximum velocity made good of the entire system
#	
#	@param wind_coming The heading of the apparent wind, in degrees CCW from East
#	@return The theoretical max velocity made good
#	
def calc_global_max_vmg(wind_coming):
	
	# TODO: Make a service and make more general for object avoidance
	
	# If direct heading is in irons, then max_vmg will be on the edge of the no go zone
	if direct_heading > wind_coming-layline and direct_heading < wind_coming+layline:
		theoretic_boat_speed = 0
		if direct_heading >= wind_coming:
			vmg_heading = wind_coming + layline 
			theoretic_boat_speed = 2.5 * 0.54444
		else :
			vmg_heading = wind_coming - layline
			theoretic_boat_speed = 2.5 * 0.54444
	
	# Otherwise max vmg is just the direct heading
	else:
		theoretic_boat_speed = 2.5 * 0.54444 # 2.5 Knots to m/s (measured boat speed)
		vmg_heading = direct_heading
	
	max_vmg = theoretic_boat_speed * math.cos(math.radians(vmg_heading - direct_heading))
	return max_vmg, vmg_heading

##	Calculate the maximum velocity made good on the current tack
#	
#	@param wind_coming The heading of the apparent wind, in degrees CCW from East
#	@return The theoretical max velocity made good
#	
def calc_cur_tack_max_vmg(wind_coming):

	# TODO: Make a service and make more general for object avoidance

	# Direct heading in irons on left of wind
	if direct_heading > wind_coming-layline and direct_heading < wind_coming+layline:
		theoretic_boat_speed = 0
		if target_heading >= wind_coming:
			vmg_heading = wind_coming + layline 
			theoretic_boat_speed = 2.5 * 0.54444
		else :
			vmg_heading = wind_coming - layline
			theoretic_boat_speed = 2.5 * 0.54444
	# Direct heading lies on navigatable path elsewhere in our current tack
	elif (is_within_bounds(direct_heading, wind_coming, apparent_wind_heading) and is_within_bounds(target_heading, wind_coming, apparent_wind_heading))or\
		(not is_within_bounds(direct_heading, wind_coming, apparent_wind_heading) and not is_within_bounds(target_heading, wind_coming, apparent_wind_heading)):
		theoretic_boat_speed = 2.5 * 0.54444 # 2.5 Knots to m/s (measured boat speed)
		vmg_heading = direct_heading
	# If none of the above, the best heading will be the one closest to the other tack
	else: 
		theoretic_boat_speed = 2.5 * 0.54444
		vmg_heading = apparent_wind_heading
	
	max_vmg = theoretic_boat_speed * math.cos(math.radians(vmg_heading - direct_heading))
	return max_vmg, vmg_heading

##	Calculate the current velocity made good along the specified heading
#	
#	@param wind_coming The heading of the apparent wind, in degrees CCW from East
#	@return The velocity made good
#	
def calc_vmg(wind_coming):
	global target_heading
	if target_heading > wind_coming-layline and target_heading < wind_coming+layline:
		theoretic_boat_speed = 0		
	else:
		theoretic_boat_speed = 2.5 * 0.54444 # 2.5 Knots to m/s (measured boat speed)

	cur_vmg = theoretic_boat_speed * math.cos(math.radians(target_heading - direct_heading))
	return cur_vmg

##	Calculate the distance from the boat to the current target
#	
#	@param Boat's current position
#	@return The distance, in meters
#	
def dist_to_target(position):
	return math.sqrt(math.pow((target.y - position.y), 2) +  math.pow((target.x - position.x), 2))


##	Determine if the dist between two points is within the specified tolerance
#	
#	@param p1 The first `boat_msgs.msg.Point`
#	@param p2 The second `boat_msgs.msg.Point`
#	@param dist The tolerance distance, in meters
#	@return `True` if the points are within the tolerance
#	
def is_within_dist(p1, p2, dist):
	a = math.pow(p1.x-p2.x, 2) + math.pow(p1.y - p2.y, 2)
	return math.sqrt(a) < dist


##	Determine whether the specified value is between `boundA` and `boundB`.
#	
#	Note that the order of `boundA` and `boundB` do not matter, either can be the upper or lower bound
#	
#	@param val The value to check
#	@param boundA The first of the two bounds (Either lower or upper)
#	@param boundB The second of the two bounds (Either lower or upper)
#	@return `True` if the value is between the specified bounds
#	
def is_within_bounds(val, boundA, boundB):
	return (boundA < val and val < boundB) or (boundB < val and val < boundA)

## Determine the percentage of the course to the target is remaining
#
#	@return Percentage of course remaining
#
def remaining_course():
	cur_angle = math.atan2(cur_pos.y - start_pos.y, cur_pos.x - start_pos.x) * 180 / math.pi
	start_angle = math.atan2(target.y - start_pos.y, target.x - start_pos.x) * 180 / math.pi
	cur_angle = (cur_angle + 360) % 360
	start_angle = (start_angle + 360) % 360
	tot_dist = math.hypot(target.y - start_pos.y, target.x - start_pos.x)
	cur_dist = math.hypot(cur_pos.y - start_pos.y, cur_pos.x - start_pos.x)
	proj_dist = cur_dist * math.cos(math.radians(cur_angle-start_angle))
	return 100 - (proj_dist/tot_dist) * 100.0

## Determine if the boat is on the layline and can make the mark on the current heading
#
#	@param wind_coming The heading of the apparent wind, in degrees CCW from East
#	@param tolerance The angular tolerance to the direct heading that the target heading needs to be
#	@return True for can make it, and false for not
#
def on_layline(wind_coming, tolerance):
	# On left side of the wind
	if direct_heading >= wind_coming:
		if (target_heading - direct_heading) <= tolerance:
			return True
		else:
			return False

	# On right side of the wind
	else:
		if (target_heading - direct_heading) >= tolerance:
			return True
		else:
			return False
	

# =*=*=*=*=*=*=*=*=*=*=*=*= Algorithms =*=*=*=*=*=*=*=*=*=*=*=*=

def awa_algorithm():
	global found_max_vmg
	global max_vmg
	global target_heading
	global new_wind
	global is_new_target
	global direct_heading
	
	# Calculate the direct heading to the next waypoint
	old_direct_heading = direct_heading
	direct_heading = math.atan2(target.y - cur_pos.y, target.x - cur_pos.x) * 180 / math.pi
	direct_heading = (direct_heading + 360) % 360 # Get rid of negative angles
	wind_coming = (apparent_wind_heading + 180) % 360 # Determine the direction the wind is coming from

	# TODO: Don't think this parameter actually restricts it to exactly the width, it is more used for shape of rectangle rn
	p = 200.0 # Beating parameter, width of course in m

	# TODO: Make n a function of boat speed to negate the effects of apparent wind
	app_wind_offset = 0.5 # Make tacking less favourable because we are measuring its favour relative to apparent not true wind
	n = 1 + dist_to_target(cur_pos)*p/100.0 # Tacking weight, can add app_wind_offset here to make even less desirable

	# Tolerance the headings and the wind possibly
	if new_wind or is_new_target or direct_heading is not old_direct_heading:
		new_wind = False
		is_new_target = False
		found_max_vmg = False
		max_vmg = 0
		
		cur_vmg = calc_vmg(wind_coming) # Calculate vmg on current path
		global_max_vmg, global_vmg_heading = calc_global_max_vmg(wind_coming) # Calculate max global vmg 
		cur_tack_max_vmg, cur_tack_vmg_heading = calc_cur_tack_max_vmg(wind_coming) # Calculate the max vmg on the current talk
		#rospy.loginfo(rospy.get_caller_id() +" Cur VMG: %f Max VMG: %f with Heading: %f Cur Tack VMG: %f with Heading: %f Direct Heading: %f", cur_vmg, global_max_vmg, global_vmg_heading, cur_tack_max_vmg, cur_tack_vmg_heading, direct_heading)
		per_course_left = remaining_course()
		
		# TODO: Tolerance these properly
		# If not currently at our optimal vmg, during our regular upwind routine (not layline setup)
		if (global_max_vmg > cur_vmg or cur_tack_max_vmg > cur_vmg):
			# If the our headings are more that 180 degrees apart, reverse travel direction
			if (abs(global_vmg_heading - target_heading)) > 180:
				boat_dir = 1 
			else:
				boat_dir = -1
			
			# Is tack required to get to vmg_heading
			if (boat_dir is 1 and not is_within_bounds(wind_coming, global_vmg_heading, target_heading)) or\
				(boat_dir is -1 and is_within_bounds(wind_coming, global_vmg_heading, target_heading)):
				# If this loop is entered, then getting to vmg_heading requires a tack
				# Now we need to calculate if the tack is worth it
				if global_max_vmg > cur_tack_max_vmg * n:
					# Worth the tack, therefore determine the tacking direction and execute the action
					target_heading = global_vmg_heading
					if is_within_bounds(target_heading, 90, 270):
						tacking_direction = -1
					else:
						tacking_direction = 1
					
					heading_pub.publish(target_heading)
					goal = TackingGoal(direction = tacking_direction)
					tacking_client.send_goal(goal)
					
					# Adjust time delay until the tack is considered failed, and we return to planning
					if not tacking_client.wait_for_result(rospy.Duration(10)):
						tacking_client.cancel_goal()
						# TODO: Add other conditions upon tack failure
					target_heading = tacking_client.get_result().target_heading
					rospy.loginfo(rospy.get_caller_id() + " Boat State = 'Autonomous - Planning'")

				# If the tack is not worth preforming, set the current heading to be the max vmg of our current tack
				else:
					target_heading = cur_tack_vmg_heading
					heading_pub.publish(target_heading)
					
			# Tack is not required to get to vmg_heading, therefore set it
			else: 
				target_heading = global_vmg_heading
				heading_pub.publish(target_heading)

		#Final leg of the course, traveling on an optimal vmg course, time to get to layline.  Second condition to make sure this doesn't run if we are already on the layline
		# TODO: Tolerance correctly	
		elif per_course_left <= 40 and not on_layline(wind_coming, 1.0) :
			rospy.loginfo(rospy.get_caller_id() + " Entering the navigate to layline routine. Saved current tack angle as: %f ", target_heading)
			goal = LaylineGoal(alt_tack_angle = target_heading, overshoot_angle = 3.0, target = target)
			layline_client.send_goal(goal)
			
			# Adjust time delay until the layline setup action is considered failed, and we return to planning
			if not layline_client.wait_for_result(rospy.Duration(40)):
				layline_client.cancel_goal()
			target_heading = layline_client.get_result().target_heading

			# Disable planning while messages are caught up on
			#no_layline_planning = True
			#rospy.Timer(rospy.Duration(0.1), timer_callback)
#		else:
#			# If the waypoint is to the right of the wind...
#			if direct_heading > cur_boat_heading:
#				wind_side = 1
#			else:
#				wind_side = -1
#			
# UNCOMMENT IF WE WANT TO FIND REAL VMG
#			goal = MaxVMGGoal(wind_side, target)
#			max_vmg_client.send_goal(goal)
#		
#			# Adjust time delay until the tack is considered failed, and we return to planning
#			if not max_vmg_client.wait_for_result(rospy.Duration(10)):
#				max_vmg_client.cancel_goal()
#				# TODO: Add other conditions upon tack failure
	
	
	rospy.Rate(100).sleep()

def taras_algorithm():
	global state
	global apparent_wind_heading
	global new_wind
	global target
	global is_new_target
	global target_heading
	buoy_tolerance = 5
	
	# Calculate the direct heading to the next waypoint
	# This should never be undefined, as the atan2(0,0) case would already be caught by the proximity check above
	best_heading = math.atan2(target.y - cur_pos.y, target.x - cur_pos.x) * 180 / math.pi
	best_heading = (best_heading + 360) % 360 # Get rid of negative angles
	wind_coming = (apparent_wind_heading + 180) % 360 # Determine the direction the wind is coming from
	
	# If the direct path isn't possible...
	if best_heading > wind_coming-layline and best_heading < wind_coming+layline:
	
		# ... and there's new wind data or a new target, update the upwind path
		if new_wind or is_new_target:
			new_wind = False
			is_new_target = False
			
			# If the current heading is still acceptable, carry on
			#if target_heading <= wind_coming-layline+3 or target_heading >= wind_coming+layline-3 and not is_new_target:
			#	best_heading = target_heading
			#else:
			# If the waypoint is to the right of the wind...
			if best_heading > wind_coming:
				best_heading = wind_coming + layline
			else:
				best_heading = wind_coming - layline
		
		# If there isn't new wind data, DON'T update the heading
		else:
			best_heading = target_heading
	
	
	# If the target heading has updated, decide if we need to tack, then publish the new heading
	if best_heading is not target_heading:
		target_heading = best_heading
		
		# If the our headings are more that 180 degrees apart, reverse travel direction
		if (abs(target_heading - cur_boat_heading)) > 180:
			boat_dir = 1 
		else:
			boat_dir = -1 
		
		wind_coming = (apparent_wind_heading + 180) % 360 # Which direction the wind is coming from
		
		if (boat_dir is 1 and not is_within_bounds(wind_coming, cur_boat_heading, target_heading)) or\
			(boat_dir is -1 and is_within_bounds(wind_coming, cur_boat_heading, target_heading)):
			# Determine which direction to tack based on the side that our goal is on
			if is_within_bounds(target_heading, 90, 270):
				tacking_direction = -1
			else:
				tacking_direction = 1 
			
			heading_pub.publish(target_heading)
			
			goal = TackingGoal(direction = tacking_direction)
			tacking_client.send_goal(goal)
			
			# Adjust time delay until the tack is considered failed, and we return to planning
			if not tacking_client.wait_for_result(rospy.Duration(10)):
				tacking_client.cancel_goal()
				# TODO: Add other conditions upon tack failure
			
			rospy.loginfo(rospy.get_caller_id() + " Boat State = 'Autonomous - Planning'")
		else:
			# Publish new heading for the rudder 
			heading_pub.publish(target_heading)
			rospy.loginfo(rospy.get_caller_id() + " New target heading: %f", target_heading)
			
	rospy.Rate(100).sleep()


##	Initialize the node
#	
#	Sets up all of the subscribers, initializes the node, and blocks until
#	the action servers are ready
#	
def init():
	rospy.init_node('navigator')
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('gps_raw', GPS, gps_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback, queue_size=1)
	rospy.Subscriber('target_point', Point, target_callback)
	rospy.Subscriber('compass', Float32, compass_callback, queue_size=1) # Only want it to receive the most recent orientation
	
	# If the filters work, change lps to use /odometry/filtered
	rospy.Subscriber('lps', Point, position_callback, queue_size=1) # Only want it to receive the most recent position
	max_vmg_client.wait_for_server()
	tacking_client.wait_for_server()
	layline_client.wait_for_server()
	rospy.spin()


if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException:
		pass

