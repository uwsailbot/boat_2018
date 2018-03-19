#!/usr/bin/env python
import actionlib
import math
import rospy
from boat_msgs.msg import BoatState, GPS, MaxVMGAction, MaxVMGGoal, Point, TackingAction, TackingGoal
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
#	@param new_heading The new heading to set, in degrees CCW from East
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
	if abs(target.x - new_target.x) > 0.01 or abs(target.y - new_target.y) > 0.01:
		is_new_target = True
		target = new_target


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
	
	awa_algorithm(position)



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

##	Calculate the theoretical maximum velocity made good along the specified heading
#	
#	@param direct_heading The direct heading to the target, in degrees CCW from East
#	@param wind_coming The heading of the apparent wind, in degrees CCW from East
#	@return The theoretical max velocity made good
#	
def theoretic_max_vmg(wind_coming):
	
	# TODO: Make a service and make more general for object avoidance
	
	if direct_heading > wind_coming-layline and direct_heading < wind_coming+layline:
		theoretic_boat_speed = 0
		if direct_heading >= wind_coming:
			vmg_heading = wind_coming + layline 
			theoretic_boat_speed = 2.5 * 0.54444
		else :
			vmg_heading = wind_coming - layline
			theoretic_boat_speed = 2.5 * 0.54444
	else:
		theoretic_boat_speed = 2.5 * 0.54444 # 2.5 Knots to m/s (measured boat speed)
		vmg_heading = direct_heading
	
	max_vmg = theoretic_boat_speed * math.cos(math.radians(vmg_heading - direct_heading))
	return max_vmg, vmg_heading


##	Calculate the current velocity made good along the specified heading
#	
#	@param direct_heading The direct heading to the target, in degrees CCW from East
#	@param wind_coming The heading of the apparent wind, in degrees CCW from East
#	@return The velocity made good
#	
def calc_vmg(wind_coming):
	global target_heading

	if target_heading > wind_coming-layline and target_heading < wind_coming+layline:
		rospy.loginfo(rospy.get_caller_id() +" Target lies within no go zone.")
		#rospy.loginfo(rospy.get_caller_id() +" No go zone: %f -> %f", wind_coming-layline, wind_coming+layline)
		
		# If in no go zone, adjust the target heading to be out of it
		if target_heading >= wind_coming:
			target_heading = wind_coming + layline
			heading_pub.publish(target_heading)
		else:
			target_heading = wind_coming - layline
			heading_pub.publish(target_heading)
				
		theoretic_boat_speed = 2.5 * 0.54444
		rospy.loginfo(rospy.get_caller_id() +" Adjusted target heading: %f", target_heading)		
	else:
		theoretic_boat_speed = 2.5 * 0.54444 # 2.5 Knots to m/s (measured boat speed)
	
	#rospy.loginfo(rospy.get_caller_id() +" Calculating Max vmg on current target heading.")
	#rospy.loginfo(rospy.get_caller_id() +" Target heading: %f Direct heading: %f Wind coming: %f Boat Speed: %f", target_heading, direct_heading, wind_coming, theoretic_boat_speed)
	return theoretic_boat_speed * math.cos(math.radians(target_heading - direct_heading))


##	Calculate the distance from the boat to the current target
#	
#	@param Boat's current position
#	@return The distance, in meters
#	
def dist_to_target(cur_pos):
	return math.sqrt(math.pow((target.y - cur_pos.y), 2) +  math.pow((target.x - cur_pos.x), 2))


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



# =*=*=*=*=*=*=*=*=*=*=*=*= Algorithms =*=*=*=*=*=*=*=*=*=*=*=*=

def awa_algorithm(cur_pos):
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
		
		cur_vmg = calc_vmg(wind_coming) # Calculate vmg on target path 
		max_vmg, vmg_heading = theoretic_max_vmg(wind_coming) # Calculate max vmg 
		rospy.loginfo(rospy.get_caller_id() +" Cur VMG: %f Max VMG: %f with Heading: %f Direct Heading: %f", cur_vmg, max_vmg, vmg_heading, direct_heading)
		
		if max_vmg > cur_vmg:
			# If the our headings are more that 180 degrees apart, reverse travel direction
			if (abs(vmg_heading - target_heading)) > 180:
				boat_dir = 1 
			else:
				boat_dir = -1
			
			# Is tack required to get to vmg_heading
			if (boat_dir is 1 and not is_within_bounds(wind_coming, vmg_heading, target_heading)) or\
				(boat_dir is -1 and is_within_bounds(wind_coming, vmg_heading, target_heading)):
				# If this loop is entered, then getting to vmg_heading requires a tack
				# Now we need to calculate if the tack is worth it
				if max_vmg > cur_vmg * n:
					# Worth the tack, therefore determine the tacking direction and execute the action
					target_heading = vmg_heading
					if is_within_bounds(target_heading, 90, 270):
						tacking_direction = -1
					else:
						tacking_direction = 1 
					
					heading_pub.publish(target_heading)
					
					goal = TackingGoal(direction = tacking_direction, boat_state = state)
					tacking_client.send_goal(goal)
					
					# Adjust time delay until the tack is considered failed, and we return to planning
					if not tacking_client.wait_for_result(rospy.Duration(10)):
						tacking_client.cancel_goal()
						# TODO: Add other conditions upon tack failure
					
					rospy.loginfo(rospy.get_caller_id() + " Boat State = 'Autonomous - Planning'")
			# Tack is not required to get to vmg_heading, therefore set it
			else: 
				target_heading = vmg_heading
				heading_pub.publish(target_heading)

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

def taras_algorithm(cur_pos):
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
			
			goal = TackingGoal(direction = tacking_direction, boat_state = state)
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
#	the max_vmg_client and tacking_client action servers are ready
#	
def init():
	rospy.init_node('navigator')
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('gps_raw', GPS, gps_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	rospy.Subscriber('target_point', Point, target_callback)
	rospy.Subscriber('compass', Float32, compass_callback)
	
	# If the filters work, change lps to use /odometry/filtered
	rospy.Subscriber('lps', Point, position_callback)
	max_vmg_client.wait_for_server()
	tacking_client.wait_for_server()
	rospy.spin()


if __name__ == '__main__':
	try:
		init()
	except rospy.ROSInterruptException:
		pass

