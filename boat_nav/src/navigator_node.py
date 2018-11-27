#!/usr/bin/env python
import rospy
import math
from actionlib import SimpleActionClient
from boat_msgs.msg import BoatState, GPS, MaxVMGAction, MaxVMGGoal, Point, Waypoint, TackingAction, TackingGoal, LaylineAction, LaylineGoal
from boat_msgs.srv import ConvertPoint
from std_msgs.msg import Float32
from boat_utilities import points, angles

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

## The `boat_msgs.msg.Waypoint` describing the location to navigate to
target = Waypoint()

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

## Minimum boat speed (in m/s) required to tack:
min_tacking_speed = rospy.get_param('/boat/nav/min_tacking_speed')

## The maximum VMG found, in m/s
max_vmg = 0

## Whether or not the max VMG has been found
found_max_vmg = False

## Direct heading to next mark
direct_heading = 0

## Beating parameter for determining when to tack
p = rospy.get_param('/boat/nav/beating')

## The angle within which the boat cannot go (Irons)
layline = rospy.get_param('/boat/nav/layline')

# Declare the publishers for the node
heading_pub = rospy.Publisher('target_heading', Float32, queue_size=10)
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
max_vmg_client = SimpleActionClient('max_vmg_action', MaxVMGAction)
tacking_client = SimpleActionClient('tacking_action', TackingAction)
layline_client = SimpleActionClient('layline_action', LaylineAction)

# Service to convert gps to lps
to_lps = rospy.ServiceProxy('gps_to_lps', ConvertPoint)


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
	global apparent_wind_heading
	global new_wind
	ane_reading = new_heading.data
	new_wind_heading = angles.normalize(ane_reading + cur_boat_heading)

	# Tolerance on a wind shift to be determined
	# Only update wind heading if a significant shift is detected, because it will then replan our upwind path
	if abs(new_wind_heading - apparent_wind_heading) > 0.1:
		new_wind = True
		apparent_wind_heading = new_wind_heading


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
	new_wind_heading = angles.normalize(ane_reading + cur_boat_heading)

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
#	@param new_target The `boat_msgs.msg.Waypoint` to set
#
def target_callback(new_target):
	global target
	global is_new_target
	global start_pos

	new_target = Waypoint(to_lps(new_target.pt).pt, new_target.type)
	# If the target has changed, save the new target
	if abs(target.pt.x - new_target.pt.x) > 0.01 or abs(target.pt.y - new_target.pt.y) > 0.01:
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

	# Temporary jank solution
	if state.challenge is BoatState.CHA_AVOID:
		return

	awa_algorithm()

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
	upper_bound = angles.normalize(wind_coming + layline)
	lower_bound = angles.normalize(wind_coming - layline)

	# If direct heading is in irons, then max_vmg will be on the edge of the no go zone
	if angles.is_within_bounds(direct_heading, lower_bound, upper_bound):

		# Snap to whichever edge of the no go zone is closer
		if angles.is_on_left(direct_heading, wind_coming):
			vmg_heading = upper_bound
		else:
			vmg_heading = lower_bound

	# Otherwise max vmg is just the direct heading
	else:
		vmg_heading = direct_heading

	theoretic_boat_speed = 2.5 * 0.514444 # 2.5 Knots to m/s (measured boat speed)
	max_vmg = theoretic_boat_speed * angles.cosd(vmg_heading - direct_heading)
	return max_vmg, vmg_heading

##	Calculate the maximum velocity made good on the current tack
#
#	@param wind_coming The heading of the apparent wind, in degrees CCW from East
#	@return The theoretical max velocity made good
#
def calc_cur_tack_max_vmg(wind_coming):

	# TODO: Make a service and make more general for object avoidance
	upper_bound = angles.normalize(wind_coming + layline)
	lower_bound = angles.normalize(wind_coming - layline)

	# If direct heading is in irons, then max_vmg will be on the edge of the no go zone
	if angles.is_within_bounds(direct_heading, lower_bound, upper_bound):
		theoretic_boat_speed = 2.5 * 0.514444 # 2.5 Knots to m/s

		# Snap to whichever edge of the no go zone is closer
		if angles.is_on_left(target_heading, wind_coming):
			vmg_heading = upper_bound
		else:
			vmg_heading = lower_bound

	# If the direct heading is somewhere in our current tack (direct_heading and target_heading on same side of wind)
	elif (angles.is_on_left(direct_heading, wind_coming) and angles.is_on_left(target_heading, wind_coming)) or\
		(angles.is_on_right(direct_heading, wind_coming) and angles.is_on_right(target_heading, wind_coming)):
		theoretic_boat_speed = 2.5 * 0.514444 # 2.5 Knots to m/s
		vmg_heading = direct_heading

	# If none of the above, the best heading will be on the opposite tack
	# however it is extremely unfavorable because clearly no best heading lies on this tack
	else:
		theoretic_boat_speed = 0
		vmg_heading = apparent_wind_heading

	max_vmg = theoretic_boat_speed * angles.cosd(vmg_heading - direct_heading)
	return max_vmg, vmg_heading

##	Calculate the current velocity made good along the current target heading
#
#	@param wind_coming The heading of the apparent wind, in degrees CCW from East
#	@return The velocity made good
#
def calc_vmg(wind_coming):
	tolerance = 1.0
	upper_bound = angles.normalize(wind_coming + layline - tolerance)
	lower_bound = angles.normalize(wind_coming - layline + tolerance)

	# If we are in irons, our theoretical speed is 0
	if angles.is_within_bounds(target_heading, lower_bound, upper_bound):
		theoretic_boat_speed = 0
	else:
		theoretic_boat_speed = 2.5 * 0.514444 # 2.5 Knots to m/s (measured boat speed)

	cur_vmg = theoretic_boat_speed * angles.cosd(target_heading - direct_heading)
	return cur_vmg

##	Calculate the distance from the boat to the current target
#
#	@param Boat's current position
#	@return The distance, in meters
#
def dist_to_target(position):
	return points.dist(target.pt, position)

## Determine the percentage of the course to the target is remaining
#
#	@return Percentage of course remaining
#
def remaining_course():
	cur_angle = angles.atan2d(cur_pos.y - start_pos.y, cur_pos.x - start_pos.x)
	start_angle = angles.atan2d(target.pt.y - start_pos.y, target.pt.x - start_pos.x)
	cur_angle = angles.normalize(cur_angle)
	start_angle = angles.normalize(start_angle)
	tot_dist = math.hypot(target.pt.y - start_pos.y, target.pt.x - start_pos.x)
	cur_dist = math.hypot(cur_pos.y - start_pos.y, cur_pos.x - start_pos.x)
	proj_dist = cur_dist * angles.cosd(cur_angle-start_angle)
	return 100 - (proj_dist/tot_dist) * 100.0

## Determine if the boat is on the layline and can make the mark on the current heading
#
#	@param wind_coming The heading of the apparent wind, in degrees CCW from East
#	@param tolerance The angular tolerance to the direct heading that the target heading needs to be
#	@return True for can make it, and false for not
#
def on_layline(wind_coming, tolerance):

	# If waypoint is on left side of the wind, we are on the layline iff the target heading
	# is to the right of the direct heading
	if angles.is_on_left(direct_heading, wind_coming):
		val = angles.is_on_right(target_heading, direct_heading + tolerance)

	# If waypoint is on right side of the wind, we are on the layline iff the target heading
	# is to the left of the direct heading
	else:
		val = angles.is_on_left(target_heading, direct_heading - tolerance)

	return val


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
	direct_heading = angles.atan2d(target.pt.y - cur_pos.y, target.pt.x - cur_pos.x)
	direct_heading = angles.normalize(direct_heading)
	wind_coming = angles.normalize(apparent_wind_heading + 180) # Determine the direction the wind is coming from

	# TODO: Make n a function of boat speed to negate the effects of apparent wind?
	n = 1 + p*1.3/dist_to_target(start_pos) # Tacking weight, can add app_wind_offset here to make even less desirable

	# Tolerance the headings and the wind possibly
	if new_wind or is_new_target or direct_heading is not old_direct_heading: # TODO: Add a slow timer to this as well. Sometimes on state changes we get stuck because nothing updates.
		new_wind = False
		is_new_target = False
		found_max_vmg = False
		max_vmg = 0

		cur_vmg = calc_vmg(wind_coming) # Calculate vmg on current path
		global_max_vmg, global_vmg_heading = calc_global_max_vmg(wind_coming) # Calculate max global vmg
		cur_tack_max_vmg, cur_tack_vmg_heading = calc_cur_tack_max_vmg(wind_coming) # Calculate the max vmg on the current tack
		#rospy.loginfo(rospy.get_caller_id() +" Cur VMG: %f Max VMG: %f with Heading: %f Cur Tack VMG: %f with Heading: %f Direct Heading: %f", cur_vmg, global_max_vmg, global_vmg_heading, cur_tack_max_vmg, cur_tack_vmg_heading, direct_heading)
		per_course_left = remaining_course()

		# TODO: Tolerance these properly
		# If not currently at our optimal vmg, during our regular upwind routine (not layline setup)
		if (global_max_vmg > cur_vmg or cur_tack_max_vmg > cur_vmg):
			# Is tack required to get to vmg_heading
			if (angles.is_on_left(global_vmg_heading, target_heading) and angles.is_on_left(wind_coming, target_heading) and angles.is_on_right(wind_coming, global_vmg_heading)) or\
				(angles.is_on_right(global_vmg_heading, target_heading) and angles.is_on_left(wind_coming, global_vmg_heading) and angles.is_on_right(wind_coming, target_heading)):
				# If this loop is entered, then getting to vmg_heading requires a tack
				# Now we need to calculate if the tack is worth it
				if global_max_vmg > cur_tack_max_vmg * n and boat_speed >= min_tacking_speed:
					# Worth the tack, therefore determine the tacking direction and execute the action
					target_heading = global_vmg_heading
					if angles.is_on_left(target_heading, wind_coming):
						tacking_direction = -1
					else:
						tacking_direction = 1

					heading_pub.publish(Float32(target_heading))
					goal = TackingGoal(direction = tacking_direction)
					tacking_client.send_goal(goal)

					# Adjust time delay until the tack is considered failed, and we return to planning
					if not tacking_client.wait_for_result(rospy.Duration(10)):
						tacking_client.cancel_goal()
						goal.direction = goal.direction * -1
						tacking_client.send_goal(goal)
						if not tacking_client.wait_for_result(rospy.Duration(10)):
							tacking_client.cancel_goal()
					if tacking_client.get_result() is not None:
						target_heading = tacking_client.get_result().target_heading
					rospy.loginfo(rospy.get_caller_id() + " Boat State = 'Autonomous - Planning'")
					heading_pub.publish(target_heading)

				# If the tack is not worth preforming, set the current heading to be the max vmg of our current tack
				else:
					target_heading = cur_tack_vmg_heading
					#print target_heading
					heading_pub.publish(Float32(target_heading))

			# Tack is not required to get to vmg_heading, therefore set it
			else:
				target_heading = global_vmg_heading
				heading_pub.publish(Float32(target_heading))

		#Final leg of the course, traveling on an optimal vmg course, time to get to layline.  Second condition to make sure this doesn't run if we are already on the layline
		# TODO: Tolerance correctly, make sure this isnt called on a downwind cuz we mistolerance it.  Shouldnt be because target_heading will constantly be updating to be equal to global_vmg_heading above
		# TODO: Ensure this is only called once per run. Currently if we have a short laylineaction, it can get repeated
		elif per_course_left <= 40 and not on_layline(wind_coming, 1.0) and boat_speed >= min_tacking_speed:
			rospy.loginfo(rospy.get_caller_id() + " Entering the navigate to layline routine. Saved current tack angle as: %f ", target_heading)
			goal = LaylineGoal(alt_tack_angle = target_heading, overshoot_angle = 3.0, target = target)
			layline_client.send_goal(goal)

			# Adjust time delay until the layline setup action is considered failed, and we return to planning
			if not layline_client.wait_for_result(rospy.Duration(40)):
				layline_client.cancel_goal()
			if layline_client.get_result() is not None:
				target_heading = layline_client.get_result().target_heading


	rospy.Rate(100).sleep()

# DEPRECATED
def taras_algorithm():
	global state
	global apparent_wind_heading
	global new_wind
	global target
	global is_new_target
	global target_heading

	# Calculate the direct heading to the next waypoint
	# This should never be undefined, as the atan2(0,0) case would already be caught by the proximity check above
	best_heading = angles.atan2d(target.pt.y - cur_pos.y, target.pt.x - cur_pos.x)
	best_heading = angles.normalize(best_heading)
	wind_coming = angles.normalize(apparent_wind_heading + 180) # Determine the direction the wind is coming from

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

		wind_coming = angles.normalize(apparent_wind_heading + 180) # Which direction the wind is coming from

		# TODO: This may be broken now that is_within_bounds is not bi-directional
		if (boat_dir is 1 and not angles.is_within_bounds(wind_coming, cur_boat_heading, target_heading)) or\
			(boat_dir is -1 and angles.is_within_bounds(wind_coming, cur_boat_heading, target_heading)):

			# Determine which direction to tack based on the side that our goal is on
			if angles.is_within_bounds(target_heading, 90, 270):
				tacking_direction = -1
			else:
				tacking_direction = 1

			heading_pub.publish(target_heading)

			goal = TackingGoal(direction = tacking_direction)
			tacking_client.send_goal(goal)

			# Adjust time delay until the tack is considered failed, and we return to planning
			if not tacking_client.wait_for_result(rospy.Duration(10)):
				tacking_client.cancel_goal()

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
	rospy.Subscriber('target_point', Waypoint, target_callback)
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

