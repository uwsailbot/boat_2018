#!/usr/bin/env python
import actionlib
import math
import rospy
from boat_msgs.msg import BoatState, Point, TackingAction, TackingGoal
from std_msgs.msg import Float32

# Declare global variables needed for the node
new_wind = False
ane_reading = 0
wind_heading = 0 # Direction the wind is pointing
state = BoatState()
target = Point()
target_heading = 0
cur_boat_heading = 0
rate = 0
is_new_target = False
layline = rospy.get_param('/boat/layline')

# Declare the publishers for the node
heading_pub = rospy.Publisher('target_heading', Float32, queue_size=10)
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)
client = actionlib.SimpleActionClient('tacking_action', TackingAction)

def boat_state_callback(new_state):
	global state
	state = new_state

def anemometer_callback(new_heading):
	global ane_reading
	ane_reading = new_heading.data

def compass_callback(compass):
	global wind_heading
	global new_wind
	global cur_boat_heading
	
	cur_boat_heading = compass.data
	new_wind_heading = (ane_reading + compass.data) % 360

	# Tolerance on a wind shift to be determined
	# Only update wind heading if a significant shift is detected, because it will then replan our upwind path
	if abs(new_wind_heading - wind_heading) > 0.1 :
		new_wind = True
		wind_heading = new_wind_heading

def target_callback(new_target):
	global target
	global is_new_target
	if abs(target.x - new_target.x) > 0.01 or abs(target.y - new_target.y) > 0.01:
		is_new_target = True
		target = new_target

def position_callback(position):
	global state
	global wind_heading
	global new_wind
	global target
	global is_new_target
	global target_heading
	global rate
	global boat_state_pub
	global heading_pub
	global client

	buoy_tolerance = 5
	
	rate = rospy.Rate(100)
	
	# If the boat isn't in the autonomous planning state, exit
	if state.major is not BoatState.MAJ_AUTONOMOUS or state.minor is not BoatState.MIN_PLANNING:
		return
	
	# Calculate the direct heading to the next waypoint
	# This should never be undefined, as the atan2(0,0) case would already be caught by the proximity check above
	best_heading = math.atan2(target.y - position.y, target.x - position.x) * 180 / math.pi
	best_heading = (best_heading + 360) % 360 # Get rid of negative angles
	wind_coming = (wind_heading + 180) % 360 # Determine the direction the wind is coming from
	
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
		
		wind_coming = (wind_heading + 180) % 360 # Which direction the wind is coming from
	
		if (boat_dir is 1 and not is_within_bounds(wind_coming, cur_boat_heading, target_heading)) or\
			(boat_dir is -1 and is_within_bounds(wind_coming, cur_boat_heading, target_heading)):
			# Determine which direction to tack based on the side that our goal is on
			if is_within_bounds(target_heading, 90, 270):
				tacking_direction = -1
			else:
				tacking_direction = 1 

			heading_pub.publish(target_heading)

			goal = TackingGoal(direction = tacking_direction, boat_state = state)
			client.send_goal(goal)

			# Adjust time delay until the tack is considered failed, and we return to planning
			if not client.wait_for_result(rospy.Duration(10)):
				client.cancel_goal()
				# TODO: Add other conditions upon tack failure
			
			rospy.loginfo(rospy.get_caller_id() + " Boat State = 'Autonomous - Planning'")
		else:
			# Publish new heading for the rudder 
			heading_pub.publish(target_heading)
			rospy.loginfo(rospy.get_caller_id() + " New target heading: %f", target_heading)
		
	# Adjust the sleep to suit the node
	rate.sleep()


# Determine if the dist between two points is within the specified tolerance
def is_within_dist(p1, p2, dist):
	a = math.pow(p1.x-p2.x, 2) + math.pow(p1.y - p2.y, 2)
	return math.sqrt(a) < dist

# Determine whether the specified value is between boundA and boundB.
# Note that the order of boundA and boundB do not matter, either can be the upper or lower bound
def is_within_bounds(val, boundA, boundB):
	return (boundA < val and val < boundB) or (boundB < val and val < boundA)



# Initialize the node
def listener():
	rospy.init_node('navigator')
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	rospy.Subscriber('target_point', Point, target_callback)
	rospy.Subscriber('compass', Float32, compass_callback)
	
	# If the filters work, change lps to use /odometry/filtered
	rospy.Subscriber('lps', Point, position_callback)
	client.wait_for_server()
	rospy.spin()


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass

