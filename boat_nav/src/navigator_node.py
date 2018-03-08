#!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import Float32
from boat_msgs.msg import Point
from boat_msgs.msg import BoatState

# Declare global variables needed for the node
new_wind = False
ane_reading = 0
wind_heading = 0 # Direction the wind is pointing
state = BoatState()
target = Point()
target_heading = 0
rate = 0
is_new_target = False
layline = rospy.get_param('/boat/layline')

# Declare the publishers for the node
heading_pub = rospy.Publisher('target_heading', Float32, queue_size=10)
boat_state_pub = rospy.Publisher('boat_state', BoatState, queue_size=10)

def boat_state_callback(new_state):
	global state
	state = new_state

def anemometer_callback(new_heading):
	global ane_reading
	global new_wind
	
	ane_reading = new_heading.data

def compass_callback(compass):
	global wind_heading
	global new_wind

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
			if target_heading <= wind_coming-layline+3 or target_heading >= wind_coming+layline-3 and not is_new_target:
				best_heading = target_heading
			else:
				# If the waypoint is to the right of the wind...
				if best_heading > wind_coming:
					best_heading = wind_coming + layline
				else:
					best_heading = wind_coming - layline
		
		# If there isn't new wind data, DON'T update the heading
		else:
			best_heading = target_heading
	
	
	# If the target heading has updated, publish the newly calculated heading
	if best_heading is not target_heading:
		target_heading = best_heading
		heading_pub.publish(target_heading)
		rospy.loginfo(rospy.get_caller_id() + " New target heading: %f", target_heading)
	
	# Adjust the sleep to suit the node
	rate.sleep()


# Determine if the dist between two points is within the specified tolerance
def is_within_dist(p1, p2, dist):
	a = math.pow(p1.x-p2.x, 2) + math.pow(p1.y - p2.y, 2)
	return math.sqrt(a) < dist


# Initialize the node
def listener():
	rospy.init_node('navigator')
	rospy.Subscriber('boat_state', BoatState, boat_state_callback)
	rospy.Subscriber('anemometer', Float32, anemometer_callback)
	rospy.Subscriber('target_point', Point, target_callback)
	rospy.Subscriber('compass', Float32, compass_callback)
	
	# If the filters work, change lps to use /odometry/filtered
	rospy.Subscriber('lps', Point, position_callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass

