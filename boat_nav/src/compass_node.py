#!/usr/bin/env python
import rospy
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
import math

compass_filtered_pub = rospy.Publisher('compass_filtered', Float32, queue_size=10)
compass_mag_pub = rospy.Publisher('compass_mag', Float32, queue_size=10)

def orientation_callback(imu):
	global compass_filtered_pub

	roll, pitch, yaw = euler_from_quaternion(imu.orientation)
    
	# Currently believe that yaw = 0 when facing north
	# We want it so that 0 is facing east
	yaw += 90
	if yaw >= 360:
		yaw = yaw % 360
	
	heading = Float32()
	heading.data = yaw
	compass_filtered_pub.publish(heading)

def mag_callback(mag):
	global compass_mag_pub

	# X axis points towards west by default
	x = -1 * mag.magnetic_field.x
	y = mag.magnetic_field.y

	heading = Float32()
	heading.data = math.degrees(math.atan2(y, x))
	compass_mag_pub.publish(heading)

def listener():
	rospy.init_node('compass')

	rospy.Subscriber('imu/data', Imu, orientation_callback)
	rospy.Subscriber('imu/mag', MagneticField, mag_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
