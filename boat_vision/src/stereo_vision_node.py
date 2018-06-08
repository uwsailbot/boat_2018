#!/usr/bin/env python
import rospy
import rospkg
import math
import cv2
import numpy as np
import threading
from boat_msgs.msg import Point, PolarPoint, VisionTarget
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
  
# Assume that both cams are facing forwards, 0deg
CAM_SPACING = 0.30;	# 0.3m
#CAM_F_LEN = 4.0;	# 4.0mm ** Not needed because FOV was specified
CAM_FOV_HORIZ = 67; # Degrees
#CAM_FOV_VERT = CAM_FOV_HORIZ * 0.75 # Not used and tbh probably not accurate
CAM_HEIGHT = 720;
CAM_WIDTH = 1280;

left_img = ()
right_img = ()
left_pub = rospy.Publisher('left_cam', Image, queue_size=10)
right_pub = rospy.Publisher('right_cam', Image, queue_size=10)
vision_pub = rospy.Publisher('vision_target', VisionTarget, queue_size=10)
debug = rospy.get_param("stereo_vision/debug")

def sind(angle):
	return math.sin(math.radians(angle))

def cosd(angle):
	return math.cos(math.radians(angle))
	
# angle is in CCW degrees from x+ axis
def get_x_displacement(x_init, rad, angle):
	return cosd(angle) * rad + x_init

# angle is in CCW degrees from x+ axis
def get_y_displacement(y_init, rad, angle):
	return sind(angle) * rad + y_init

def process_cams(left, right, spacing):
	
	target = PolarPoint()
	
	# Output the request
	#rospy.loginfo(rospy.get_caller_id() + " Request: left=%.2f,%.2f, right=%.2f,%.2f", left.x, left.y, right.x, right.y)
	
	# Calculate the average y value and the angle to that y val
	#avg_y = (left.y + right.y)/2.0
	#vert_angle = ((avg_y / CAM_HEIGHT) * 2 - 1) * CAM_FOV_VERT / 2
	
	# Determine the angle to the object in each camera, from -30 to 30deg (0deg is straight ahead)
	angle1 = ((left.x / CAM_WIDTH) * 2 - 1) * CAM_FOV_HORIZ / 2; # -30 to 30deg
	angle2 = ((right.x / CAM_WIDTH) * 2 - 1) * CAM_FOV_HORIZ / 2; # -30 to 30deg
	
	#print("left: %f, right: %f", angle1, angle2)
	
	# If the lines are parallel or diverging, there is no solution
	if (angle1 <= angle2 or angle2 >= angle1):
		rospy.logwarn("Invalid input, angles are diverging")
		return False
	
	# Convert the angles from 0deg = straight to the internal angle of the triangle bound by the rays
	angle1 = -angle1 + 90
	angle2 += 90
	
	#print("int A: %f, int B: %f", angle1, angle2)
	
	# Determine the third angle through simple geometrical relationships
	angle3 = 180 - angle1 - angle2
	
	# Determine the length from each camera to the buoy, using the sine law
	dist1 = spacing / sind(angle3) * sind(angle1)
	dist2 = spacing / sind(angle3) * sind(angle2)
	
	# Determine the coordinates of the buoy relative to the center of the two cams.
	# x is horiz position, y is depth, in meters
	x_final = get_x_displacement(-spacing/2.0, dist1, angle1)
	y_final = get_y_displacement(0, dist1, angle1)
	
	# Perhaps useful if the cameras are tilted up or down?
	#depth_2d = sqrt(x_final * x_final + y_final * y_final)
	#z_final = depth_2d / sinDeg(90-vert_angle) * sinDeg(vert_angle)
	
	# Use simple trig to determine the distance and angle from the center of the
	# two cameras to the buoy's coordinates.
	
	target.dist = math.sqrt(x_final * x_final + y_final * y_final)
	#res.buoy.dist = sqrt(x_final * x_final + y_final * y_final + z_final * z_final)
	target.heading = math.degrees(math.atan2(y_final, x_final))
	
	#rospy.loginfo(rospy.get_caller_id() + "Returning response: [dist:%.2f, heading:%.2f]", res.target.dist, res.target.heading)
	return target

# Super sloppy OpenCV code
def get_buoy_coords(img, pub):
	p = Point()
	found = False
	
	#Filter
	filtered = cv2.bilateralFilter(img, 5, 500, 500)
	
	# HSV Threshold
	hsv_img = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV);
	#threshold_img = cv2.inRange(hsv_img, np.array([0, 165, 130]), np.array([15, 255, 255]));
	threshold_img = cv2.inRange(hsv_img, np.array([0, 125, 100]), np.array([25, 255, 255]));
	
	# Blur
	threshold_img = cv2.medianBlur(threshold_img, 3);
	threshold_img = cv2.GaussianBlur(threshold_img, (5,5), 2, 2);
	
	# Get center via Hough Circles
	circles = cv2.HoughCircles(threshold_img, cv2.HOUGH_GRADIENT, 1, 2, param1=200,param2=15,minRadius=0,maxRadius=0)
	
	if circles is not None:
	
		for c in circles[0]:
			center = (c[0], c[1])
			radius = c[2]
		
			if debug:
				cv2.circle(filtered, center, radius, (0,255,0), 1, 1)
		
			p.x += float(c[0])/len(circles[0])
			p.y += float(c[1])/len(circles[0])
			found = True
	
	# Get center via contours
	edge_detected_image = cv2.convertScaleAbs(cv2.Canny(threshold_img, 75, 200))
	_,contours,_ = cv2.findContours(edge_detected_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
	maxArea = 0
	bestContour = ()
	if contours is not None:
		for c in contours:
			
			# Approximate as polygon
			approx = cv2.approxPolyDP(c, 0.012 * cv2.arcLength(c, True), True)
			
			# Area
			area = cv2.contourArea(c)
			
			# Find the largest semi-circular (
			if len(approx) > 3 and area > maxArea:
				maxArea = area
				bestContour = c
			
		# Average the two measurements
		if bestContour is not ():
			m = cv2.moments(bestContour)
			
			if found:
				p.x = (p.x + m['m10']/m['m00'])/2
				p.y = (p.y + m['m01']/m['m00'])/2
			else:
				p.x = m['m10']/m['m00']
				p.y = m['m01']/m['m00']
			
			found = True
	
	# Draw stuff for debug
	if debug:
		if bestContour is not ():
			cv2.drawContours(filtered, [bestContour], 0, (255,255,255), 1)
		
		c = (int(p.x), int(p.y))
		cv2.drawMarker(filtered, c, (0,255,0), cv2.MARKER_CROSS, 150, 3);
		cv2.drawMarker(filtered, c, (0,0,255), cv2.MARKER_CROSS, 125, 3);
		cv2.drawMarker(filtered, c, (0,255,0), cv2.MARKER_CROSS, 100, 3);
		cv2.drawMarker(filtered, c, (0,0,255), cv2.MARKER_CROSS, 75, 3);
		cv2.drawMarker(filtered, c, (0,255,0), cv2.MARKER_CROSS, 50, 3);
		cv2.drawMarker(filtered, c, (0,0,255), cv2.MARKER_CROSS, 25, 3);
		cv2.drawMarker(filtered, c, (0,255,0), cv2.MARKER_CROSS, 20, 3);
		cv2.drawMarker(filtered, c, (0,0,255), cv2.MARKER_CROSS, 15, 3);
		cv2.drawMarker(filtered, c, (0,255,0), cv2.MARKER_CROSS, 10, 3);
		cv2.drawMarker(filtered, c, (0,0,255), cv2.MARKER_CROSS, 5, 3);
		
		image_message = CvBridge().cv2_to_imgmsg(filtered, encoding="bgr8")
		pub.publish(image_message)
	
	if found:
		return p
	return False

# This thread simply grabs the most recent frame as fast as possible from the cameras.
# This in combination with a synchronized retrieve() procides the most in-sync images.
class cam_thread(threading.Thread):

	def __init__(self, cam):
		threading.Thread.__init__(self)
		self.cam = cam
		self.running = True
		self.has = False
	
	def stop(self):
		self.running = False
	
	def run(self):
		
		while self.running:
			self.has = self.cam.grab()
		
		self.cam.release


def initialize_node():
	"""Initialize the node. Setup the node handle and subscribers for ROS."""
	rospy.init_node('stereo_vision')
	
	r = rospkg.RosPack()
	path = r.get_path('boat_vision') + "/debug/18"
	
	#left = cam_thread(cv2.VideoCapture(1))
	#right = cam_thread(cv2.VideoCapture(2))
	left = cam_thread(cv2.VideoCapture(rospy.get_param("/stereo_vision/left_cam")))
	right = cam_thread(cv2.VideoCapture(rospy.get_param("/stereo_vision/right_cam")))
	
	left.start()
	right.start()
	
	# Constantly process cams
	rate = rospy.Rate(rospy.get_param("/stereo_vision/rate"))
	while not rospy.is_shutdown():
		if not left.has or not right.has:
			continue
		
		_, left_img = left.cam.retrieve()
		_, right_img = right.cam.retrieve()
		
		# Make sure the resolution is accurate
		global CAM_WIDTH
		global CAM_HEIGHT
		CAM_WIDTH = left.cam.get(cv2.CAP_PROP_FRAME_WIDTH)
		CAM_HEIGHT = left.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
		
		points = map(get_buoy_coords, [left_img, right_img], [left_pub, right_pub])
		if points[0] is not False and points[1] is not False:
			polar = process_cams(points[0], points[1], CAM_SPACING)
			msg = VisionTarget()
			msg.header.stamp = rospy.Time.now()
			if polar is not False:
				msg.has = True
				msg.pt = polar
			else:
				msg.has = False
			vision_pub.publish(msg)
		
		rate.sleep()
	
	left.stop()
	right.stop()
	
	left.join()
	right.join()
	cv2.destroyAllWindows() 


if __name__ == '__main__':
	try:
		initialize_node()
	except rospy.ROSInterruptException:
		pass

