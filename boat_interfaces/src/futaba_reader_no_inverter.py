# ====================== DOESN'T WORK =================================

#!/usr/bin/env python
import pigpio #sudo pigpiod
import rospy
from sensor_msgs.msg import Controller
import sys
from struct import *
import time
import serial

#Requires a serial port to read from
# if len(sys.argv) == 1:
# 	print("Must give serial port")
# 	sys.exit()

# ser = serial.Serial(
# port=sys.argv[1],
# baudrate=9600,
# parity=serial.PARITY_NONE,
# stopbits=serial.STOPBITS_ONE,
# bytesize=serial.EIGHTBITS,
# timeout=1
# )

counter = 0

RX = sys.argv[1]
pi = pigpio.pi()
pi.set_mode(RX, pigpio.INPUT)
try:
    pi.bb_serial_read_close(RX)
except:
    print ""

pi.bb_serial_read_open(RX, 100000, 8)
(count, inputData) = pi.bb_serial_read(RX)
pi.bb_serial_invert(RX, 1)


class parse_receiver:
	def __init__(self, data):
		# Sort input data into 8 channels
		self.channel = [0]*16
		self.channel[0]  = ((data[1]|data[2]<< 8) & 0x07FF)
        self.channel[1]  = ((data[2]>>3|data[3]<<5) & 0x07FF)
        self.channel[2]  = ((data[3]>>6|data[4]<<2|data[5]<<10) & 0x07FF)
        self.channel[3]  = ((data[5]>>1|data[6]<<7) & 0x07FF)
        self.channel[4]  = ((data[6]>>4|data[7]<<4) & 0x07FF)
        self.channel[5]  = ((data[7]>>7|data[8]<<1|data[9]<<9) & 0x07FF)
        self.channel[6]  = ((data[9]>>2|data[10]<<6) & 0x07FF)
        self.channel[7]  = ((data[10]>>5|data[11]<<3) & 0x07FF) 
        self.channel[8]  = ((data[12]|data[13]<< 8) & 0x07FF)
        self.channel[9]  = ((data[13]>>3|data[14]<<5) & 0x07FF)
        self.channel[10] = ((data[14]>>6|data[15]<<2|data[16]<<10) & 0x07FF)
        self.channel[11] = ((data[16]>>1|data[17]<<7) & 0x07FF)
        self.channel[12] = ((data[17]>>4|data[18]<<4) & 0x07FF)
        self.channel[13] = ((data[18]>>7|data[19]<<1|data[20]<<9) & 0x07FF)
        self.channel[14] = ((data[20]>>2|data[21]<<6) & 0x07FF)
        self.channel[15] = ((data[21]>>5|data[22]<<3) & 0x07FF)
		
		# each channel has 11 bits
		
		#self.data_list = data.split(' ')
	
	# def parse(self):
	# 	try:
	# 		self.axes = []
	# 		self.buttons = []
			
	# 		buttons_temp = []
	# 		joystick_temp = []
			
	# 		for i in range(1, 5): #switch A, B, C, D (channels 1 to 4)
	# 			buttons_temp.append(self.channel[i])
			
	# 		for j in range(5, 8): #vr, left joy, right joy (channels 5-7)
	# 			joystick_temp.append(self.channel[j])
			
	# 		# Map switches A, B, C, D (labelled as per the manual) 
	# 		length = len(buttons_temp)
	# 		for x in range (0, length):
	# 			if buttons_temp[x] > 0x0700: #should be distinction point between two states of the switch
	# 				self.buttons.append(1)
	# 			else:
	# 				self.buttons.append(0)
			
	# 		# Map the joystick values from 0 -> 255 (as a string) to  -1.00 -> 1.00
	# 		length = len(joystick_temp)
	# 		for x in (0, length):
	# 			pass
	# 			# if x is not '':
	# 			#	 x = unpack('B', x)
	# 			#	 x = float(x[0])
	# 			#	 x = (x / 255.0 * 2)-1
	# 			#	 self.axes.append(x)
	# 			# else:
	# 			#	 self.axes.append(0.00)
		
	# 	except:
	# 		print "Caught Exception"

	# def length(self):
	# 	return len(self.channel)


def start():
	# Setup publisher for the /joy topic
	pub = rospy.Publisher('Controller', Controller, queue_size=10)
	rospy.init_node('receiver_to_controller', anonymous=True)
	rate = rospy.Rate(100)
	
	while not rospy.is_shutdown():
		received_data = [0]*25
		endBytes = [20,4,36,52]
		# Loop to check to make sure we have received a string of proper length with the right formatting
		#changed parameters below for SBUS

    	(count, inputData) = pi.bb_serial_read(RX)
    	if count:
    		for k in inputData:
    			recieved_data.append(k)
    			received_data.pop(0)

				if received_data[0] == 0x0F and received_data[24] in endBytes:  
					receiver = parse_receiver(received_data)
		# Once a proper packet has been received, parse it
		# receiver.parse() not parsing for now
		# Setup a JOY message with the parsed data and publish it
		controller = Controller()
		controller.header.stamp = rospy.get_rostime()
		controller.channels = self.channels
		pub.publish(channels)
		rate.sleep()


if __name__ == '__main__':
	try: 
		start()
	except rospy.ROSInterruptException:
		pass
