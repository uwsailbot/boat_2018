#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import sys
from struct import *
import time
import serial
import signal

def signal_handler(siganl, frame):
	sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#Requires a serial port to read from
if len(sys.argv) == 1:
	print("Must give serial port")
	sys.exit()
ser = serial.Serial(
port=sys.argv[1],
baudrate=100000,
parity=serial.PARITY_EVEN,
stopbits=serial.STOPBITS_TWO,
bytesize=serial.EIGHTBITS,
timeout=1
)
counter = 0


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
	
	def parse(self):
		try:
			self.axes = []
			self.buttons = []
			
			buttons_temp = []
			joystick_temp = []
			
			joystick_temp.append(self.channel[1]) #channel 1-4 are for joysticks
			joystick_temp.append(self.channel[2])

			joystick_temp.append(self.channel[3])
			joystick_temp.append(self.channel[4])
			
			# Map switches A, B, C, D (labelled as per the manual) 
			# length = len(buttons_temp)
			# for x in range (0, length):
			# 	if buttons_temp[x] > 0x0700: #should be distinction point between two states of the switch
			# 		self.buttons.append(1)
			# 	else:
			# 		self.buttons.append(0)
			
			# Map the joystick values from 0 -> 255 (as a string) to  -1.00 -> 1.00
			length = len(joystick_temp)
			for x in (0, length):
				pass
				# if x is not '':
				#	 x = unpack('B', x)
				#	 x = float(x[0])
				#	 x = (x / 255.0 * 2)-1
				#	 self.axes.append(x)
				# else:
				#	 self.axes.append(0.00)
		
		except:
			print "Caught Exception"

	def length(self):
		return len(self.channel)


def start():
	# Setup publisher for the /joy topic
	pub = rospy.Publisher('joy', Joy, queue_size=10)
	rospy.init_node('receiver_to_joy', anonymous=True)
	rate = rospy.Rate(100)
	
	while not rospy.is_shutdown():
		received_data = [0]*25
		endBytes = [20,4,36,52]
		# Loop to check to make sure we have received a string of proper length with the right formatting
		#changed parameters below for SBUS
		

		while not (received_data[0] == 0x0F and received_data[24] in endBytes): 
			received_data.append(ser.read(1))
			received_data.pop(0) 

		# Once a proper packet has been received, parse it
		# receiver.parse() parsing doesn't work fully right now
		# Setup a JOY message with the parsed data and publish it

		receiver = parse_receiver(received_data)
		joy = Joy()
		joy.header.stamp = rospy.get_rostime()
		joy.right_stick_x = self.channels[1]
		joy.right_stick_y = self.channels[2]
		joy.left_stick_x = self.channels[3]
		joy.left_stick_y = self.channels[4]
		pub.publish(channels)
		rate.sleep()

if __name__ == '__main__':
	try: 
		start()
	except rospy.ROSInterruptException:
		pass
