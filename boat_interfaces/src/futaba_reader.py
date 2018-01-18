#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import sys
from struct import *
      
import time
import serial

#Requires a serial port to read from
if len(sys.argv) == 1:
    print("Must give serial port")
    sys.exit()
ser = serial.Serial(
port=sys.argv[1],
baudrate=9600,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,
timeout=1
)
counter = 0


class parse_receiver:
    def __init__(self, data):
        # Sort input data into 8 channels
        self.channel = []
        self.channel.append(data[0])
        self.channel.append((((data[2] & 0x0007) << 8) | data[1]))
        self.channel.append((((data[2] & 0x00F7) >> 3) | ((data[3] & 0x003F) << 5)))
        self.channel.append((((data[3] & 0x00C0) >> 6) | (data[4] << 2) | ((data[5] & 0x0001) << 10)))
        self.channel.append((((data[5] & 0x00FE) >> 1) | ((data[6] & 0x000F) << 7)))
        self.channel.append((((data[6] & 0x00F0) >> 4) | ((data[7] & 0x007F) << 4)))
        self.channel.append((((data[7] & 0x0080) >> 7) | (data[8] << 1) | ((data[9] & 0x0003) << 9)))
        self.channel.append((((data[9] & 0x00FC) >> 2) | ((data[10] & 0x001F) << 6)))

        # each channel has 11 bits

        #self.data_list = data.split(' ')

    def parse(self):
        try:
            self.axes = []
            self.buttons = []
            
            buttons_temp = []
            joystick_temp = []

            for i in range(1, 5): #switch A, B, C, D (channels 1 to 4)
                buttons_temp.append(self.channel[i])

            for j in range(5, 8): #vr, left joy, right joy (channels 5-7)
                joystick_temp.append(self.channel[j])

            # Map switches A, B, C, D (labelled as per the manual) 
            length = len(buttons_temp)
            for x in range (0, length):
                if buttons_temp[x] > 0x0700: #should be distinction point between two states of the switch
                    self.buttons.append(1)
                else:
                    self.buttons.append(0)

            # Map the joystick values from 0 -> 255 (as a string) to  -1.00 -> 1.00
            length = len(joystick_temp)
            for x in (0, length):

                # if x is not '':
                #     x = unpack('B', x)
                #     x = float(x[0])
                #     x = (x / 255.0 * 2)-1
                #     self.axes.append(x)
                # else:
                #     self.axes.append(0.00)

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
        data_length = 0
        received_data = []
        # Loop to check to make sure we have received a string of proper length with the right formatting
        #changed parameters below for SBUS
        while received_data[0] != 0x0F or received_data[24] != 0 or num_channel != 8:  
            for i in range (0,24):
                received_data[i] = ser.read(1) #reads one byte
            if received_data[0] == 0x0F and received_data[24] == 0:
                receiver = parse_receiver(received_data)
                num_channel = receiver.length()
        # Once a proper packet has been received, parse it
        receiver.parse()
        # Setup a JOY message with the parsed data and publish it
        joy = Joy()
        joy.header.stamp = rospy.get_rostime()
        joy.axes = receiver.axes
        joy.buttons = receiver.buttons
        pub.publish(joy)
        rate.sleep()


if __name__ == '__main__':
    try: 
	    start()
    except rospy.ROSInterruptException:
	    pass
