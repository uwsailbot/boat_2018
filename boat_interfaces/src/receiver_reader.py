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
        # Each string of information sent from the controller is sent in the following format:
        # s 4 4 4 4 4 4 4 4 e\r\n
        # The following code deletes the start and end characters and splits the string of values into an list of values
        data = data.replace('s ', "")
        data = data.replace(' e', "")
        data = data.replace('\r\n', "")
        self.data_list = data.split(' ')

    def parse(self):
        try:
            self.axes = []
            self.buttons = []
            joystick_temp = self.data_list[:4]
            dpad_temp = []
            button_temp = []

            # Split the list of values into joystick and button controls
            for i in range(4, 8):
                dpad_temp.append(self.data_list[i])

            for j in range(8,19):
                button_temp.append(self.data_list[j])

            # Map the joystick values from 0 -> 255 (as a string) to  -1.00 -> 1.00
            for x in joystick_temp:
                if x is not '':
                    x = unpack('B', x)
                    x = float(x[0])
                    x = (x / 255.0 * 2)-1
                    self.axes.append(x)
                else:
                    self.axes.append(0.00)

            # Map the D-pad values from -1 -> 0 -> 1 (as a string) to  0 -> 1
            for x in dpad_temp:
                if x is not '':
                    x = unpack('B', x)
                    x = x[0]
                    x = x - 1
                    self.axes.append(x)
                else:
                    self.axes.append(1)

            # Map the regular button values from 0 -> 1 (as a string) to  0 -> 1
            for x in button_temp:
                if x is not '':
                    x = unpack('B', x)
                    x = x[0]
                    self.buttons.append(x)
                else:
                    self.buttons.append(0)

        except:
            print "Caught Exception"

    def length(self):
        return len(self.data_list)


def start():
    # Setup publisher for the /joy topic
    pub = rospy.Publisher('joy', Joy, queue_size=10)
    rospy.init_node('receiver_to_joy', anonymous=True)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        data_length = 0
        received_data= ""
        # Loop to check to make sure we have received a string of proper length with the right formatting
        while received_data[:2] != "s " or received_data[-4:] != ' e\r\n' or data_length != 19:
            received_data = ser.readline()
            if received_data[:2] == "s " and received_data[-4:] == ' e\r\n':
                receiver = parse_receiver(received_data)
                data_length = receiver.length()
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
