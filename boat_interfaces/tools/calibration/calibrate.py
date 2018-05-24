#!/usr/bin/env python
from subprocess import Popen, PIPE
import re
import time

#file paths, change these
calibration_prog_path = "./compasscal"
launch_file_path = "/home/cliff/catkin_ws/src/boat_nav/launch/imu.launch"
launch_file_backup_path = "/home/cliff/catkin_ws/src/boat_nav/launch/imu.old.launch"

output = ""
cur_index = 0
line_num = 1

print("Opening " + calibration_prog_path)
p = Popen([calibration_prog_path], stdin=PIPE, stdout=PIPE, shell=True)


def send_input(s):
	print("(I) "+s)
	p.stdin.write(s+"\n")

#read line, ouptut to console, append it to output var
def read_line():
	global output
	global cur_index
	global line_num

	line = p.stdout.readline()
	print("(%i) "%line_num + line.rstrip())
	line_num += 1
	output += line

def wait_for_string(s):
	global cur_index
	
	count = 0

	while True:
		# check for matching string
		substr_index = output.find(s, cur_index)
		if substr_index is -1:
			cur_index = len(output)-len(s)
		else:
			cur_index = substr_index
			return
		
		read_line()
		
		if count > 1000:
			raise Exception("Looped too many times, please check output above for abnormalities.")
		else:
			count+=1

def get_next_number():
	global cur_index
	
	count = 0

	while True:
		# collect digits until end of output
		while cur_index < len(output):
			num_str = ""
			for i in range(cur_index, cur_index+100):
				if i >= len(output):
					break
				if output[i].isdigit() or (output[i] is "." and (num_str.find(".") is -1)):
					num_str += output[i]
				elif len(num_str)>0:
					cur_index += len(num_str)
					return float(num_str)
				else:
					break;
			cur_index += 1
		
		read_line()

		if count > 1000:
			raise Exception("Looped too many times, please check output above for abnormalities.")
		else:
			count+=1

# -------------------- #

# for some reason we can only read from the compasscal program after it finishes outputting everything
# so we can't seem to print its outputs while it's running
.
print("Doing 3-axis calibration")
send_input("3")
print("Starting sampling...")
send_input("")
print("Move the compass around for sampling, press enter to finish.")
time.sleep(3)
raw_input()
print("Finishing sampling...")
send_input("")
print("Reading calibration program output:")
send_input("")
wait_for_string("Vars:")
values = []
for i in range(0,13):
	values.append(get_next_number())
wait_for_string("changed")

# -------------------- #

try:
	p.kill()
except OSError:
	pass

# -------------------- #

print("")
print("Parsed values from calibration program output:")
print(values)
print("Writing to launch file:")

infile = open(launch_file_path,'r')
temp_data = [line.strip() for line in infile]
infile.close()

print("Creating backup")
backupfile = open(launch_file_backup_path,'w')
backupfile.write('\n'.join(temp_data))
backupfile.close()
print("Backup created at "+launch_file_backup_path)

print("Changes:")
i = 0
for j in range(0,len(temp_data)):
	line = temp_data[j]
	if line.find("name=\"cc") is not -1:
		start_index = line.find("value=\"")
		if start_index is -1:
			raise Exception("launch file not formatted as expected, please no spaces between = and \" symbols")
		start_index += len("value=\"")
		end_index = line.find("\"/>")
		old_val = line[start_index:end_index]
		print (line[:start_index]+"   "+str(old_val)+" => "+str(values[i])+"   "+line[end_index:len(line)])
		temp_data[j] = line[:start_index]+str(values[i])+line[end_index:len(line)]
		i += 1

print("Saving launch file")
outfile = open(launch_file_path,'w')
outfile.write('\n'.join(temp_data))
outfile.close()
print("Saved.")
