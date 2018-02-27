# boat_2018
2018 UW Sailbot ROS Architecture

To build, clone this repo as your src folder:

	$ cd catkin_ws
	$ git clone https://github.com/uwsailbot/boat_2018 src

Or you can clone it under your existing src folder, though this will result in a redundant CMakeList.txt.


[See here](https://docs.google.com/spreadsheets/d/e/2PACX-1vRJPwn2XdzYAXsQpauvrPZ5q93W5B0C67GXXE5j3xL2SomwScCGoXGPIHMnfCvfF4DZ18LwXwg1yf4g/pubhtml) for a breakdown of the topics, nodes, launch files and custom messages used throughout this project.

[See Here]() (Coming soon) for the UWSailbot style guide.

## boat_description
Visualisation and simulation for testing of the functionality of other packages.  COMING SOON.

## boat_interfaces
Package containing all nodes and files relevant to communication with the onboard arduino, the remote controller.  Used to interface with onboard sensors and write values to motors.  Package also supports remote control, without interfacing with any of the other packages.

## boat_nav 
Package for planning boat path given gps waypoints, as well as controlling the onboard IMU and doing neccessary position filtering.  The package "that is the brains" and decides how the boat should sail autonomously.  NOTE: required compass calibration for use in using the imu can be found in the tools section of the boat_interfaces package, along with a readme on how to use it.

 Download package dependancies with the following command:

	$ cd catkin_ws
	$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
	
This package relies on phidgets_imu in order to compile, which won't get downloaded with the previous command.  Clone it from https://github.com/ros-drivers/phidgets_drivers.git and then run the following:
	$ cd catkin_ws
	$ catkin_make

Make sure to read the README.md in their repo and follow some extra steps to setup the phidgets package.
You need to download the library from this [link](https://www.phidgets.com/downloads/phidget22/libraries/linux/libphidget22.tar.gz) and you must run this in the libphidget folder to build it:
Run this in the libphidget folder to build it:
	
	$ ./configure --prefix=/usr && make && sudo make install

Also be sure to download the phidgets compass calibration program [See here](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=32)

### Phidgets API (OPTIONAL)
If you want to download the python phidgets API [See here](https://www.phidgets.com/docs/Language_-_Python#Install_Phidget_Python_module_for_Linux)

## boat_msgs
Custom messages for path planning, gps coordinates and boat states in order to facilitate the communication of other packages.
