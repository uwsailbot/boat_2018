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

This package relies on phidgets_imu and imu_filter_madgwick in order to compile.  Clone it from https://github.com/ros-drivers/phidgets_drivers.git and https://github.com/ccny-ros-pkg/imu_tools (into your catkin_ws/src folder) or download both using (if the following command doesn't work, just download the above git repos):

	$ rosdep install boat_nav

Either way make sure to read the README.md in their repo and follow some extra steps.

You must download the python phidgets API [See here](https://www.phidgets.com/docs/Language_-_Python#Install_Phidget_Python_module_for_Linux)

Run this in the libphidget folder to build it:

	$ ./configure --prefix=/usr && make && sudo make install

Also be sure to download the phidgets compass calibration program [See here](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=32)

## boat_msgs
Custom messages for path planning, gps coordinates and boat states in order to facilitate the communication of other packages.
