# boat_2018
2018 UW Sailbot ROS Architecture

[![Build Status](https://travis-ci.org/uwsailbot/boat_2018.svg?branch=master)](https://travis-ci.org/uwsailbot/boat_2018)


To install, clone this repo as your src folder:

	$ cd catkin_ws
	$ git clone https://github.com/uwsailbot/boat_2018 src

Or you can clone it under your existing src folder, though this will result in a redundant CMakeList.txt.

Download package dependancies with the following command:

	$ cd catkin_ws
	$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

[See here](https://docs.google.com/spreadsheets/d/e/2PACX-1vRJPwn2XdzYAXsQpauvrPZ5q93W5B0C67GXXE5j3xL2SomwScCGoXGPIHMnfCvfF4DZ18LwXwg1yf4g/pubhtml) for a breakdown of the topics, nodes, launch files and custom messages used throughout this project.

[See Here]() (Coming soon) for the UWSailbot style guide.

## boat_description
Package for simulation of the entire code stack for debugging and testing, as well as live visualisation of the boat and its data.

## boat_interfaces
Package containing all nodes and files relevant to communication with the onboard arduino, the remote controller. Used to interface with onboard sensors and write values to motors. Package also supports remote control, without interfacing with any of the other packages.

## boat_msgs
Custom messages for path planning, gps coordinates and boat states in order to facilitate the communication of other packages.

## boat_nav 
Package for planning boat navigation given the specified task. This package is "the brains" of the boat, and decides how the boat should sail autonomously.

## boat_utilities
Package for various helper functions and utilities used throughout the codebase.

## boat_vision
Package for stereoscopic machine vision. This package is responsible for identifying objects in the water including obstacles (eg. Other boats, the shore, etc) and buoys used for navigation.

## Deprecated

Previously, this stack relied on the `phidgets_imu` package. This is no longer required. If you still wish to install it, clone https://github.com/ros-drivers/phidgets_drivers.git into your src and then rebuild:

	$ catkin build

Make sure to read the README.md in their repo and follow some extra steps to setup the phidgets package.
You need to download the library from this [link](https://www.phidgets.com/downloads/phidget22/libraries/linux/libphidget22.tar.gz) and you must run this in the libphidget folder to build it:
Run this in the libphidget folder to build it:
	
	$ ./configure --prefix=/usr && make && sudo make install

Also be sure to download the phidgets compass calibration program [See here](https://www.phidgets.com/?tier=3&catid=10&pcid=8&prodid=32)


The compass calibration script can be found in the tools section of the `boat_interfaces` package, along with a readme on how to use it.

If you want to download the python phidgets API [See here](https://www.phidgets.com/docs/Language_-_Python#Install_Phidget_Python_module_for_Linux)
