cmake_minimum_required(VERSION 2.8.3)
project(boat_description)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
  std_msgs
  sensor_msgs
  boat_msgs
  tf
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs boat_msgs tf
)

# Mark executable scripts (ie. Those with a __main__) for installation
install(PROGRAMS
  src/sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# TODO: Unit testing

# Ensure launch files are valid
roslaunch_add_file_check(launch)
