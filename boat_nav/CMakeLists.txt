cmake_minimum_required(VERSION 2.8.3)
project(boat_nav)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
  actionlib
  std_msgs
  boat_msgs
  tf
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy actionlib std_msgs boat_msgs tf
)

# Mark executable scripts (ie. Those with a __main__) for installation
install(PROGRAMS
  src/path_planner_node.py
  src/navigator_node.py
  src/sensor_parser_node.py
  src/max_vmg_action.py
  src/layline_action.py
  src/irons_action.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# TODO: Unit testing

# Ensure launch files are valid
roslaunch_add_file_check(launch)
