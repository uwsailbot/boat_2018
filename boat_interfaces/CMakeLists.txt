cmake_minimum_required(VERSION 2.8.3)
project(boat_interfaces)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  roslaunch
  actionlib
  std_msgs
  boat_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy actionlib std_msgs boat_msgs
)

# Mark executable scripts (ie. Those with a __main__) for installation
install(PROGRAMS
  src/rudder_node.py
  src/winch_node.py
  src/tacking_action.py
  src/pid.py
  src/boat_state_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Mark config files for installation
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Mark challenge files for installation
install(DIRECTORY challenges/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/challenges
)

# TODO: Unit testing

# Ensure launch files are valid
roslaunch_add_file_check(launch)
