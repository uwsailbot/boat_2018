cmake_minimum_required(VERSION 2.8.3)
project(boat_msgs)

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  message_generation
)

# Mark all messages in the 'msg' folder
add_message_files(DIRECTORY msg/)

# Mark all services in the 'srv' folder
add_service_files(DIRECTORY srv/)

# Mark all actions in the 'action' folder
add_action_files(DIRECTORY action/)

# Generate marked messages, services, and actions with their dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs actionlib_msgs message_runtime
)
