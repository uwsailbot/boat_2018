#/bin/bash
source /opt/ros/kinetic/setup.bash
cd /tmp/
rosrun rosserial_arduino make_libraries.py .
platformio ci --board=uno --lib=lib --lib="/tmp/ros_lib
