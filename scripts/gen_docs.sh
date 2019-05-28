#!/bin/bash

#NOTE: Must be called from the catkin_ws/src directory
pip install doxypypy
doxygen doxygen/Doxyfile
