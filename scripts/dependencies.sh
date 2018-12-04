#!/bin/bash

# NOTE: This is a file to install dependencies for Travis.  Rosdep currently
# installs most dependencies, but those that can't be added to the package.xml
# file must be added here, to download from apt
sudo apt install python-pip
pip install freetype-py
pip install overrides
pip install doxypypy # Not needed for Travis, but needed for Doxygen
pip install opencv-python
pip install yapf
