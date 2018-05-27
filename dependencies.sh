# NOTE: This is a file to install dependancies for travis.  Rosdep currently
# installs most dependancies, but those that can't be added to the package.xml
# file must be added here,  to download from apt
sudo apt-get install python-pip
pip install freetype-py
pip install overrides
