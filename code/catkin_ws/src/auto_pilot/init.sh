#/bin/bash

set -e # Exit immediately if a command exits with a non-zero status.

# SET THESE VARIABLES BEFORE RUNNING THIS SCRIPT (should be left as is in principle)
webots_path='/usr/local/webots'
python_version='' # should be left blank if using the default python version (38)
ros_version='noetic' # melodic, kinetic, etc... 

##########################
## Install dependencies ##
##########################

# Ask if should update python and ros packages
read -p "Do you want to install/update required packages? (y/n) " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
    # Update python packages
    echo 
    echo "Python packages will be updated"
    python3 -m pip install --upgrade pip
    echo "Pip version: "$(pip3 -V)
    pip3 install numpy
    pip3 install numba
    pip3 install --upgrade colorama 
    pip3 install pandas
    pip3 install scipy

    # Install ROS Noetic packages 
    echo 
    echo "Installing ROS $ros_version packages"
    sudo apt install ros-$ros_version-webots-ros
    sudo apt install ros-$ros_version-mavros ros-$ros_version-mavros-extras
fi

#############################################################
## Set the environment variables for the autopilot project ##
#############################################################

echo >> ~/.bashrc
echo '### Autopilot configurations ###' >> ~/.bashrc 
echo >> ~/.bashrc 

# Add autopilot scripts to PYTHONPATH
echo 'export PYTHONPATH=$PYTHONPATH:'$(pwd)/scripts >> ~/.bashrc 

# Set up Webots with ROS and Python3 
echo "export WEBOTS_HOME=$webots_path" >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/webots:$WEBOTS_HOME/lib/controller' >> ~/.bashrc
echo 'export PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python'$python_version >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=$WEBOTS_HOME/projects/default/controllers/ros:$ROS_PACKAGE_PATH' >> ~/.bashrc

echo 
echo 'Added environment variables for the autopilot project at the end of ~/.bashrc'