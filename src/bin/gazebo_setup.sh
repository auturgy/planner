#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

# install required packages
CMD_CHECK "sudo apt-get -fy install g++ python-pip python-matplotlib python-serial python-wxgtk2.8 python-scipy python-opencv python-numpy python-pyparsing ccache realpath libopencv-dev"
CMD_CHECK "sudo pip2 install pymavlink MAVProxy catkin_pkg --upgrade"


# compile a specific branch of ardupilot

CMD_CHECK "rm -rf ~/simulation"
CMD_CHECK "rm -rf /tmp/ArduCopter.build/"
CMD_CHECK "rm -rf /home/ubuntu/.ccache"

CMD_CHECK "mkdir -p ~/simulation"
CMD_CHECK "cd ~/simulation && git clone https://github.com/erlerobot/ardupilot"
CMD_CHECK "cd ~/simulation/ardupilot && git reset --hard fe72403"
CMD_CHECK "cd ~/simulation/ardupilot/ArduCopter && make sitl -j4"

# Getting latest version of JSBSim
CMD_CHECK "sudo apt-get -fy install libtool automake autoconf libexpat1-dev"
CMD_CHECK "cd ~/simulation && git clone git://github.com/tridge/jsbsim.git"
CMD_CHECK "cd ~/simulation/jsbsim && git reset --hard 57af008"

CMD_CHECK "cd ~/simulation/jsbsim && ./autogen.sh --enable-libraries"
CMD_CHECK "cd ~/simulation/jsbsim && make -j2"
CMD_CHECK "cd ~/simulation/jsbsim && sudo make install"

# Get rosinstall and some additional dependencies
CMD_CHECK "sudo apt-get -fy install python-rosinstall ros-indigo-octomap-msgs ros-indigo-joy \
  ros-indigo-geodesy ros-indigo-octomap-ros ros-indigo-mavlink ros-indigo-control-toolbox unzip"

# Add OSRF repository and install drcsim
CMD_CHECK "sudo sh -c 'echo \"deb http://packages.osrfoundation.org/drc/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/drc-latest.list'"
CMD_CHECK "wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add -"
CMD_CHECK "sudo apt-get update"
CMD_CHECK "sudo apt-get -fy install drcsim"


# build ROS system
CMD_CHECK "cd /home/ubuntu/dist/ws/src && git clone https://github.com/erlerobot/ardupilot_sitl_gazebo_plugin"
CMD_CHECK "cd /home/ubuntu/dist/ws/src/ardupilot_sitl_gazebo_plugin && git reset --hard 431b632"
CMD_CHECK "cd /home/ubuntu/dist/ws/src && git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/"
CMD_CHECK "cd /home/ubuntu/dist/ws/src/hector_gazebo && git reset --hard 5653b39"
CMD_CHECK "cd /home/ubuntu/dist/ws/src && git clone https://github.com/erlerobot/rotors_simulator -b sonar_plugin"
CMD_CHECK "cd /home/ubuntu/dist/ws/src/rotors_simulator && git reset --hard 7e1a97e"
CMD_CHECK "cd /home/ubuntu/dist/ws/src && git clone https://github.com/PX4/mav_comm.git"
CMD_CHECK "cd /home/ubuntu/dist/ws/src/mav_comm && git reset --hard 5e0a51c"
CMD_CHECK "cd /home/ubuntu/dist/ws/src && git clone https://github.com/ethz-asl/glog_catkin.git"
CMD_CHECK "cd /home/ubuntu/dist/ws/src/glog_catkin && git reset --hard eb23bbb"
CMD_CHECK "cd /home/ubuntu/dist/ws/src && git clone https://github.com/catkin/catkin_simple.git"
CMD_CHECK "cd /home/ubuntu/dist/ws/src/catkin_simple && git reset --hard 0e62848"
CMD_CHECK "cd /home/ubuntu/dist/ws/src && git clone https://github.com/erlerobot/mavros.git"
CMD_CHECK "cd /home/ubuntu/dist/ws/src/mavros && git reset --hard eb752c1"

CMD_CHECK "/bin/bash -c 'source /opt/ros/indigo/setup.bash && source /home/ubuntu/dist/ws/devel/setup.bash && \
  roscd && cd .. && catkin_make --pkg mav_msgs'"

# the previous command extends what's available in the ROS environment, and it's used in the next command.
# it is not possible to do 'catkin_make --pkg mav_msgs && catkin_make'.
# See http://forum.erlerobotics.com/t/solved-problem-with-error-invoking-make-j4-l4-failed/803/2

CMD_CHECK "/bin/bash -c 'source /opt/ros/indigo/setup.bash && source /home/ubuntu/dist/ws/devel/setup.bash && \
  roscd && cd .. && catkin_make'"

# install gazebo

CMD_CHECK "sudo sh -c 'echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main\" > /etc/apt/sources.list.d/gazebo-latest.list'"
CMD_CHECK "wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -"
CMD_CHECK "sudo apt-get update"
CMD_CHECK "sudo apt-get -fy install gazebo4 libgazebo4-dev"

# install erlerobotics models

CMD_CHECK "cd ~/simulation && git clone https://github.com/erlerobot/erle_gazebo_models"
CMD_CHECK "cd ~/simulation/erle_gazebo_models && git reset --hard bcaef64"

CMD_CHECK "rm -rf ~/.gazebo/models"
CMD_CHECK "mkdir -p ~/.gazebo/models"
CMD_CHECK "mv ~/simulation/erle_gazebo_models/* ~/.gazebo/models"
CMD_CHECK "rm -rf ~/simulation/erle_gazebo_models"

# include skysense ROS package in gazebo ros launch file
# by adding '<include file="$(find skysense)/launch/default.launch"></include>'
# before '</launch>'

tmp_pathname="/tmp/$RANDOM"
gazebo_launchfile="/home/ubuntu/dist/ws/src/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin/launch/erlecopter_spawn.launch"
CMD_CHECK "grep -v '</launch>' $gazebo_launchfile >$tmp_pathname"
CMD_CHECK "echo '<include file=\"\$(find skysense)/launch/default.launch\"></include>' >>$tmp_pathname"
CMD_CHECK "echo '</launch>' >>$tmp_pathname"
CMD_CHECK "cat $tmp_pathname >$gazebo_launchfile"

# add CAPRONI airport new point.
# locations format: lat,lon,alt,heading
locations_pathname="/home/ubuntu/simulation/ardupilot/Tools/autotest/locations.txt"
CMD_CHECK "echo 'CAPRONI=46.02073,11.12620,180,0' >>$locations_pathname"


# disable gazebo gui
# disable ros bag file logging
# disable warehouse 3d model
CMD_CHECK "cp /home/ubuntu/dist/data/erlecopter_spawn.launch /home/ubuntu/dist/ws/src/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin/launch/erlecopter_spawn.launch"

#####################################################################
#EOF
