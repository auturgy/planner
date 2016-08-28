#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################

#make sure that we run the script as root
if [ "$(id -u)" != "0" ]; then
  echo "Not root, restarting as root ..."
  sudo $0
  exit 0
fi

# stop ROS
echo "Stopping ROS if running...."
# not stopping ROS may result in a broken ROS setup.
supervisorctl stop ros
echo "Waiting 10 seconds ..."
sleep 10
#

# add bashrc
grep "magictoken" /home/ubuntu/.bashrc >/dev/null
ret="$?"

if [ "$ret" -eq "1" ]
then
  echo "Adding source to bashrc ..."
  echo "## magictoken" >>/home/ubuntu/.bashrc
  echo "source /home/ubuntu/dist/conf/bashrc.source" >>/home/ubuntu/.bashrc
fi


# enforce correct file permissions
/home/ubuntu/dist/bin/fixperm.sh

# init and build ROS instance
export LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8
source /opt/ros/indigo/setup.bash
export ROS_LANG_DISABLE=genlisp:gencpp

rm -rf /home/ubuntu/dist/ws/ && \
mkdir -p /home/ubuntu/dist/ws/src && \
cd /home/ubuntu/dist/ws/src && \
ln -s ../../services/ros_packages/ && \
catkin_init_workspace && \
cd .. && \
catkin_make


# enforce correct file permissions (again)
/home/ubuntu/dist/bin/fixperm.sh

# sync disk
sync

#####################################################################
#EOF
