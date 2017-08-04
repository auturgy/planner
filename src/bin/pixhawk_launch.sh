#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################

if [ "$(id -u)" != "0" ]; then
  echo "Not root, restarting as root ..."
  sudo $0
  exit 0
fi

# launch ROS
export LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8
source /opt/ros/indigo/setup.bash
source /home/ubuntu/dist/ws/devel/setup.bash

cd /home/ubuntu/dist/ws

trap 'kill -TERM $PID; exit' TERM INT
roslaunch skysense pixhawk.launch &
PID=$!
wait $PID
trap - TERM INT
wait $PID
EXIT_STATUS=$?

while :
do
    sleep 1
done




###############################################################################
#EOF
