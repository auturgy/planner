#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

if [ "$(id -u)" != "0" ]; then
  echo "Not root, restarting as root ..."
  sudo $0
  exit 0
fi

export LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8

export HOME=/home/ubuntu # required by gazebo.
source /opt/ros/indigo/setup.bash
source /home/ubuntu/dist/ws/devel/setup.bash
cd /home/ubuntu/dist/ws

function terminate() {
  echo "Forcing termination of gazebo ..."
  /home/ubuntu/dist/bin/ros_kill.sh
  exit
}

# if INT or TERM signals received, call function 'terminate'.
trap terminate INT TERM

roslaunch ardupilot_sitl_gazebo_plugin erlecopter_spawn.launch &

# avoid logs:
# http://answers.ros.org/question/9627/how-can-i-completely-disable-writing-logs-to-filesystem/
#sleep 1
#rosclean purge -y


# continue forever, to keep alive the trap on TERM and INT signals.
while true; do sleep 1 ; done



#####################################################################
#EOF
