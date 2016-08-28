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
source /opt/ros/indigo/setup.bash
source /home/ubuntu/dist/ws/devel/setup.bash

#cmd_extra=""
#cmd_extra="$cmd_extra param load /home/ubuntu/simulation/ardupilot/Tools/Frame_params/Erle-Copter.param ; "
#cmd_extra="$cmd_extra param set ARMING_CHECK 0 ;"
#cmd_extra="$cmd_extra arm safetyoff;"
#cmd_extra="$cmd_extra mode GUIDED ;"

location="CAPRONI"

cd /home/ubuntu/simulation/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo -L $location
#--cmd="script /home/ubuntu/dist/conf/mavproxy.conf"

#--cmd="\"$cmd_extra\""


# --map --console

#####################################################################
#EOF
