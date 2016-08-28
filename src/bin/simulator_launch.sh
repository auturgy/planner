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

function terminate() {
  echo "Forcing termination of simulator ..."
  sudo supervisorctl stop gazebo arducopter
  exit
}


trap terminate INT TERM

sudo supervisorctl start arducopter
sleep 5
sudo supervisorctl start gazebo

while :
do
    sleep 1
done





#####################################################################
#EOF
