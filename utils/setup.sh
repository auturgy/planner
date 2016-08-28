#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

# clean prior setup/installation attempts
CMD_CHECK "${UTILS_DIR}/ssh.sh $SRV_TAGNAME 'sudo rm -rf /home/ubuntu/dist /home/ubuntu/simulation'"

# basic system setup
CMD_CHECK "${UTILS_DIR}/system_setup.sh $SRV_TAGNAME"

# install demo files
CMD_CHECK "${UTILS_DIR}/deploy.sh $SRV_TAGNAME"

# setup and install ros
CMD_CHECK "${UTILS_DIR}/ssh.sh $SRV_TAGNAME /home/ubuntu/dist/bin/ros_setup.sh"

# setup and install gazebo and arducopter
CMD_CHECK "${UTILS_DIR}/ssh.sh $SRV_TAGNAME /home/ubuntu/dist/bin/gazebo_setup.sh"

#####################################################################
#EOF
