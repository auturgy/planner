#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

PASSWORD="ciao"
CMD_SSH_CHECK "cat ${SRV_HOMEDIR}/dist/conf/ros_auth.conf | cut -f4 -d\\\" >${SRV_HOMEDIR}/dist/conf/auth_ros_local.conf"
CMD_SSH_CHECK "echo $PASSWORD | ${SRV_HOMEDIR}/dist/bin/passlib_ros.py >>${SRV_HOMEDIR}/dist/conf/auth_ros_local.conf"

###############################################################################
#EOF
