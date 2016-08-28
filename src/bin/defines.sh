#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################

RESTORE=$(echo -en '\033[0m')
RED=$(echo -en '\033[00;31m')
GREEN=$(echo -en '\033[00;32m')
WHITE=$(echo -en '\033[01;37m')
MAGENTA=$(echo -en '\033[01;36m')

function FATAL {
  echo
  echo "${RED}FATAL: $@; exiting. $RESTORE"
  echo
  exit 1
}

function SUCCESSFUL {
  echo "${GREEN}SUCCESSFUL $RESTORE"
}

function RESULT {
  echo "${WHITE}$@ $RESTORE"
}


#####################################################################

SRV_USERNAME="ubuntu"

#####################################################################

DIST_DIR="${SCRIPT_DIR}/../"
SOURCE_DIR="${DIST_DIR}/src/"
BIN_DIR="${DIST_DIR}/bin"
CONF_DIR="${DIST_DIR}/conf"
SRV_HOMEDIR="/home/$SRV_USERNAME"

#####################################################################

function CMD_CHECK {
    echo "${WHITE}Executing: $@ $RESTORE"
    sh -c "$@"
    ret=$?
    #sleep 1
    if [ ! $ret -eq 0 ] ; then FATAL "Executing: $@" ; else SUCCESSFUL ; fi
}


#####################################################################
#EOF
