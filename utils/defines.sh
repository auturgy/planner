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

# set server tagname to first argument
# shift 1 drops the first argument from $@
SRV_TAGNAME="$1"
shift 1

#####################################################################

if [ -z "$SRV_TAGNAME" ]; then FATAL "SRV_TAGNAME not defined"; fi

#####################################################################

ROOT_DIR="${SCRIPT_DIR}/../"
SOURCE_DIR="${ROOT_DIR}/src/"
UTILS_DIR=${SCRIPT_DIR}

SRV_HOMEDIR="/home/$SRV_USERNAME"
SRV_CREDENTIALS_DIR="${ROOT_DIR}/credentials/"

if [ ! -f "${SRV_CREDENTIALS_DIR}/${SRV_TAGNAME}.host" ]; then FATAL "Invalid SRV_TAGNAME '$SRV_TAGNAME'" ; fi

SRV_ADDRESS=$(cat ${SRV_CREDENTIALS_DIR}/${SRV_TAGNAME}.host | cut -f1 -d:)
SRV_ADDRESS_PORT=$(cat ${SRV_CREDENTIALS_DIR}/${SRV_TAGNAME}.host | cut -f2 -d:)
SRV_IDENTITY="${SRV_CREDENTIALS_DIR}/${SRV_TAGNAME}_key"
SRV_DESC="${SRV_TAGNAME} (${SRV_ADDRESS})"
SRV_SSH_ADDRESS="${SRV_USERNAME}@${SRV_ADDRESS}"
SRV_SSH_ADDRESS_HOME="${SRV_SSH_ADDRESS}:${SRV_HOMEDIR}"

# make sure that new credentials have the correct rights.
chmod 600 ${SRV_CREDENTIALS_DIR}/*

#####################################################################

function CMD_SCP_CHECK {
    echo "${MAGENTA}Executing: scp -i $SRV_IDENTITY -P $SRV_ADDRESS_PORT $@ $RESTORE"
    scp -i $SRV_IDENTITY -P $SRV_ADDRESS_PORT $@
    ret=$?
    if [ ! $ret -eq 0 ] ; then FATAL "request failed" ; else SUCCESSFUL ; fi
}

function CMD_SSH {
    ssh -i $SRV_IDENTITY -p $SRV_ADDRESS_PORT $SRV_SSH_ADDRESS $@
    return $?
}

function CMD_SCP {
    scp -i $SRV_IDENTITY -P $SRV_ADDRESS_PORT $@
    return $?
}

function CMD_SSH_CHECK {
    echo "${MAGENTA}Executing: ssh -i $SRV_IDENTITY -p $SRV_ADDRESS_PORT $SRV_SSH_ADDRESS $@ $RESTORE"
    ssh -i $SRV_IDENTITY -p $SRV_ADDRESS_PORT $SRV_SSH_ADDRESS $@
    ret=$?
    #sleep 1
    if [ ! $ret -eq 0 ] ; then FATAL "request failed" ; else SUCCESSFUL ; fi
}

function CMD_CHECK {
    echo "${MAGENTA}Executing: $@ $RESTORE"
    sh -c "$@"
    ret=$?
    #sleep 1
    if [ ! $ret -eq 0 ] ; then FATAL "request failed" ; else SUCCESSFUL ; fi
}


function simplepath {
    directory="$(dirname $1)"
    basename=$(basename $1)
    cd $directory
    ret=$(pwd)
    cd - >/dev/null
    echo ${ret}/$basename
}

#####################################################################
#EOF
