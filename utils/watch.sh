#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################


# re-build and deploy once...
${UTILS_DIR}/deploy.sh $SRV_TAGNAME

# CTRL-C interrupts the script.
trap ctrl_c INT
function ctrl_c() {
        echo "** Trapped CTRL-C"
        exit 1
}

# main loop...
while :
do
  echo Monitoring changes in directory $SOURCE_DIR ...
  fswatch --one-event $SOURCE_DIR
  ${UTILS_DIR}/deploy.sh $SRV_TAGNAME
  sleep 1
done

###############################################################################
#EOF
