#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

RSYNC_EXCLUDE=${UTILS_DIR}/rsync.exclude

echo "Synching dist/ to $SRV_DESC ..."

rsync --checksum -l --exclude-from "$RSYNC_EXCLUDE" -avzL -e "ssh -i $SRV_IDENTITY -p $SRV_ADDRESS_PORT" \
    ${ROOT_DIR}/dist ${SRV_SSH_ADDRESS_HOME}



###############################################################################
#EOF
