#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

echo $SRV_TAGNAME

echo "Installing upgrade..."


CMD_SSH_CHECK "mkdir -p ${SRV_HOMEDIR}/dist/www"

CMD_CHECK "${UTILS_DIR}/build.sh $SRV_TAGNAME"
CMD_SSH_CHECK "sudo chown -R ${SRV_USERNAME}:${SRV_USERNAME} ${SRV_HOMEDIR}/dist/"
CMD_CHECK "${UTILS_DIR}/rsync.sh $SRV_TAGNAME"

# xxx this needs to be handled in javascript.
CMD_SSH_CHECK "echo 'planner_address = \"${SRV_ADDRESS}\";' >/home/ubuntu/dist/www/js/address.js"

CMD_SSH_CHECK "${SRV_HOMEDIR}/dist/bin/fixperm.sh"

###############################################################################
#EOF
