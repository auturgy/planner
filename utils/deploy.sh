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

# set javascript variable with server address
#CMD_CHECK "echo \"srv_addr='${SRV_ADDRESS}'\" > ${SOURCE_DIR}/frontend/app/js/srv_addr.js"

cd ${ROOT_DIR} # required by build.py

CMD_SSH_CHECK "mkdir -p ${SRV_HOMEDIR}/dist/www"

CMD_CHECK "${UTILS_DIR}/build.sh $SRV_TAGNAME"
CMD_SSH_CHECK "sudo chown -R ${SRV_USERNAME}:${SRV_USERNAME} ${SRV_HOMEDIR}/dist/"
CMD_CHECK "${UTILS_DIR}/rsync.sh $SRV_TAGNAME"
#CMD_SSH_CHECK "sudo sh -c 'echo \"srv_addr = \\\"${SRV_ADDRESS}\\\";\" >${SRV_HOMEDIR}/dist/www/js/srv_addr.js'"

CMD_SSH_CHECK "echo 'planner_address = \"${SRV_ADDRESS}\";' >/home/ubuntu/dist/www/js/address.js"

CMD_SSH_CHECK "${SRV_HOMEDIR}/dist/bin/fixperm.sh"
#CMD_SSH_CHECK "sudo chown -R www-data:www-data ${SRV_HOMEDIR}/dist/www/ && \
#  sudo chmod -R g+rwX ${SRV_HOMEDIR}/dist/www/"

###############################################################################
#EOF
