#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

# Example: utils/startline.sh ros1 setup.sh 70

scriptname=$1
line=$2

echo "Executing script ${SCRIPT_DIR}/$scriptname from line $line ..."

tmp_script="/${SCRIPT_DIR}/startline_${RANDOM}.sh"
sed -n "${line},\$p" "${SCRIPT_DIR}/${scriptname}" >$tmp_script
chmod +x $tmp_script

source $tmp_script

rm $tmp_script

#####################################################################
#EOF
