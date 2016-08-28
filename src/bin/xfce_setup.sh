#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

CMD_CHECK "sudo apt-get update"
CMD_CHECK "sudo apt-get -fy install xfce4 xfce4-goodies tightvncserver"

#####################################################################
#EOF
