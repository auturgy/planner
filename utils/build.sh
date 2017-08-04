#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
source "${SCRIPT_DIR}/defines.sh"
#####################################################################

cd ${ROOT_DIR}

CMD_CHECK "rm -rf ${ROOT_DIR}/dist/"
CMD_CHECK "mkdir -p dist"
CMD_CHECK "cp -rv src/bin src/conf src/services dist"
CMD_CHECK "mkdir dist/www"
CMD_CHECK "echo UP AND RUNNING > dist/www/index.html"
CMD_CHECK "cp -rv ${ROOT_DIR}/src/frontend/* dist/www"
CMD_CHECK "cp -rv ${ROOT_DIR}/src/data dist/"
CMD_CHECK "cp -rv ${ROOT_DIR}/src/docs/manual dist/www/"
echo "dist ready."

###############################################################################
#EOF
