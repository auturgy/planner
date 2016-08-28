#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################
SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

mkdir -p ${SCRIPT_DIR}/../docs && \
rm -rf ${SCRIPT_DIR}/../docs/manual && \
cd ${SCRIPT_DIR}/../src/docs/manual && \
make clean html && \
cp -rv _build/html ../../../docs/manual
