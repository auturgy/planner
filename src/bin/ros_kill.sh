#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################

sudo kill $(ps aux | grep /opt/ros | tr -s ' ' | cut -f2 -d' ')

# EOF
