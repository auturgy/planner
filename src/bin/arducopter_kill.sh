#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# Author: Michele Dallachiesa
#####################################################################

sudo kill $(ps aux | grep /ardupilot/ | tr -s ' ' | cut -f2 -d' ')

# EOF
