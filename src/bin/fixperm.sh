#!/bin/bash
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################

sudo chown -R ubuntu:ubuntu /home/ubuntu/
sudo chown -R www-data:www-data /home/ubuntu/dist/www
sudo chmod -R g+rwX /home/ubuntu/dist/www
sudo chmod +x /home/ubuntu/dist/bin/*.sh
sudo chmod +x /home/ubuntu/dist/ws/src/ros_packages/src/*.py 2>/dev/null


# ignore errors, always returns 0.
exit 0

#####################################################################
#EOF
