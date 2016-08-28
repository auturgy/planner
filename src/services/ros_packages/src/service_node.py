#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################

#####################################################################

# Import all services here so they're available in globals()

import rospy
import sys
from services.service_ping import ServicePing
#from services.service_controller_land import ServiceControllerLand

#####################################################################

def main(argv):
    # Keep in mind that the node name passed here is just a default and will be
    # overridden by the "name" parameter in the launch file
    rospy.init_node("service_node")

    # Obtain the class and service names from runtime parameters
    cls_name = rospy.get_param('~service_class')
    service_name = rospy.get_param('~service_name')

    # resolve and instantiate class
    service_class = globals()[cls_name]
    service = service_class(service_name = service_name)

#####################################################################

if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv))
    except KeyboardInterrupt:
        print "Ctrl-c pressed ..."
        sys.exit(1)

#####################################################################
