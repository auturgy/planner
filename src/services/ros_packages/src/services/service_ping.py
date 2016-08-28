
#! /usr/bin/python
# -*- coding: utf-8 -*-
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################


from skysense.srv import * # here we import the classResponse
import rospy
import socket
import json
import subprocess

#####################################################################

class ServicePing:

    def __init__(self, service_name):
        self.service_name = service_name
        service = rospy.Service(service_name, PingPong, self.call)
        rospy.spin()

    def call(self, request):
        return PingPongResponse(pong = request.ping)

#####################################################################
# EOF
