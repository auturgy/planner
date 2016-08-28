#! /usr/bin/python
# -*- coding: utf-8 -*-
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################


from .topic import Topic
from skysense.msg import *
import rospy
import socket

#####################################################################

class TopicUptime(Topic):
    msg_class = Uptime

    def __init__(self, topic, update_frequency):
        super(TopicUptime, self).__init__(topic = topic, update_frequency = update_frequency)

    def build_message(self):

        try:
            with open('/proc/uptime', 'r') as f:
                uptime_seconds = float(f.readline().split()[0])
        except:
            uptime_seconds = 0

        return self.msg_class(seconds = uptime_seconds)

#####################################################################
# EOF
