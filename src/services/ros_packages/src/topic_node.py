#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################

#####################################################################

import rospy
import sys
from topics.topic_uptime import TopicUptime
from topics.topic_radar import TopicRadar
from topics.topic_controller import TopicController

#####################################################################

def main(argv):
    # Keep in mind that the node name passed here is just a default and will be
    # overridden by the "name" parameter in the launch file
    rospy.init_node('topic_node')

    # Obtain the topic root from runtime paramters
    topic_root = rospy.get_param('~topic_root')

    # Obtain the class name from runtime parameters
    cls_name = rospy.get_param('~topic_class')

    # frequency, in seconds, of published topic
    update_frequency = rospy.get_param('~update_frequency')

    # resolve and instantiate class
    topic_class = globals()[cls_name]
    topic = topic_class(topic=topic_root, update_frequency = update_frequency)



#####################################################################

if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv))
    except KeyboardInterrupt:
        print "Ctrl-c pressed ..."
        sys.exit(1)

#####################################################################
