#! /usr/bin/python
# -*- coding: utf-8 -*-
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################


import rospy
from std_msgs.msg import Header

#####################################################################

class Topic(object):
    """ Base class for topics. """

    def __init__(self, topic, update_frequency, **kwargs):
        self.topic = topic

        rospy.loginfo("Rate seconds for topic %s = %d" % (str(topic), update_frequency))

        pub = rospy.Publisher(self.topic, self.msg_class, latch=True, queue_size=10)

        rate = rospy.Rate(1.0/float(update_frequency))
        while not rospy.is_shutdown():
            m = self.build_message()
            # classes can stop the publication by returning False
            if m:
                if hasattr(m, 'header'):
                    m.header = Header()
                    m.header.stamp = rospy.Time.now()
                #rospy.loginfo(m)
                pub.publish(m)
            rate.sleep()

        self.cleanup()


    def cleanup(self):
        pass

    def build_message(self):
        raise NotImplementedError("build_message should be overriden in topic classes")
