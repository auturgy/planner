#! /usr/bin/python
# -*- coding: utf-8 -*-
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################


from .topic import Topic
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import BatteryStatus
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointClear
from std_msgs.msg import String, Float64
from skysense.msg import *
import rospy
import json
import random
from functools import partial

import numpy as np
import matplotlib as mp
from matplotlib.path import Path as mp_path



#import message_filters
#topic_radar = message_filters.Subscriber("/device/radar", String)
#topic_uptime = message_filters.Subscriber("/device/uptime", Uptime)
#ts=message_filters.ApproximateTimeSynchronizer([topic_radar,topic_uptime],10,1, allow_headerless = True)
#ts.registerCallback(callback)

#####################################################################

import time

class CheckState:

    @staticmethod
    def critical(name, desc):
        return CheckState(name = name, state = "CRITICAL", desc = desc)

    @staticmethod
    def warning(name, desc):
        return CheckState(name = name, state = "WARNING", desc = desc)

    @staticmethod
    def ok(name):
        return CheckState(name = name, state = "OK", desc = "")

    def __init__(self, name, state, desc):
        self.name = name
        self.desc = desc

        self.ok = False
        self.critical = False
        self.warning = False
        self.state = state
        if state == "OK":
            self.ok = True
        if state == "WARNING":
            self.warning = True
        if state == "CRITICAL":
            self.critical = True

    def __str__(self):
        return "%s: %s" % (self.name, self.desc)


class TimeMsg:

    def __init__(self, msg):
        self.time = time.time()
        self.msg = msg

class TopicController(Topic):
    msg_class = String

    def callback(self, msg, path):
        #rospy.loginfo("MSG path=%s msg=%s" % (str(path), str(msg)))
        self.last[path] = TimeMsg(msg)

    def track_topic(self, path, type):
        self.last[path] = TimeMsg(None)
        f = partial(self.callback, path = path)
        rospy.Subscriber(path, type, f)

    def __init__(self, topic, update_frequency):

        self.checks = []
        self.last = {}

        self.track_topic(path = "/device/radar", type = String)
        self.track_topic(path = "/mavros/global_position/global", type = NavSatFix)
        self.track_topic(path = "/mavros/global_position/rel_alt", type = Float64)
        self.track_topic(path = "/mavros/battery", type = BatteryStatus)
        self.track_topic(path = "/mavros/state", type = State)
        self.track_topic(path = "/client/heartbeat", type = String)
        self.track_topic(path = "/client/geofence", type = String)
        self.track_topic(path = "/mavros/global_position/rel_alt", type = Float64)

        self.register_check(self.check_drone)
        self.register_check(self.check_gps)
        self.register_check(self.check_battery)
        self.register_check(self.check_client)
        self.register_check(self.check_radar)
        self.register_check(self.check_geofence)
        self.register_check(self.check_waypoints)

        self.prev_client_heartbeat = None

        self.emergency_landing_requested = False

        super(TopicController, self).__init__(topic = topic, update_frequency = update_frequency)

    def armed(self):
        if self.last["/mavros/state"].msg == None:
            return False
        return self.last["/mavros/state"].msg.armed

    def landed(self):
        return not self.armed()


    def register_check(self, f):
        self.checks.append(f)

    def emergency_landing(self):
        if self.emergency_landing_requested:
            return

        if self.landed():
            rospy.loginfo("already landed")
            return

        rospy.loginfo("emergency landing")

        self.emergency_landing_requested = True

        try:
            rospy.wait_for_service('/mavros/set_mode')
            rospy.wait_for_service('/mavros/mission/clear')
            rospy.wait_for_service('/mavros/cmd/land')

            mavros_set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            mavros_waypoints_clear = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)
            mavros_land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

            ret = mavros_set_mode(custom_mode = "STABILIZE")
            rospy.loginfo("ret mavros_set_mode = %s" % str(ret))

            ret = mavros_waypoints_clear()
            rospy.loginfo("ret mavros_waypoints_clear = %s" % str(ret))

            ret = mavros_land(min_pitch = 0.0, yaw = 0.0, latitude = 0.0, longitude = 0.0, altitude = 0.0)
            rospy.loginfo("ret mavros_land = %s" % str(ret))

        except Exception as e:
            traceback.print_exc()





    def check_all(self):
        count_critical = 0
        count_warning = 0
        ret = True
        self.reports = []

        if self.landed(): # xxx
            return True

        for check in self.checks:
            report = check()
            self.reports.append(report)
            if not report.ok:
                if report.critical:
                    self.emergency_landing()
                ret = False
                # do not break here! we want to run all checks anyways.

    # xxx
    #    if count_critical > 0:
    #        self.emergency_landing()
    #    else:
    #        self.emergency_landing_requested = False

        return ret

    def get_critical_reports(self):

        reports = []

        for report in self.reports:
            if report.critical:
                reports.append(report)

        return reports

    def get_warning_reports(self):

        reports = []

        for report in self.reports:
            if report.warning:
                reports.append(report)

        return reports


    def check_drone(self):
        return CheckState.ok("DRONE")

    def check_client(self):

        return CheckState.ok("CLIENT")



        if self.last["/client/heartbeat"].msg == None:
            return CheckState.critical("CLIENT", "No client heartbeat")

#        if self.prev_client_heartbeat == None:
#        self.prev_client_heartbeat = json.loads(self.last["/client/heartbeat"].msg.data).count
#        rospy.loginfo("prev : %d" % self.prev_client_heartbeat)

        rospy.loginfo("time: %f last: %f" % (time.time(),self.last["/client/heartbeat"].time ))
        if time.time() - self.last["/client/heartbeat"].time > 20:
            return CheckState.critical("CLIENT", "Heartbeat not received for 20 seconds.")
        else:
            return CheckState.ok("CLIENT")

    def check_radar(self):
        return CheckState.ok("RADAR")
        # https://pypi.python.org/pypi/LatLon

    def check_geofence(self):

    #    self.last["/client/geofence"] = TimeMsg({"coordinates":[{"0":{"lat":46.020083634747635,"lng":11.124015656212007}},{"1":{"lat":46.02184291741568,"lng":11.123612692025473}},{"2":{"lat":46.022825756895024,"lng":11.126207388250943}},{"3":{"lat":46.02039814338102,"lng":11.127917528944995}},{"4":{"lat":46.019061481689114,"lng":11.125362146298704}},{"5":{"lat":46.01922856440058,"lng":11.12405496979118}}]})

        if self.last["/mavros/global_position/global"].msg == None:
            return CheckState.critical("GEOFENCE", "No GPS data")

        if self.last["/client/geofence"].msg == None:
            return CheckState.critical("GEOFENCE", "No geofence data") # geofence not set

        # todo: check that GPS has fix, and check how old is the last location.

        drone_pos = [self.last["/mavros/global_position/global"].msg.latitude,self.last["/mavros/global_position/global"].msg.longitude]
        drone_pos = np.array(drone_pos)

        coordinates = json.loads(self.last["/client/geofence"].msg.data)["coordinates"]

        if len(coordinates) == 0: # empty geofence
            return CheckState.ok("GEOFENCE")

#        rospy.loginfo("COORDINATES: %s" % str(coordinates))
#        return CheckState.ok("GEOFENCE")

        polygon = []
        for pos in coordinates:
            pos = pos.values()[0]
            #rospy.loginfo("%s %s", pos["lat"],pos["lng"])
            polygon.append([pos["lat"],pos["lng"]])
        polygon = np.array(polygon)

#        rospy.loginfo("drone_pos = %s" % str(drone_pos))
#        rospy.loginfo("polygon = %s" % str(polygon))

        path = mp_path(polygon)

        if path.contains_point(drone_pos):
            return CheckState.ok("GEOFENCE")
        else:
            return CheckState.critical("GEOFENCE", "Drone outside geofence")

#        rospy.loginfo("Drone inside" if inside else "Drone outside")


    def check_battery(self):

        if(self.last["/mavros/battery"].msg == None):
            return CheckState.critical("BATTERY", "No data")

        if self.last["/mavros/battery"].msg.remaining < 0.2:
            return CheckState.critical("BATTERY", "Battery level below 20%")
        else:
            return CheckState.ok("BATTERY")


    def check_gps(self):

        if(self.last["/mavros/global_position/global"].msg == None):
            return CheckState.critical("GPS", "No data")

        if self.last["/mavros/global_position/global"].msg.status.status < 0:
            return CheckState.critical("GPS", "No GPS Fix")
        else:
            return CheckState.ok("GPS")

    def check_waypoints(self):
        return CheckState.ok("WAYPOINTS")

    def build_message(self):

        msg = {}
        self.check_all()

        if len(self.get_critical_reports()) >0:
            msg["state"] = "CRITICAL"
        elif len(self.get_warning_reports()) >0:
            msg["state"] = "WARNING"
        else:
            msg["state"] = "OK"

        msg["criticals"] = []
        for critical_report in self.get_critical_reports():
            msg["criticals"].append(str(critical_report))

        msg["warnings"] = []
        for warning_report in self.get_warning_reports():
            msg["warnings"].append(str(warning_report))

        rospy.loginfo(str(msg))

        return self.msg_class(data = json.dumps(msg))

#####################################################################
# EOF
