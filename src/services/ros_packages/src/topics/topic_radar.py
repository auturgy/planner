#! /usr/bin/python
# -*- coding: utf-8 -*-
#####################################################################
# Copyright (c) 2016 Skysense Inc.
# License: GNU General Public License v3
# Author: Michele Dallachiesa
#####################################################################


from .topic import Topic
from std_msgs.msg import String
import rospy
import socket
import json
import random
import socket
from threading import Thread
import time
import traceback
from LatLon import *

try:
    import asyncio
except ImportError:
    # Trollius >= 0.3 was renamed
    import trollius as asyncio

import threading

#####################################################################

class AircraftsDB():

    class Aircraft:
        def __init__(self):
            self.hexident = ""
            self.callsign = ""
            self.altitude = ""
            self.latitude = ""
            self.longitude = ""
            self.lastseen = time.time()
            self.last2positions = []
            self.direction = "N" # N NE E SE S SW W NW
            self.heading = 0

        def last(self):
            return int(time.time() - self.lastseen)

        def id(self):
            aircraft_id = self.hexident
            if self.callsign != "":
                aircraft_id+= " (%s)" % self.callsign
            return aircraft_id

        def init(self, hexident, callsign, altitude, latitude, longitude):
            self.hexident = hexident
            self.callsign = callsign
            self.altitude = altitude
            self.latitude = latitude
            self.longitude = longitude

        def update_direction(self):

            if self.latitude == "" or self.longitude == "":
                return # there's no position data

#            rospy.loginfo("%s %s Current latlon: %s,%s" % (id(self), self.hexident, self.latitude, self.longitude))
#            rospy.loginfo("%s Current direction: %s" % (self.hexident, str(self.last2positions)))

            if len(self.last2positions) == 1:
                if self.last2positions[0] == (self.latitude, self.longitude):
                    return

            if len(self.last2positions) == 2:
                if self.last2positions[1] == (self.latitude, self.longitude):
                    return
                else:
                    self.last2positions.pop(0)

            self.last2positions.append((self.latitude, self.longitude))

            if len(self.last2positions) == 2: # determine direction!
                p1 = LatLon(Latitude(self.last2positions[0][0]), Longitude(self.last2positions[0][1]))
                p2 = LatLon(Latitude(self.last2positions[1][0]), Longitude(self.last2positions[1][1]))
                self.heading = p1.heading_initial(p2)

                if self.heading < 0:
                    self.heading+= 360

                if self.heading > 337.5 or self.heading < 22.5:
                    self.direction = "N"
                elif 22.5 < self.heading < 67.5:
                    self.direction = "NE"
                elif 67.5 < self.heading < 112.5:
                    self.direction = "E"
                elif 112.5 < self.heading < 157.5:
                    self.direction = "SE"
                elif 157.5 < self.heading < 202.5:
                    self.direction = "S"
                elif 202.5 < self.heading < 247.5:
                    self.direction = "SW"
                elif 247.5 < self.heading < 292.5:
                    self.direction = "W"
                elif 292.5 < self.heading < 337.5:
                    self.direction = "NW"

            #    rospy.loginfo("Points: p1=%s p2=%s" % (p1,p2))
                rospy.loginfo("Radar: New heading for hexident=%s: %f %s" % (self.hexident, self.heading, self.direction))

            #rospy.loginfo("%s New direction: %s" % (self.hexident, str(self.last2positions)))




        def __str__(self):
            return "[%s:%s] alt:%s latlon:%s,%s last:%ds pair:%s" % (self.hexident, self.callsign, self.altitude, self.latitude, self.longitude, self.last(), str(self.latlon_pair))

        def merge(self, aircraft):
            assert self.hexident == aircraft.hexident

            if aircraft.callsign != "":
                self.callsign = aircraft.callsign.strip()

            if aircraft.altitude != "":
                self.altitude = aircraft.altitude.strip()

            if aircraft.latitude != "":
                self.latitude = aircraft.latitude.strip()

            if aircraft.longitude != "":
                self.longitude = aircraft.longitude.strip()

            self.lastseen = time.time()
            self.update_direction()

        def looksgood(self):
            if self.latitude != "" and self.longitude != "":
                return True



    def __init__(self):
        self.aircrafts = {}
        self.aircraft_timeout = 60

    def add(self, line):

    #    rospy.loginfo("Adding new line: %s" % line)

        fields = line.split(",")
        message_type = fields[0]
        transmission_type = fields[1]

#        rospy.loginfo("Present %d fields" % len(fields))

        aircraft = AircraftsDB.Aircraft()
        aircraft.init(hexident = fields[4], callsign = fields[10], altitude = fields[11], latitude = fields[14], longitude = fields[15])

        if aircraft.hexident in self.aircrafts:
        #    rospy.loginfo("Aircraft merged: %s" % aircraft.hexident)
            self.aircrafts[aircraft.hexident].merge(aircraft)
        else:
        #    rospy.loginfo("Aircraft added: %s" % aircraft.hexident)
            aircraft.update_direction()
            self.aircrafts[aircraft.hexident] = aircraft

        #rospy.loginfo("Line: %s" % line)
        #rospy.loginfo("Aircraft: %s" % self.aircrafts[aircraft.hexident])

    def check_timeout(self):

        now = time.time()

        hexidents = self.aircrafts.keys()
        for hexident in hexidents:
            if now - self.aircrafts[hexident].lastseen > self.aircraft_timeout:
                self.aircrafts.pop(hexident, None)



class LineBuffer:
    def __init__(self):
        self.b = ""

    def recv(self, data):
        self.b+= data

    def readline(self):
        eol = self.b.find('\n')
        if eol == -1:
            return None
        else:
            l = self.b[:eol]
            self.b = self.b[eol+1:]
            return l.strip() # take care of the optional \r

    def readlines(self):

        lines = []

        while True:
            l = self.readline()
            if l == None:
                break
            lines.append(l)

        return lines

    def test():
        lb = LineBuffer()
        lb.recv("aaaXbbXc")
        print lb.readline()
        print lb.readline()
        lb.recv("dddXeee")
        print lb.readline()



class Radar1090(Thread):

    class RadarProtocol(asyncio.Protocol):

        def __init__(self, thread):
            self.thread = thread
            super(Radar1090.RadarProtocol, self).__init__()

        def connection_made(self, transport):
            rospy.loginfo("Radar: Connected to dump1090")
            self.lb = LineBuffer()

        def data_received(self, data):
            #rospy.loginfo("Data received: %s" % data)
            self.lb.recv(data)

            lines = self.lb.readlines()

            try:
                for line in lines:
                    if len(line) == 0:
                        continue
                    self.thread.db.add(line)
            except:
                pass
                #traceback.print_exc()


        def connection_lost(self, exc):
            rospy.loginfo("Radar: Connection lost")
            asyncio.get_event_loop().stop()

    def __init__(self, db):
        self.db = db
        Thread.__init__(self)

    def stop(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.stopped = True
        self.join()

    def run(self):

        self.stopped = False

        host = "127.0.0.1"
        port = 30003
        conn_timeout = 5
        reconnect_delay = 5

        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        while not self.stopped:
            try:
                rospy.loginfo("Radar: Connecting to %s:%d ..." % (host , port))
                protocol = Radar1090.RadarProtocol(self)
                future = self.loop.create_connection(lambda : protocol, host, port)
                self.loop.run_until_complete(asyncio.wait_for(future, conn_timeout))
                #rospy.loginfo("Connected!")
                self.loop.run_forever()
            except:
                pass
                #traceback.print_exc()

            if not self.stopped: # it's just a disconnect!
                rospy.loginfo("Radar: Trying reconnect in %ss ..." % reconnect_delay)
                self.loop.run_until_complete(asyncio.sleep(reconnect_delay))




class TopicRadar(Topic):
    msg_class = String

    def cleanup(self):
        self.radar1090.stop()



    def __init__(self, topic, update_frequency):

        self.db = AircraftsDB()

        self.radar1090 = Radar1090(self.db)
        self.radar1090.start()

        super(TopicRadar, self).__init__(topic = topic, update_frequency = update_frequency)

    def build_message(self):

        self.db.check_timeout()

        aircrafts = []
        for key,aircraft in self.db.aircrafts.iteritems():
            if aircraft.looksgood():
                aircrafts.append({"desc" : "%s" % aircraft.id(), "hexident": aircraft.hexident, "callsign": aircraft.callsign, "lat": aircraft.latitude, "lon": aircraft.longitude, "alt": aircraft.altitude, "direction": aircraft.direction,  "heading": aircraft.heading, "last" : aircraft.last()})

#        aircrafts = [{"direction": "NW", "last": 4, "lon": "11.04535", "lat": "46.18588", "alt": "38025", "id": "501D21 (CTN52R)"}, {"direction": "SW", "last": 4, "lon": "10.79469", "lat": "46.28186", "alt": "35050", "id": "42428D (AFL2472)"}]

        return self.msg_class(data = json.dumps(aircrafts))

#####################################################################
# EOF
