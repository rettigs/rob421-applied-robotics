#!/usr/bin/env python

import serial

# Constants
OFFSET_TYPE = 6
OFFSET_ID = 1
OFFSET_DIRECTION = 0
OFFSET_MAGNITUDE1 = 8
OFFSET_MAGNITUDE2 = 0
PACKET_DEVICE = 0b00
PACKET_SHOOT = 0b01
LAUNCH = 0
CARRIAGE = 1
SWAT = 2

class Robot(object):

    def __init__(self, serialDevice, robotq, appq, launchspeed):
        self.speeds = {}
        self.port = serial.Serial(serialDevice, baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=3.0)
        self.robotq = robotq
        self.appq = appq
        self.launchspeed = launchspeed
        self.swatted = False

    def main(self):
        self.speeds[1] = 0
        self.unswat()
        while True:

            # Check for work from the GUI
            try: work = self.robotq.get(False)
            except: pass
            else:
                if work == 'exit': exit()
                elif work == 'shoot': self.shoot()
                elif work == 'swat': self.swat()
                else:
                    self.setSpeed(*work)

            # Check for packets from the microcontroller
            self.readPackets()

            # Send the launch motor's speed to the app for display
            self.launchspeed.value = self.speeds.get(1)

    def readPackets(self):
        while self.port.inWaiting() >= 3:
            packet = self.port.read(3)
            #print "received: '{}'".format([ord(p) for p in packet])
            byte1, byte2, byte3 = map(ord, packet)
            deviceid = byte1 << (8 - OFFSET_TYPE) >> (8 - OFFSET_TYPE + OFFSET_ID) << OFFSET_ID
            magnitude = (byte2 << OFFSET_MAGNITUDE1) | (byte3 << OFFSET_MAGNITUDE2)
            self.speeds[deviceid] = magnitude
            #print "device {} ({}) updated with magnitude {} ({})".format(deviceid, dbyte, magnitude, mbyte)

    def getSpeed(self, deviceid):
        return self.speeds.get(deviceid, 0)

    def setSpeed(self, deviceid, speed=0, direction=0):
        byte1 = (PACKET_DEVICE << OFFSET_TYPE) | (deviceid << OFFSET_ID) | (direction << OFFSET_DIRECTION)
        byte2 = (speed >> OFFSET_MAGNITUDE1) & 0b11111111
        byte3 = (speed >> OFFSET_MAGNITUDE2) & 0b11111111
        packet = ''.join(chr(b) for b in [byte1, byte2, byte3])

        #print "sending: '{}'".format([ord(p) for p in packet])
        self.port.write(packet)

    def shoot(self):
        byte1 = (PACKET_SHOOT << OFFSET_TYPE)
        byte2 = 0
        byte3 = 0
        packet = ''.join(chr(b) for b in [byte1, byte2, byte3])

        self.port.write(packet)

    def swat(self):
        byte1 = (PACKET_DEVICE << OFFSET_TYPE) | (SWAT << OFFSET_ID)
        if not self.swatted:
            pos = 1220
            self.swatted = True
        else:
            pos = 1152
            self.swatted = False
        byte2 = (pos >> OFFSET_MAGNITUDE1) & 0b11111111
        byte3 = (pos >> OFFSET_MAGNITUDE2) & 0b11111111
        packet = ''.join(chr(b) for b in [byte1, byte2, byte3])

        self.port.write(packet)

    def unswat(self):
        byte1 = (PACKET_DEVICE << OFFSET_TYPE) | (SWAT << OFFSET_ID)
        pos = 1152
        byte2 = (pos >> OFFSET_MAGNITUDE1) & 0b11111111
        byte3 = (pos >> OFFSET_MAGNITUDE2) & 0b11111111
        packet = ''.join(chr(b) for b in [byte1, byte2, byte3])

        self.port.write(packet)
