#!/usr/bin/env python

import serial
import struct 
import time

# Constants
OFFSET_TYPE = 6
OFFSET_ID = 1
OFFSET_DIRECTION = 0
OFFSET_MAGNITUDE1 = 8
OFFSET_MAGNITUDE2 = 0
PACKET_DEVICE = 0b00
PACKET_RELOAD = 0b01
PACKET_SWAT = 0b10

class SerialManager(object):

    def __init__(self):
        self.speeds = {}
        self.port = serial.Serial("/dev/ttyACM0", baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=3.0)

    def main(self):
        while True:
            self.readPackets()
            print self.getSpeed(0)
            self.setSpeed(0, 127)
            time.sleep(1)
            self.readPackets()
            print self.getSpeed(0)
            self.setSpeed(0, 239)
            time.sleep(1)

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

        print "sending: '{}'".format([ord(p) for p in packet])
        self.port.write(packet)

if __name__ == '__main__':
    serialManager = SerialManager()
    serialManager.main()
