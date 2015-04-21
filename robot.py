#!/usr/bin/env python

import serial
import struct 
import time

# Constants
OFFSET_TYPE = 6
OFFSET_ID = 1
OFFSET_DIRECTION = 0
OFFSET_MAGNITUDE = 0
PACKET_DEVICE = 0b00
PACKET_RELOAD = 0b01
PACKET_SWAT = 0b10

class SerialManager(object):

    def __init__(self):
        self.speeds = {}
        self.port = serial.Serial("/dev/ttyACM0", baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=3.0)

    def bytes_to_strings(self, packet):
        return [format(int(p), '#010b') for p in packet[1:-1].split(', ')]

    def readPackets(self):
        while self.port.inWaiting() > 0:
            packet = self.port.read(2)
            print "received: '{}'".format([ord(p) for p in packet])
            byte1, byte2 = map(ord, packet)
            deviceid = byte1 << (8 - OFFSET_TYPE) >> (8 - OFFSET_TYPE + OFFSET_ID) << OFFSET_ID
            magnitude = byte2
            dbyte, mbyte = 1, 2#self.bytes_to_strings(packet)
            #print "device {} ({}) updated with magnitude {} ({})".format(deviceid, dbyte, magnitude, mbyte)

    def getSpeed(self, deviceid):
        return self.speeds[deviceid]

    def setSpeed(self, deviceid, speed=0, direction=0):
        byte1 = (PACKET_DEVICE << OFFSET_TYPE) | (deviceid << OFFSET_ID) | (direction << OFFSET_DIRECTION)
        byte2 = (speed << OFFSET_MAGNITUDE)
        packet = bytes([byte1, byte2])
        #print packet
        #print self.bytes_to_strings(packet)

        packet = '\x00\x80'
        print "sending: '{}'".format([ord(p) for p in packet])
        self.port.write(packet)

    def main(self):
        while True:
            '''
            self.readPackets()
            #print getSpeed(0)
            self.setSpeed(0, 100, 0)
            time.sleep(1)
            '''
            #print getSpeed(0)
            self.setSpeed(0, 0, 0)
            self.readPackets()
            time.sleep(1)

if __name__ == '__main__':
    serialManager = SerialManager()
    serialManager.main()
