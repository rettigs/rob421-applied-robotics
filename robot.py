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

speeds = {}

port = serial.Serial("/dev/ttyACM0", baudrate=9600, timeout=3.0)

def readPackets():
    while port.inWaiting() > 0:
        packet = port.read(2)
        byte1, byte2 = map(int, packet)
        deviceid = byte1 << (8 - OFFSET_TYPE) >> (8 - OFFSET_TYPE + OFFSET_ID) << OFFSET_ID
        magnitude = byte2
        print "device {} updated with magnitude {}".format(deviceid, magnitude)

def getSpeed(deviceid):
    return speeds[deviceid]

def setSpeed(deviceid, speed=0, direction=0):
    byte1 = (PACKET_DEVICE << OFFSET_TYPE) | (deviceid << OFFSET_ID) | (direction << OFFSET_DIRECTION)
    byte2 = (speed << OFFSET_MAGNITUDE)
    packet = bytes([byte1, byte2])
    print packet
    print [format(int(p), '#010b') for p in packet[1:-1].split(', ')]
    port.write(bytes([byte1, byte2]))

while True:
    readPackets()
    print getSpeed(1)
    setSpeed(1, 255, 0)
    time.sleep(1)

    readPackets()
    print getSpeed(1)
    setSpeed(1, 0, 0)
    time.sleep(1)
