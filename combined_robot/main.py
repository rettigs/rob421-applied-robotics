#!/usr/bin/env python

import getopt
from multiprocessing import Process, Queue, Value
import sys

from app import App
from robot import Robot

def usage():
    print 'Usage: {0} [-s serialdevice] [-c capturedevice] [-h] [-v]... [-d]...'.format(sys.argv[0])
    print '\t-h\tview this help'
    print '\t-s\tspecify a serial device to use, defaults to "/dev/ttyACM0"'
    print '\t-c\tspecify a video capture device to use, defaults to "/dev/video0"'
    print '\t-v\tenable more verbose messages; use -vv for more even more messages'
    print '\t-d\tenable debug messages; use -dd for more even more messages'
    sys.exit(2)

if __name__ == '__main__':

    # Defaults
    serialDevice = '/dev/ttyACM0'
    captureDevice = '0'

    # Parse arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:], "s:c:dvh")
    except getopt.GetoptError as err:
        # print help information and exit:
        print str(err) # will print something like "option -a not recognized"
        sys.exit(2)
    for o, a in opts:
        if o == "-d":
            global debug
            debug += 1
        elif o == "-v":
            global verbose
            verbose += 1
        elif o == "-s":
            serialDevice = a
        elif o == "-c":
            captureDevice = a
        else:
            usage()
    
    robotq, appq = Queue(), Queue()
    launchspeed = Value('i', 0)

    robot = Robot(serialDevice, robotq, appq, launchspeed)
    robotProcess = Process(target=robot.main)
    robotProcess.start()

    #opts, args = getopt.getopt(sys.argv[1:], '', ['pause'])
    #opts = dict(opts)

    #App(videoDevice, paused = '--pause' in opts).run()
    App(captureDevice, robotq, appq, launchspeed).run()
