#!/usr/bin/env python

import getopt
from multiprocessing import Process, Queue
import sys

from app import App
from robot import Robot

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

    robot = Robot(serialDevice, robotq, appq)
    robotProcess = Process(target=robot.main)
    robotProcess.start()

    #opts, args = getopt.getopt(sys.argv[1:], '', ['pause'])
    #opts = dict(opts)

    #App(videoDevice, paused = '--pause' in opts).run()
    App(captureDevice, robotq, appq).run()
