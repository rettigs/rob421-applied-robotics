#!/usr/bin/env python

import random as rand
import numpy as np
import cv2
import cv2.cv as cv
from video import create_capture
from common import clock, draw_str

help_message = '''
USAGE: cupdetect.py [<video_source>]
'''

if __name__ == '__main__':
    import sys, getopt
    print help_message

    args, video_src = getopt.getopt(sys.argv[1:], '', [])
    try: video_src = video_src[0]
    except: video_src = 0
    args = dict(args)

    cam = cv2.VideoCapture()
    cam.open("http://localhost:8000/stream.avi")

    while True:
        ret, img = cam.read()
        print ret
        if ret:
            cv2.imshow('raw', img)
        else:
            cam.open("http://localhost:8000/stream.avi")

        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows()
