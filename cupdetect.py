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

def isEllipse(contour):
    rect = cv2.fitEllipse(contour) # Rotated rectangle representing the ellipse it tries to fit
    (x, y), (w, h), angle = rect # x offset, y offset, width, height, angle
    w, h = h, w # Switch them since our ellipses are usually rotated 90 degrees

    # Draw TEST_INSIDE points inside the ellipse and TEST_OUTSIDE points outside and see if they're in the hull
    TEST_INSIDE = 10
    TEST_OUTSIDE = 10

    # Equation of ellipse: (x/a)^2 + (y/b)^2 = 1

    e = 0.1
    
    tests = []
    tests.append(cv2.pointPolygonTest(contour, (x, y), False))

    tests.append(cv2.pointPolygonTest(contour, (x+w*(0.5-e), y), False))
    tests.append(cv2.pointPolygonTest(contour, (x-w*(0.5-e), y), False))
    tests.append(cv2.pointPolygonTest(contour, (x, y+h*(0.5-e)), False))
    tests.append(cv2.pointPolygonTest(contour, (x, y-h*(0.5-e)), False))

    tests.append(not cv2.pointPolygonTest(contour, (x+w*(0.5-e), y+h*(0.5-e)), False))
    tests.append(not cv2.pointPolygonTest(contour, (x+w*(0.5-e), y-h*(0.5-e)), False))
    tests.append(not cv2.pointPolygonTest(contour, (x-w*(0.5-e), y+h*(0.5-e)), False))
    tests.append(not cv2.pointPolygonTest(contour, (x-w*(0.5-e), y-h*(0.5-e)), False))

    for test in tests:
        if test == -1.0:
            return False

    return True

if __name__ == '__main__':
    import sys, getopt
    print help_message

    args, video_src = getopt.getopt(sys.argv[1:], '', [])
    try: video_src = video_src[0]
    except: video_src = 0
    args = dict(args)

    cam = create_capture(video_src)

    while True:
        ret, img = cam.read()

        t = clock() # Start timing how long it took to process this frame

        grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayimg = cv2.equalizeHist(grayimg)
        ret, binimg = cv2.threshold(grayimg, 200, 255, cv2.THRESH_BINARY)
        #binimg = cv2.adaptiveThreshold(grayimg, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 13, 5)
        #binimg = cv2.Canny(grayimg, 250, 255)
        # do erosion and dilation?
        visbinimg = binimg.copy()
        contours, hierarchy = cv2.findContours(binimg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cup_contours = []
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area > 100 and contour_area < 800 and isEllipse(contour):
                cup_contours.append(contour)

        color = (255, 0, 0) # Color is BGR, not RBG!
        cv2.drawContours(img, cup_contours, -1, color, thickness=-1)

        dt = clock() - t # Stop timing how long it took to process this frame

        # Show the images
        draw_str(grayimg, (20, 20), 'time: %.1f ms' % (dt*1000))
        draw_str(visbinimg, (20, 20), 'time: %.1f ms' % (dt*1000))
        draw_str(binimg, (20, 20), 'time: %.1f ms' % (dt*1000))
        draw_str(img, (20, 20), 'time: %.1f ms' % (dt*1000))
        cv2.imshow('grayscale', grayimg)
        cv2.imshow('binary', visbinimg)
        cv2.imshow('contours', binimg)
        cv2.imshow('raw', img)

        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows()
