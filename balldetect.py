#!/usr/bin/env python

from __future__ import division
import math

import numpy as np
import cv2
import cv2.cv as cv
from video import create_capture
from common import clock, draw_str

help_message = '''
USAGE: cupdetect.py [<video_source>]
'''

def isEllipse(contour):
    '''
    Detects if the given polygon is an ellipse.
    Returns the ellipse form of the polygon if it's an ellipse, None otherwise.
    '''
    try: rect = cv2.fitEllipse(contour) # Rotated rectangle representing the ellipse it tries to fit
    except: return None
    (x, y), (w, h), angle = rect # x offset, y offset, width, height, angle
    w, h = h, w # Switch them since our ellipses are usually rotated 90 degrees

    TEST_POINTS = 16
    E = 0.1 # Radius percentage increase/decrease for test points
    MIN_SUCC_RATE = 0.9
    successes = 0

    for a in np.arange(0, 2*math.pi, 2*math.pi/TEST_POINTS):
        ipoint = (x+0.5*w*math.cos(a)*(1-E), y+0.5*h*math.sin(a)*(1-E))
        opoint = (x+0.5*w*math.cos(a)*(1+E), y+0.5*h*math.sin(a)*(1+E))
        #test = cv2.pointPolygonTest(contour, ipoint, False)
        if cv2.pointPolygonTest(contour, ipoint, False) > 0: # The inside point is inside
            successes += 1
        if cv2.pointPolygonTest(contour, opoint, False) < 0: # The outside point is outside
            successes += 1

    succ_rate = successes / (TEST_POINTS * 2)
    if succ_rate >= MIN_SUCC_RATE:
        return rect
    else:
        return None

def nothing(*arg):
    pass

if __name__ == '__main__':
    import sys, getopt
    print help_message

    args, video_src = getopt.getopt(sys.argv[1:], '', [])
    try: video_src = video_src[0]
    except: video_src = 0
    args = dict(args)

    cam = create_capture(video_src)
    #stream = cv2.VideoWriter("/home/chekkaa/git/rob421-applied-robotics/stream.avi", cv.CV_FOURCC(*'MJPG'), 60.0, (640, 480), True)

    shapes = [] # List of tracked shapes

    cv2.namedWindow('grayscale')
    cv2.namedWindow('binary')
    #cv2.namedWindow('contours')
    cv2.namedWindow('raw')
    cv2.createTrackbar('satthresh', 'raw', 200, 255, nothing)
    #cv2.createTrackbar('circlelowthresh', 'raw', 50, 1000, nothing)
    #cv2.createTrackbar('circlehighthresh', 'raw', 100, 2000, nothing)
    cv2.createTrackbar('minarea', 'raw', 100, 10000, nothing)
    cv2.createTrackbar('maxarea', 'raw', 800, 10000, nothing)
    cv2.createTrackbar('valthresh', 'raw', 200, 255, nothing)
    #frame_counter = 1

    while True:
        ret, img = cam.read()
        #frame_counter += 1
        #If the last frame is reached, reset the capture and the frame_counter
        #if frame_counter == cam.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT):
        #    frame_counter = 0 #Or whatever as long as it is the same as next line
        #    cam.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)

        t = clock() # Start timing how long it took to process this frame

        img = cv2.flip(img, -1)
        cv2.circle(img, (100, 100), 20, (250, 250, 250), -1, 8, 0);
        
        # Make all colorful (high saturation) pixels black to help the white ping pong ball stand out
        hsvimg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV);
        hsvimg = cv2.GaussianBlur(hsvimg, (5, 5), 2, 2);
        h, s, v = cv2.split(hsvimg)
        cv2.imshow('h', h)
        cv2.imshow('s', s)
        cv2.imshow('v', v)
        satthresh = cv2.getTrackbarPos('satthresh', 'raw')

        #satlowerbound = cv.Scalar(0, 0, 0)
        #satupperbound = cv.Scalar(255, satthresh, 255)
        #cv2.inRange(hsvimg, satlowerbound, satupperbound, hsvimg)

        cv2.inRange(s, satthresh, 255, s)

        #ret, v = cv2.threshold(s, satthresh, 255, cv2.THRESH_BINARY_INV)
        hsvimg = cv2.merge((h, s, v))
        cfimg = cv2.cvtColor(hsvimg, cv2.COLOR_HSV2BGR); # Color-filtered image
        cv2.imshow('cfimg', cfimg)

        grayimg = cv2.cvtColor(cfimg, cv2.COLOR_BGR2GRAY)
        #grayimg = cv2.equalizeHist(grayimg)
        grayimg = cv2.GaussianBlur(grayimg, (5, 5), 2, 2);
        valthresh = cv2.getTrackbarPos('valthresh', 'raw')
        ret, binimg = cv2.threshold(grayimg, valthresh, 255, cv2.THRESH_BINARY)
        visbinimg = binimg.copy()
        contours, hierarchy = cv2.findContours(binimg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cup_contours = []
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            minarea = cv2.getTrackbarPos('minarea', 'raw')
            maxarea = cv2.getTrackbarPos('maxarea', 'raw')
            if contour_area > minarea and contour_area < maxarea:
                ellipse = isEllipse(contour)
                if ellipse:
                    cup_contours.append(contour)
  
        color = (255, 0, 0) # Color is BGR, not RBG!
        cv2.drawContours(img, cup_contours, -1, color, thickness=-1)

        #circlelowthresh = cv2.getTrackbarPos('circlelowthresh', 'raw')
        #circlehighthresh = cv2.getTrackbarPos('circlehighthresh', 'raw')
        #circles = cv2.HoughCircles(grayimg, cv.CV_HOUGH_GRADIENT, 1, len(grayimg)/8, circlehighthresh, circlelowthresh)

        #if circles is not None:
        #    for circle in circles:
        #        print "circle"
        #        #circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
        #        cv2.circle(img, (circle[0][0], circle[0][1]), circle[0][2], color, 3, 8, 0);

        dt = clock() - t # Stop timing how long it took to process this frame

        # Show the images
        draw_str(grayimg, (20, 20), 'time: %.1f ms' % (dt*1000))
        draw_str(img, (20, 20), 'time: %.1f ms' % (dt*1000))
        cv2.imshow('grayscale', grayimg)
        cv2.imshow('binary', binimg)
        cv2.imshow('raw', img)

        #stream.write(img)

        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows()

class Circle(object):
    '''
    Represents a circle in (x, y, r) format.
    Stores a number of "previous" shapes in an attempt to track the shape.
    '''
    
    def __init__(self):
        self.x = 320
        self.y = 240
        self.r = 10

    def add_frame(self, circle):
        alpha = 0.8
        e = 0.5
        if  abs(self.x - circle[0]) / self.x < e \
        and abs(self.y - circle[1]) / self.y < e \
        and abs(self.r - circle[2]) / self.r < e:
            self.x = (circle[0] * alpha) + (self.x * (1 - alpha))
            self.y = (circle[1] * alpha) + (self.y * (1 - alpha))
            self.r = (circle[2] * alpha) + (self.r * (1 - alpha))
        else:
            print "Circle too different"
