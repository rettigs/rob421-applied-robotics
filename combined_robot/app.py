#!/usr/bin/env python

from __future__ import division

import math

import cv
import cv2
import numpy as np
import video

from common import draw_str, RectSelector
from mosse import MOSSE

# Constants
LAUNCH = 0
CARRIAGE = 1
SWAT = 2

class App:
    def __init__(self, maincap, bouncecap, robotq, appq, launchspeed):
        self.maincap = video.create_capture(maincap)
        self.bouncecap = video.create_capture(bouncecap)
        _, self.mainframe = self.maincap.read()
        cv2.namedWindow('mainframe')
        cv2.namedWindow('bounceframe')
        self.row = 0
        self.bounceshot = 0
        cv2.createTrackbar('row', 'mainframe', 0, 2, self.onrow)
        cv2.createTrackbar('speed', 'mainframe', 0, 512, self.onspeed)
        cv2.createTrackbar('bounceshot', 'mainframe', 0, 1, self.onbounceshot)
        cv2.imshow('mainframe', self.mainframe)
        self.rect_sel = RectSelector('mainframe', self.onrect)
        self.trackers = []
        self.robotq = robotq
        self.appq = appq
        self.launchspeed = launchspeed

        ret, self.prevbounce = self.bouncecap.read()
        self.prevbouncegray = cv2.cvtColor(self.prevbounce, cv2.COLOR_BGR2GRAY)

    def nothing(*arg):
        pass

    def draw_flow(img, flow, step=32):
        h, w = img.shape[:2]
        y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
        fx, fy = flow[y,x].T
        lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
        lines = np.int32(lines + 0.5)
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.polylines(vis, lines, 0, (0, 255, 0)) 
        for (x1, y1), (x2, y2) in lines:
            cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1) 
            mag = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            draw_str(vis, (x1, y1), "{}".format(int(mag)))
        return vis

    def onrow(self, row):
        '''When the row is changed, update the speed.'''
        self.row = row
        if self.bounceshot:
            if   row == 0: speed = 160
            elif row == 1: speed = 165
            elif row == 2: speed = 170
        else:
            if   row == 0: speed = 230
            elif row == 1: speed = 235
            elif row == 2: speed = 240
        cv2.setTrackbarPos('speed', 'frame', speed)

    def onspeed(self, speed):
        '''When the speed is changed, send it to the robot.'''
        self.robotq.put((0, speed))

    def onbounceshot(self, bounceshot):
        '''When we toggle bounce shots, update the speed.'''
        self.bounceshot = bounceshot
        self.onrow(self.row)

    def onrect(self, rect):
        frame_gray = cv2.cvtColor(self.mainframe, cv2.COLOR_BGR2GRAY)
        tracker = MOSSE(frame_gray, rect)
        self.trackers = [tracker]

    def drawcrosshairs(self, img, width, height, color=(0, 255, 255), thickness=1):
        p0 = int(width // 2), 0
        p1 = int(width // 2), int(height)
        cv2.line(img, p0, p1, color, thickness)
        p0 = int(width // 2) - int(width // 10), int(height // 2)
        p1 = int(width // 2) + int(width // 10), int(height // 2)
        cv2.line(img, p0, p1, color, thickness)

    def run(self):
        direction = 0
        while True:
            ret, self.mainframe = self.maincap.read()
            self.mainframe = cv2.flip(self.mainframe, -1)
            if not ret:
                break
            frame_gray = cv2.cvtColor(self.mainframe, cv2.COLOR_BGR2GRAY)
            for tracker in self.trackers:
                tracker.update(frame_gray)

            vis = self.mainframe.copy()
            width = self.maincap.get(cv.CV_CAP_PROP_FRAME_WIDTH)
            height = self.maincap.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
            if len(self.trackers) > 0:
                x, _ = self.trackers[0].draw_state(vis)
                x = int(x)

                # Make the robot move toward the object
                if x < width // 2:
                    if direction >= 0:
                        print "Going left"
                        self.robotq.put((1, 100000, 1))
                        direction = -1
                elif x > width // 2:
                    if direction <= 0:
                        print "Going right"
                        self.robotq.put((1, 100000, 0))
                        direction = 1
                else:
                    print "Cup targeting complete"
                    self.robotq.put((1, 0, 0))
                    direction = 0
            elif direction != 0:
                self.robotq.put((1, 0, 0))
                direction = 0

            self.drawcrosshairs(vis, width, height)
            self.rect_sel.draw(vis)

            draw_str(vis, (5, 15), "Launch speed: {}".format(self.launchspeed.value))

            ret, self.bounceframe = self.bouncecap.read()
            self.bounceframe = cv2.flip(self.bounceframe, -1)
            if not ret:
                break

            self.bouncegray = cv2.cvtColor(self.bounceframe, cv2.COLOR_BGR2GRAY)
            bounceflow = cv2.calcOpticalFlowFarneback(self.prevbouncegray, self.bouncegray, 0.5, 3, 15, 3, 5, 1.2, 0)
            self.prevbouncegray = self.bouncegray

            cv2.imshow('bounceframe', self.draw_flow(self.bouncegray, bounceflow))

            cv2.imshow('mainframe', vis)
            ch = cv2.waitKey(10)
            if ch == 27:
                break
            if ch == ord('d'):
                print "Manually going right"
                self.robotq.put((1, 50, 0))
            if ch == ord('a'):
                print "Manually going left"
                self.robotq.put((1, 50, 1))
            if ch == ord(' '):
                print "Shooting"
                self.robotq.put('shoot')
            if ch == ord('s'):
                print "Swatting"
                self.robotq.put('swat')
            if ch == ord('c'):
                self.trackers = []
        cv2.destroyAllWindows()
        self.robotq.put('exit')
