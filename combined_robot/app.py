#!/usr/bin/env python

from __future__ import division

import cv
import cv2
import video

from common import RectSelector
from mosse import MOSSE

# Constants
LAUNCH = 0
CARRIAGE = 1

class App:
    def __init__(self, video_src, robotq, appq):
        self.cap = video.create_capture(video_src)
        _, self.frame = self.cap.read()
        cv2.namedWindow('frame')
        cv2.createTrackbar('row', 'frame', 0, 2, self.onrow)
        cv2.createTrackbar('speed', 'frame', 1000, 10000, self.onspeed)
        cv2.imshow('frame', self.frame)
        self.rect_sel = RectSelector('frame', self.onrect)
        self.trackers = []
        self.robotq = robotq
        self.appq = appq

    def nothing(*arg):
        pass

    def onrow(self, row):
        '''When the row is changed, update the speed.'''
        if   row == 0: speed = 1000
        elif row == 1: speed = 1005
        elif row == 2: speed = 1010
        cv2.setTrackbarPos('speed', 'frame', speed)

    def onspeed(self, speed):
        '''When the speed is changed, send it to the robot.'''
        self.robotq.put((0, speed))

    def onrect(self, rect):
        frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        tracker = MOSSE(frame_gray, rect)
        self.trackers.append(tracker)

    def drawcrosshairs(self, img, width, height, color=(0, 255, 255), thickness=1):
        p0 = int(width // 2), 0
        p1 = int(width // 2), int(height)
        cv2.line(img, p0, p1, color, thickness)
        p0 = 0, int(height // 2)
        p1 = int(width), int(height // 2)
        cv2.line(img, p0, p1, color, thickness)

    def run(self):
        direction = 0
        while True:
            ret, self.frame = self.cap.read()
            if not ret:
                break
            frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            for tracker in self.trackers:
                tracker.update(frame_gray)

            vis = self.frame.copy()
            width = self.cap.get(cv.CV_CAP_PROP_FRAME_WIDTH)
            height = self.cap.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
            if len(self.trackers) > 0:
                x, _ = self.trackers[0].draw_state(vis)
                x = int(x)

                # Make the robot move toward the object
                if x < width // 2:
                    if direction >= 0:
                        print "going right"
                        self.robotq.put((1, 100000, 0))
                        direction = -1
                elif x > width // 2:
                    if direction <= 0:
                        print "going left"
                        self.robotq.put((1, 100000, 1))
                        direction = 1
                else:
                    print "Cup targeting complete; shooting"
                    self.robotq.put((1, 0, 0))
                    direction = 0
                    self.robotq.put('shoot')
                    self.trackers = []
            elif direction != 0:
                self.robotq.put((1, 0, 0))
                direction = 0

            self.drawcrosshairs(vis, width, height)
            self.rect_sel.draw(vis)

            cv2.imshow('frame', vis)
            ch = cv2.waitKey(10)
            if ch == 27:
                break
            if ch == ord(' '):
                self.paused = not self.paused
            if ch == ord('c'):
                self.trackers = []
        cv2.destroyAllWindows()
        self.robotq.put('exit')
