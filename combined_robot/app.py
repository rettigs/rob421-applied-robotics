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
        self.row = 0
        self.bounceshot = 0
        cv2.createTrackbar('row', 'frame', 0, 2, self.onrow)
        cv2.createTrackbar('speed', 'frame', 3920, 10000, self.onspeed)
        cv2.createTrackbar('bounceshot', 'frame', 0, 1, self.onbounceshot)
        cv2.imshow('frame', self.frame)
        self.rect_sel = RectSelector('frame', self.onrect)
        self.trackers = []
        self.robotq = robotq
        self.appq = appq

    def nothing(*arg):
        pass

    def onrow(self, row):
        '''When the row is changed, update the speed.'''
        self.row = row
        if self.bounceshot:
            if   row == 0: speed = 1920
            elif row == 1: speed = 1930
            elif row == 2: speed = 1940
        else:
            if   row == 0: speed = 3920
            elif row == 1: speed = 3930
            elif row == 2: speed = 3940
        cv2.setTrackbarPos('speed', 'frame', speed)

    def onspeed(self, speed):
        '''When the speed is changed, send it to the robot.'''
        self.robotq.put((0, speed))

    def onbounceshot(self, bounceshot):
        '''When we toggle bounce shots, update the speed.'''
        self.bounceshot = bounceshot
        self.onrow(self.row)

    def onrect(self, rect):
        frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        tracker = MOSSE(frame_gray, rect)
        self.trackers = [tracker]

    def drawcrosshairs(self, img, width, height, color=(0, 255, 255), thickness=1):
        p0 = int(width // 2), int(height // 2) - int(height // 10)
        p1 = int(width // 2), int(height // 2) + int(height // 10)
        cv2.line(img, p0, p1, color, thickness)
        p0 = int(width// 2) - int(width // 10), int(height // 2)
        p1 = int(width// 2) + int(width // 10), int(height // 2)
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

            cv2.imshow('frame', vis)
            ch = cv2.waitKey(10)
            if ch == 27:
                break
            if ch == ord('d'):
                print "Manually going right"
                self.robotq.put((1, 10, 1))
            if ch == ord('a'):
                print "Manually going left"
                self.robotq.put((1, 10, 0))
            if ch == ord(' '):
                print "Shooting"
                self.robotq.put('shoot')
            if ch == ord('c'):
                self.trackers = []
        cv2.destroyAllWindows()
        self.robotq.put('exit')
