#!/usr/bin/env python

import cv2
import numpy as np
import video

from common import draw_str, RectSelector
from mosse import MOSSE

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

    def run(self):
        while True:
            ret, self.frame = self.cap.read()
            if not ret:
                break
            frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            for tracker in self.trackers:
                tracker.update(frame_gray)

            vis = self.frame.copy()
            for tracker in self.trackers:
                tracker.draw_state(vis)
            #if len(self.trackers) > 0:
            #    cv2.imshow('tracker state', self.trackers[-1].state_vis)
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
