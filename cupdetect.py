#!/usr/bin/env python

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

    cam = create_capture(video_src, fallback='synth:bg=../cpp/lena.jpg:noise=0.05')

    while True:
        ret, img = cam.read()

        t = clock() # Start timing how long it took to process this frame

        grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grayimg = cv2.equalizeHist(grayimg)
        ret, binimg = cv2.threshold(grayimg, 100, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(binimg, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        cup_contours = []
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area > 100:
                ellipse = cv2.fitEllipse(contour)
                ellipse_area = ellipse[1][0] * ellipse[1][1] # Area of rotated rect representing the ellipse
                if abs(ellipse_area * 1.27 - contour_area) < 1000:
                    cup_contours.append(contour)

        color = (255, 0, 0) # Color is BGR, not RBG!
        cv2.drawContours(img, cup_contours, -1, color, thickness=-1)

        dt = clock() - t # Stop timing how long it took to process this frame

        # Show the images
        draw_str(img, (20, 20), 'time: %.1f ms' % (dt*1000))
        #draw_str(grayimg, (20, 20), 'time: %.1f ms' % (dt*1000))
        draw_str(binimg, (20, 20), 'time: %.1f ms' % (dt*1000))
        cv2.imshow('raw', img)
        #cv2.imshow('grayscale', grayimg)
        cv2.imshow('binary', binimg)

        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows()
