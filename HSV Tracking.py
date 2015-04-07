import cv2
import cv2.cv as cv
import numpy as np
import time

numFrames = 0
erosionKernel = np.ones((3,3),np.uint8)
dilationKernel = np.ones((5,5),np.uint8)

cap = cv2.VideoCapture(0)
#cap.set(3,320)
#cap.set(4,240)

cv2.namedWindow('params', 0)
H_max = 255
H_min = 0
S_max = 255
S_min = 0
V_max = 255
V_min = 0
def set_scale_H_max(val):
    global H_max
    H_max = val
cv2.createTrackbar('H_Max', 'params', H_max, 255, set_scale_H_max)
def set_scale_H_min(val):
    global H_min
    H_min = val
cv2.createTrackbar('H_Min', 'params', H_min, 255, set_scale_H_min)

def set_scale_S_max(val):
    global S_max
    S_max = val
cv2.createTrackbar('S_Max', 'params', S_max, 255, set_scale_S_max)
def set_scale_S_min(val):
    global S_min
    S_min = val
cv2.createTrackbar('S_Min', 'params', S_min, 255, set_scale_S_min)

def set_scale_V_max(val):
    global V_max
    V_max = val
cv2.createTrackbar('V_Max', 'params', V_max, 255, set_scale_V_max)
def set_scale_V_min(val):
    global V_min
    V_min = val
cv2.createTrackbar('V_Min', 'params', V_min, 255, set_scale_V_min)

timeStart = time.time()
while(1):

    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of white color in HSV
    # change it according to your need !
    lower = np.array([H_min,S_min,V_min], dtype=np.uint8)
    upper = np.array([H_max,S_max,V_max], dtype=np.uint8)

    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(hsv, lower, upper)

    mask = cv2.erode(mask, erosionKernel,iterations = 1)
    mask = cv2.dilate(mask, dilationKernel, iterations = 2)
    mask = cv2.erode(mask,erosionKernel, iterations = 1)


    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    print contours
    for cnt in contours :
        if (cv2.contourArea(cnt) > 50):
            cv2.drawContours(frame, [cnt],-1, (0,0,255), thickness = 2)

#    circle = cv2.HoughCircles(mask, cv.CV_HOUGH_GRADIENT, 1, 5)
#    print circle

    cv2.imshow('frame' ,frame)
#    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    numFrames += 1
    if cv2.waitKey(2) == 27:
        timeEnd = time.time()
        break

print "Frames per second:", (numFrames/(timeEnd - timeStart))
print "H_Max: " + str(H_max)
print "H_Min: " + str(H_min)
print "S_Max: " + str(S_max)
print "S_Min: " + str(S_min)
print "V_Max: " + str(V_max)
print "V_Min: " + str(V_min)
cap.release()
cv2.destroyAllWindows()
