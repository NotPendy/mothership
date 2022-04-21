'''
    Continuously reads in frames of video, commmands copter to change yaw rate based on ball 
    horizontal position.

    Author: Robby Rivenbark
'''
import time
from imutils.video import VideoStream
import xml.etree.ElementTree as ET
import imutils
import cv2
import numpy as np
from pathlib import Path


def get_hsv_thresholds(threshold_color) :
    threshold_list = []
    
    current_dir = Path(__file__).parent.resolve()
        
    tree = ET.parse(str(current_dir / 'test_thresholds.xml'))
    root = tree.getroot()

    color_node = root.find(threshold_color)

    for thresh in list(color_node) :
        hsv_thresh = []
        for hsv_val in list(thresh) :
            hsv_thresh.append(int(hsv_val.text))
        threshold_list.append(hsv_thresh)
    
    return threshold_list

thresholds = get_hsv_thresholds("dark_red")

#initialize video capture
cap = VideoStream(src=0).start()

time.sleep(2.0)

#start frame read loop
while True :
    #get frame
    frame = cap.read()

    frame = imutils.resize(frame, width=600)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    im_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    #mask image using each hsv filter
    masks = []
    i = 0
    for hsv in thresholds :
        i += 1
        hsv_min = (hsv[0], hsv[1], hsv[2])
        hsv_max = (hsv[3], hsv[4], hsv[5])
        m = cv2.inRange(im_hsv, hsv_min, hsv_max)
        masks.append(m)
        cv2.imshow("Individual Mask" + str(i), m)
        key = cv2.waitKey(1) & 0xFF
    
    # add masks 
    result = masks[0]
    for m in masks:
        result = cv2.add(result, m)

    # show results
    cv2.imshow('result', result)
    key = cv2.waitKey(1) & 0xFF


    # if frame is read correctly ret is True
    if frame is None:
        print("Can't receive frame (stream end?). Exiting ...")
        break


#release video capture
cap.stop()

