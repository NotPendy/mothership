'''
    Author: Robby Rivenbark
    Email: rsrivenb@ncsu.edu, robbyisgreat@gmail.com

    Purpose: Maintains data and functionality for reading in a frame of video from the mothership, deciding what movements
    should be made, and outputting a commanded velocity.
    
    Does not directly interface with hardware, does not need to know where images are coming from or how the velocity
    information gets to the drone, it just uses these as input and output.
'''

from pickle import FALSE
import cv2
import math
import time
import numpy as np
import imutils
from collections import deque
import xml.etree.ElementTree as ET
from pathlib import Path

class Frame_Processor:

    # region [init]

    MAX_YAW_RATE = np.pi / 4
    MAX_VERTICAL_SPEED = .1
    FORWARD_VELOCITY = .5

    CENTERED_TOLERANCE_HORIZONTAL = .05
    CENTERED_TOLERANCE_VERTICAL = .02
    VERTICAL_OFFSET = .1

    '''
        Constructor sets up necessary blob detection parameters

        threshold_color specifies which threshold colors to use from "thresholds.xml"

        rotating seek specifies whether values to rotate will be given if sight of the target is lost while attempting to center.
    '''
    def __init__(self, threshold_color="default", rotating_seek = False):

        self.rotating_seek = rotating_seek

        self.centered_horizontally = False
        self.centered_vertically = False
        self.in_frame = False

        self.pts = deque(maxlen=10)

        #hsv thresholds
        self.hsv_thresholds = []

        for hsv_threshold in self.__read_hsv_thresholds__(threshold_color) :
            self.hsv_thresholds.append(hsv_threshold)
        

    # endregion

    # region [High-Level Algorithms]

    '''
        This function checks whether a red blob is in the frame
    '''
    def process_frame_check_blob(self, frame, show = False) :
        self.__find_blobs__(frame, show=show)

        return self.in_frame


    '''
        This function processes one frame of video and returns the vertical velocity that will bring the drone
        closer to being centered.
    '''
    def center_vertically(self, frame, show = False, stop_when_centered = True) :

        #Run blob detection on frame
        self.__find_blobs__(frame, show=show)

        if not stop_when_centered : self.centered_vertically = False

        if not self.centered_vertically :
            return self.__center_blob__(horizontal=False)

        else :
            return self.__stop__()

    '''
        This function is called to process one frame of video once the drone has already been centered vertically.
        
        It returns the velocity that should be commanded to the vehicle.
    '''
    def center_horizontally_and_advance(self, frame, show = False, advance = True):

        #Run blob detection on frame
        self.__find_blobs__(frame, show=show)

        if not advance : self.centered_horizontally = False

        #If the camera is not centered on red, center on red
        if not self.centered_horizontally :
            return self.__center_blob__(horizontal=True)
    
        print("advancing")
        return self.__advance__(frame)

    '''
        resets all flags that may have been modified by frame processing
    '''
    def reset_flags(self) :
        self.centered_horizontally = False


    # endregion

    # region [Primary Helpers]

    '''
        Commands drone to make necessary adjustments to center camera on red ball
    '''
    def __center_blob__(self, horizontal, modify_conditions = True, record = True) :

        #if we don't detect a blob, seek
        if not self.in_frame:
            print("seeking")
            return self.__seek__()

        #get normalized x position of blob in frame
        horizontal_position_blob, vertical_position_blob = self.__get_blob_relative_position__()
        vertical_position_blob = vertical_position_blob - self.VERTICAL_OFFSET

        print(horizontal_position_blob) if horizontal else print(vertical_position_blob)

        #calculate commanded velocity based off of blob location in frame
        forward_rate = 0.0
        #TODO: are signs right on these?
        vertical_rate = 0.0 if horizontal else self.MAX_VERTICAL_SPEED * (vertical_position_blob)
        yaw_rate = self.MAX_YAW_RATE * horizontal_position_blob if horizontal else 0.0
        cmd_vel = Commanded_Velocity(forward_rate, vertical_rate, yaw_rate)

        if modify_conditions and horizontal :
            #if the red ball's position within frame is within tolerance, set centered_red flag to true
            if math.fabs(horizontal_position_blob) < self.CENTERED_TOLERANCE_HORIZONTAL :
                self.centered_horizontally = True
            else :
                self.centered_horizontally = False
        
        elif modify_conditions and not horizontal :
            #if the red ball's position within frame is within tolerance, set centered_red flag to true
            if math.fabs(vertical_position_blob) < self.CENTERED_TOLERANCE_VERTICAL:
                self.centered_vertically = True
            cmd_vel, tmp = self.__stop__() 
        else :
                self.centered_vertically = False

        if record :
            #record value and time of last cmd_vel published
            self.last_cmd_vel = cmd_vel
            self.last_cmd_vel_time = time.time()

        return cmd_vel, False
    

    '''
        Constructs and returns command for drone to move at the target blob, keeping the target blob
        in its center of vision horizontally.
    '''
    def __advance__(self, frame, clockwise = 1.0) :

        #if we don't detect a blob decide between following through and stopping
        if not self.in_frame:
            current_time = time.time()

            #If the time between now and the last commanded velocity is less than 1 second, follow through
            if None is self.last_cmd_vel_time or self.__time_difference_seconds__( self.last_cmd_vel_time, current_time ) <= 1.0 :
                return self.__follow_through__()
            #otherwise, stop
            else :
                self.centered_horizontally = False
                return self.__stop__()
        
        #call center red to make any angular adjustments
        cmd_vel, tmp = self.__center_blob__(self, modify_conditions=False, record=False)
        cmd_vel.forward = self.FORWARD_VELOCITY

        #record value and time of last cmd_vel published
        self.last_cmd_vel = cmd_vel
        self.last_cmd_vel_time = time.time()

        return cmd_vel, False


    # endregion

    # region [Secondary Helpers]

    '''
        Turn the drone to help it find the ball
        sends out indication that drone has not stopped
    '''
    def __seek__(self, clockwise = 1.0) :
        if self.rotating_seek : return Commanded_Velocity(0,0,-1.0*clockwise), False

        return Commanded_Velocity(0,0,0), False

    '''
        Tells the drone to follow through and repeat the last commanded velocity.
        sends out indication that drone has not stopped
    '''
    def __follow_through__(self) :
        return self.last_cmd_vel, False

    '''
        commands the drone to stop moving.
        sends out indication that the drone has stopped
    '''
    def __stop__(self) :
        return Commanded_Velocity(0,0,0), True

    '''
        returns time difference in seconds
    '''
    def __time_difference_seconds__(self, start_time, end_time) :
        nanosec_in_sec = 10 ** 9
        time_diff_nanosec = 1.0 * (end_time -  start_time)
        time_diff_sec = time_diff_nanosec / nanosec_in_sec
        return time_diff_sec

    # endregion

    # region [Vision Helpers]

    '''
        Takes in image and finds keypoints. 

    '''
    def __find_blobs__(self, frame, show = False) :

        # set red_in_frame to false, only set to true if we find ball in frame
        self.in_frame = False
        
        frame = imutils.resize(frame, width=600)

        self.im = frame

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        im_hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        #get mask by using thresholds
        mask = self.__get_mask__(im_hsv)

        #dilating increases size of white areas, eroding trims.
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.erode(mask, None, iterations=1)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] != 0.0 :
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 5:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(frame, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

                    #red is in frame
                    self.in_frame = True

                    #record horizontal, vert position of blob
                    self.horizontal_pos = x
                    self.vertical_pos = y

        if show :
            # update the points queue
            self.pts.appendleft(center)

            # loop over the set of tracked points
            for i in range(1, len(self.pts)):
                # if either of the tracked points are None, ignore
                # them
                if self.pts[i - 1] is None or self.pts[i] is None:
                    continue

                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                thickness = int(np.sqrt(10 / float(i + 1)) * 2.5)
                cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

            # show the frame to our screen
            cv2.imshow("Frame", frame)
            cv2.imshow("Mask", mask)
            key = cv2.waitKey(1) & 0xFF

    '''
        Finds normalized distance of the blob from center of screen.
        e.g. x,y, value of 0,0 means blob is in exact center.
        x,y value of 1,1 means blob is in bottom right corner.
        x,y value of -1,-1 means blob in top left corner.
    '''
    def __get_blob_relative_position__(self):
        rows = float(self.im.shape[0])
        cols = float(self.im.shape[1])
        center_x = 0.5 * cols
        center_y = 0.5 * rows
        x = (self.horizontal_pos - center_x) / center_x
        y = (self.vertical_pos - center_y) / center_y
        return (x, y)


    '''
        Combines multiple black and white masks into one big mask.
    '''
    def __get_mask__(self, im_hsv) :
        #mask image using each hsv filter
        masks = []
        for hsv in self.hsv_thresholds :
            hsv_min = (hsv[0], hsv[1], hsv[2])
            hsv_max = (hsv[3], hsv[4], hsv[5])
            masks.append(cv2.inRange(im_hsv, hsv_min, hsv_max))

        #combine images into one big mask

        result = masks[0]
        for m in masks:
            result = cv2.add(result, m)

        return result
    # endregion

    # region [Threshold Reading]

    '''
        Reads in HSV thresholds from xml file.
    '''
    def __read_hsv_thresholds__(self, threshold_color) :
        threshold_list = []

        current_dir = Path(__file__).parent.resolve()
        
        tree = ET.parse(str(current_dir / 'thresholds.xml'))
        root = tree.getroot()

        color_node = root.find(threshold_color)

        for thresh in list(color_node) :
            hsv_thresh = []
            for hsv_val in list(thresh) :
                hsv_thresh.append(int(hsv_val.text))
            threshold_list.append(hsv_thresh)
        
        return threshold_list

    # endregion

'''
    Abstract representation of forward, downward, yaw velocities.
'''
class Commanded_Velocity :

    def __init__(self, forward, downward, yaw_rate) :
        self.forward = forward
        self.downward = downward
        self.yaw_rate = yaw_rate
