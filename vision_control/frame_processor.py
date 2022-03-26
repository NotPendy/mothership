'''
    Author: Robby Rivenbark
    Email: rsrivenb@ncsu.edu

    Purpose: Maintains data and functionality for reading in a frame of video from the mothership and deciding what 
    movements should be made. 
    
    Does not directly interface with hardware, does not need to know where images are coming from or how the velocity
    information gets to the drone, it just uses these as input and output.
'''

from pickle import FALSE
import cv2
import math
import time
import numpy as np

class Frame_Processor:

    # region [init]

    MAX_YAW_RATE = np.pi / 8
    MAX_VERTICAL_SPEED = .1
    FORWARD_VELOCITY = .1

    CENTERED_TOLERANCE_HORIZONTAL = .05
    CENTERED_TOLERANCE_VERTICAL = .05

    '''
        Constructor sets up necessary blob detection parameters
    '''
    def __init__(self, red_hsv_min = (0, 34, 0), red_hsv_max = (10,255,255)):

        self.centered_red_horizontally = False

        #hsv thresholds
        self.red_hsv_min = red_hsv_min
        self.red_hsv_max = red_hsv_max

        red_params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        red_params.minThreshold = 0
        red_params.maxThreshold = 100

        # Filter by Area.
        red_params.filterByArea = True
        red_params.minArea = 10
        red_params.maxArea = 1000000

        # Filter by Circularity
        red_params.filterByCircularity = True
        red_params.minCircularity = 0.7

        # Filter by Convexity
        red_params.filterByConvexity = True
        red_params.minConvexity = 0.9

        # Filter by Inertia
        red_params.filterByInertia = True
        red_params.minInertiaRatio = 0.9

        self.red_detector = cv2.SimpleBlobDetector_create(red_params)

    # endregion

    # region [High-Level Algorithms]

    def process_frame_check_red(self, frame) :
        self.__find_red_blobs__(frame)

        return self.red_in_frame


    def center_vertically(self, frame) :

        #Run blob detection on frame
        self.__find_red_blobs__(frame)

        if not self.centered_red_vertically :
            return self.__center_red__(horizontal=False)

        else :
            return self.__stop__()

    '''
        This function is called to process one frame of video.
        
        It returns the velocity that should be commanded to the vehicle
    '''
    def center_horizontally_and_advance(self, frame):

        #Run blob detection on frame
        self.__find_red_blobs__(frame)

        #If the camera is not centered on red, center on red
        if not self.centered_red_horizontally :
            return self.__center_red__(horizontal=True)
    
        return self.__advance__()

    '''
        resets all flags that may have been modified by frame processing
    '''
    def reset_flags(self) :
        self.centered_red_horizontally = False


    # endregion

    # region [Primary Helpers]

    '''
        Commands drone to make necessary adjustments to center camera on red ball
    '''
    def __center_red__(self, horizontal, modify_conditions = True, record = True) :

        #if we don't detect a blob, seek
        if len(self.red_keypoints) == 0:
            return self.__seek__()

        #get biggest red keypoint (we'll assume the ball is the biggest red blob in frame)
        red_keypoint = self.red_keypoints[0]
        for r in self.red_keypoints:
            red_keypoint = red_keypoint if r.size < red_keypoint.size else r 

        #get normalized x position of blob in frame
        horizontal_position_blob, vertical_position_blob = self.__get_blob_relative_position__(self.red_im, red_keypoint)

        #calculate commanded velocity based off of blob location in frame
        forward_rate = 0.0
        #TODO: are signs right on these?
        vertical_rate = 0.0 if horizontal else self.MAX_VERTICAL_SPEED * vertical_position_blob
        yaw_rate = -self.MAX_YAW_RATE * horizontal_position_blob if horizontal else 0.0
        cmd_vel = Commanded_Velocity(forward_rate, vertical_rate, yaw_rate)

        if modify_conditions and horizontal :
            #if the red ball's position within frame is within tolerance, set centered_red flag to true
            if math.fabs(horizontal_position_blob) < self.CENTERED_TOLERANCE_HORIZONTAL :
                self.centered_red_horizontally = True
            else :
                self.centered_red_horizontally = False
        
        elif modify_conditions and not horizontal :
            #if the red ball's position within frame is within tolerance, set centered_red flag to true
            if math.fabs(vertical_position_blob) < self.CENTERED_TOLERANCE_VERTICAL :
                self.centered_red_vertically = True
            else :
                self.centered_red_vertically = False

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
        if len(self.red_keypoints) == 0:
            current_time = time.time()

            #If the time between now and the last commanded velocity is less than 1 second, follow through
            if None is self.last_cmd_vel_time or self.__time_difference_seconds__( self.last_cmd_vel_time, current_time ) <= 1.0 :
                return self.__follow_through__()
            #otherwise, stop
            else :
                self.centered_red_horizontally = False
                return self.__stop__()
        
        #call center red to make any angular adjustments
        cmd_vel = self.__center_red__(self, frame, modify_conditions=False, record = False)
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
        return Commanded_Velocity(0,0,-1.0*clockwise), False

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
    def __find_red_blobs__(self, frame, show = False) :
        self.red_im = frame

        red_hsv = cv2.cvtColor(self.red_im, cv2.COLOR_BGR2HSV)
        
        # standard opencv blob detection magic.
        red_mask = cv2.inRange(red_hsv, self.red_hsv_min, self.red_hsv_max)
        red_mask = cv2.dilate(red_mask, None, iterations=2)
        red_mask = cv2.erode(red_mask, None, iterations=2)

        red_reverse_mask = 255 - red_mask
        self.red_keypoints = self.red_detector.detect(red_reverse_mask)

        self.red_in_frame = not(len(self.red_keypoints) == 0)

    '''
        Finds normalized distance of the blob from center of screen.
        e.g. x,y, value of 0,0 means blob is in exact center.
        x,y value of 1,1 means blob is in bottom right corner.
        x,y value of -1,-1 means blob in top left corner.
    '''
    def __get_blob_relative_position__(self, image, keyPoint):
        rows = float(image.shape[0])
        cols = float(image.shape[1])
        center_x = 0.5 * cols
        center_y = 0.5 * rows
        x = (keyPoint.pt[0] - center_x) / center_x
        y = (keyPoint.pt[1] - center_y) / center_y
        return (x, y)

    # endregion

'''
    Abstract representation of forward, downward, yaw velocities.
'''
class Commanded_Velocity :

    def __init__(self, forward, downward, yaw_rate) :
        self.forward = forward
        self.downward = downward
        self.yaw_rate = yaw_rate