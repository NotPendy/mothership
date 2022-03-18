'''
    Author: Robby Rivenbark
    Email: rsrivenb@ncsu.edu

    Purpose: To allow the mothership to locate the baby ship, face the baby ship, and advance toward the baby ship.
'''

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import cv2
import os

from frame_processor import Frame_Processor

class Vision_Controller :
    

    # region [init]

    '''
        start video capture, drone connection
    '''
    def __init__(self) :

        # Initialize frame processor
        self.frame_processor = Frame_Processor()

        try:
            print("Creating Drone Object")
            connection_string = "udp:127.0.0.1:5760"
            self.vehicle = connect(connection_string, wait_ready=True)
            self.add_callback()
            self.vehicle.mode = VehicleMode("GUIDED")
        except KeyboardInterrupt:
            self.clean_exit()

    '''
        Closes vehicle connection upon exit
    '''        
    def __del__(self):
        print("Exiting")
        self.vehicle.close()

    '''
        Print a message whenever mode is changed
    '''
    def mode_callback(self,attr_name,value):
        #Check Mode
        self.mode = value.name
        print(f"CALLBACK: {value.name}")
        if value.name == 'LAND':
            os._exit(1)

    '''
        Add callback function(s) to vehicle
    '''
    def add_callback(self):
        self.vehicle.add_attribute_listener('mode.name', self.mode_callback)
    
    # endregion

    # region [High Level Algorithms]

    '''
        Continually reads in frames of video and sends them out to be processed.
        Uses output of frame processor to command the drone to move.
    '''
    def image_loop(self) :

        #initialize video capture
        cap = cv2.VideoCapture(0)

        #check if capture opened correctly
        if not cap.isOpened():
            print("Cannot open camera")
            return
        
        #start frame read loop
        while True :
            #get frame
            ret, frame = cap.read()

            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            #send frame to be processed
            cmd_vel = self.frame_processor.process_frame_target_acquired(frame)

            #command drone to perform specified 

        
        #release video capture

        cap.release()

    # endregion

    # region