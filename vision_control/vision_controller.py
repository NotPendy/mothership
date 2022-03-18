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
        self.cam = cv2.VideoCapture(0)

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

    # region [Main Flow]

    def image_loop(self) :
        frame_processor = Frame_Processor()
        

    # endregion
