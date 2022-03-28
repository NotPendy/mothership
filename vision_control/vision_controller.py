'''
    Author: Robby Rivenbark
    Email: rsrivenb@ncsu.edu

    Purpose: To allow the mothership to locate the baby ship, face the baby ship, and advance toward the baby ship.
'''

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
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

    def seek(self) :
        unit_length = 1
        units_per_side = 1

        


    '''
        Centers drone camera on target ball horizontally, drone advances toward target (baby).
        
        Continually reads in frames of video and sends them out to be processed.
        Uses output of frame processor to command the drone to move.

        Shouldn't change vertical position of drone, this function should only be called after
        the target ball is in vision and is vertically aligned with camera.

        Returns True if image loop exits successfully (drone centers on target, advances to 
        target, follows through and stops) or False if the image loop ends in any other way.
    '''
    def center_in_direction(self, horizontal, stop = True) :

        #initialize video capture
        cap = cv2.VideoCapture(0)

        successful_exit = False

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

            cmd_vel
            stopped

            #send frame to be processed and recieve commanded velocity
            if horizontal : 
                cmd_vel, stopped = self.frame_processor.center_horizontally_and_advance(frame)
            else :
                cmd_vel, stopped = self.frame_processor.center_vertically(frame)

            #command drone to perform specified action
            self.vehicle.send_mavlink(self.__move_by_cmd_vel__(cmd_vel))

            #if the drone has been told to stop, this function's job is done.
            if stop and stopped :
                successful_exit = True
                break

        #reset flags modified by process_frame_target_acquired()
        self.frame_processor.reset_flags()

        #release video capture
        cap.release()

        return successful_exit

    # endregion

    # region [Movement Helpers]

    '''
        Constructs and returns SET_POSITION_TARGET_LOCAL_NED message with the given
        forward, downward velocities and yaw rate to cmd_vel
    '''
    def __move_by_cmd_vel__(self, cmd_vel):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b010111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            cmd_vel.forward, 0, cmd_vel.downward, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, cmd_vel.yaw_rate)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        return msg

    # endregion
