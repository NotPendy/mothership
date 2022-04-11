'''
    Author: Robby Rivenbark
    Email: rsrivenb@ncsu.edu

    Purpose: To allow the mothership to locate the baby ship, face the baby ship, and advance toward the baby ship.
'''

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import os
from imutils.video import VideoStream
from frame_processor import Frame_Processor
import time

class Vision_Controller :
    

    # region [init]

    '''
        start video capture, drone connection
    '''
    def __init__(self) :

        # Initialize frame processor
        self.frame_processor = Frame_Processor()

        self.cap = VideoStream(src=0).start()

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
        Drone creates increasingly large squares.
        The drone will stop at fixed intervals in its travel to seek for the red ball.
        When the drone finds the ball, it exits the loop.
    '''
    def seek(self) :

        unit_length = 1
        units_per_side = 2

        while True :
            #move forward by units_per_side / 2
            if self.__turn_step__(units_per_side/2, unit_length, 0) :
                return True
            
            #turn 90 degrees, move forward by units_per_side / 2
            if self.__turn_step__(units_per_side/2, unit_length, 90) :
                return True

            #turn 90 degrees, move forward by units_per_side
            for _ in range(3) :
                if self.__turn_step__(units_per_side, unit_length, 90) :
                    return True
            
            #turn 90 degrees, move forward by units_per_side / 2
            for _ in range(2) :
                if self.__turn_step__(units_per_side/2, unit_length, 90) :
                    return True
            
            #turn 180 degrees
            if self.__turn_step__(0, unit_length, 180) :
                return True
            
            #increase units per side
            units_per_side *= 2

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
        #cap = VideoStream(src=0).start()

        successful_exit = False

        #check if capture opened correctly
        if not self.cap.isOpened():
            print("Cannot open camera")
            return
        
        #start frame read loop
        while True :
            #get frame
            frame = self.cap.read()

            # if frame is read correctly ret is True
            if frame is None:
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
            self.vehicle.send_mavlink(self.__move_by_cmd_vel_msg__(cmd_vel))

            #if the drone has been told to stop, this function's job is done.
            if stop and stopped :
                successful_exit = True
                break

        #reset flags modified by frame processor
        self.frame_processor.reset_flags()

        #release video capture
        #cap.release()

        return successful_exit

    # endregion

    # region [Primary Helpers]

    '''
        Drone checks in a circle for red ball.
        Drone rotates given amount.
        Drone makes units_per_side units of unit_length forward.
    '''
    def __turn_step__(self, units_per_side, unit_length, turn_amount_degrees) :
        #check for ball.
        if self.__circle_seek__() :
            return True
        
        #turn by turn_amount_degrees.
        self.vehicle.send_mavlink(self.__condition_yaw_msg__(turn_amount_degrees))
        time.sleep(2)

        #move units_per_side units of unit_length forward.
        for _ in range(units_per_side) :
            self.vehicle.send_mavlink(self.__move_forward_meters_msg__(unit_length))


    # endregion

    # region [Secondary Helpers]

    '''
        Rotates drone in a circle. Periodically checks to see if ball in frame.
    '''
    def __circle_seek__(self) :
        #do 16 rotations of pi/8 rads (22.5 degrees)
        for rot in range(16) :

            #before each rotation, check for blob.
            while True :

                #get frame
                frame = self.cap.read()
                # if frame is read correctly ret is True
                if frame is None:
                    print("Can't receive frame (stream end?). Exiting ...")
                    break

                frames_with_red = 0

                for _ in range(5) :
                    red_in_frame = self.frame_processor.process_frame_check_red(frame)
                    if red_in_frame :
                        frames_with_red += 1

                #if all frames have red ball in them, return True
                if frames_with_red == 5 :
                    return True
                
                #if no frames have red, keep rotating.
                if frames_with_red == 0 :
                    break

                #otherwise keep rotating
            
            #create and send the rotation command
            self.vehicle.send_mavlink(self.__condition_yaw_msg__(22.5))

            #wait for drone to rotate
            time.sleep(0.25)

            return False

    # endregion

    # region [Movement Helpers]

    '''
        Constructs and returns SET_POSITION_TARGET_LOCAL_NED message with the given
        forward, downward velocities and yaw rate to cmd_vel.
    '''
    def __move_by_cmd_vel_msg__(self, cmd_vel):

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
    
    '''
        Constructs and returns SET_POSITION_TARGET_LOCAL_NED message with the given
        forward velocity and distance to move forward
    '''
    def __move_forward_meters_msg__(self, distance, vel_forward=2):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b110111000110, # type_mask (only speeds enabled)
            distance, 0, 0, # x, y, z positions (not used)
            vel_forward, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        return msg
    
    '''
        Constructs and returns CONDITION_YAW message with the given heading.
    '''
    def __condition_yaw_msg__(self, heading, relative=True):
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        
        return msg

    # endregion
