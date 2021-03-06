'''
    Author: Robby Rivenbark
    Email: rsrivenb@ncsu.edu, robbyisgreat@gmail.com

    Purpose: To allow the mothership to locate the baby ship, center camera on the baby ship, and advance toward the baby ship.

    Meant to take vehicle in constructor and then have high-level functions that you can easily call for seeking, centering.

    e.g If you had a vehicle in a calling program you could initialize a vision controller:
    vision_controller = Vision_Controller(vehicle)

    you could then tell it to seek the target:
    vision_controller.translate_seek()

    center vertically:
    vision_controller.center_in_direction(horizontal=False)

    center horizontally:
    vision_controller.center_in_direction(horizontal=True)

    This can all be done simply with a few function calls.
'''

from pymavlink import mavutil
from imutils.video import VideoStream
from vision_control.frame_processor import Frame_Processor
import time

class Vision_Controller :
    

    # region [init]

    DEFAULT_VELOCITY = 2

    '''
        start video capture, drone connection
    '''
    def __init__(self, vehicle) :

        # Initialize frame processor
        self.frame_processor = Frame_Processor("default", rotating_seek=False)

        self.cap = VideoStream(src=0).start()

        self.vehicle = vehicle

    '''
        Closes vehicle connection upon exit
    '''        
    def __del__(self):
        print("Exiting")
        self.vehicle.close()
        self.cap.stop()
    
    # endregion

    # region [High Level Algorithms]

    '''
        UNTESTED, NOT USED

        Drone creates increasingly large squares.
        The drone will stop at fixed intervals in its travel to seek for the red ball.
        When the drone finds the ball, it exits the loop.
    '''
    def seek(self, show=False) :

        unit_length = 1
        units_per_side = 2

        while True :
            #move forward by units_per_side / 2
            if self.__turn_step__(int(units_per_side/2), unit_length, 0, show=show) :
                return True
            
            #turn 90 degrees, move forward by units_per_side / 2
            if self.__turn_step__(int(units_per_side/2), unit_length, 90, show=show) :
                return True

            #turn 90 degrees, move forward by units_per_side
            for _ in range(3) :
                if self.__turn_step__(units_per_side, unit_length, 90, show=show) :
                    return True
            
            #turn 90 degrees, move forward by units_per_side / 2
            for _ in range(2) :
                if self.__turn_step__(int(units_per_side/2), unit_length, 90, show=show) :
                    return True
            
            #turn 180 degrees
            if self.__turn_step__(0, unit_length, 180, show=show) :
                return True
            
            #increase units per side
            units_per_side *= 2

    '''
        slides 1 meter to the right, looks for blobs, slides back to middle, looks for blobs, 
        slides 1 meter to the left, looks for blobs, slides back to middle, looks for blobs

        Currently not too smart but gets the job done. Could modify to make step start smaller and increase over time.
    '''
    def translate_seek(self, show = False) :

        unit_length = 1
        units_per_side = 1

        while True :

            if self.__check_10_frames__(show=show) :
                return

            # Slide right
            self.__slide__(units_per_side, unit_length, 1, show=show)
            # Read 10 frames and check for target
            if self.__check_10_frames__(show=show) :
                return
            
            # Slide left
            self.__slide__(units_per_side, unit_length, -1, show=show)
            # Read 10 frames and check for target
            if self.__check_10_frames__(show=show) :
                return
            
            # Slide left
            self.__slide__(units_per_side, unit_length, -1, show=show)
            # Read 10 frames and check for target
            if self.__check_10_frames__(show=show) :
                return
            
            # Slide right
            self.__slide__(units_per_side, unit_length, 1, show=show)
            # Read 10 frames and check for target
            if self.__check_10_frames__(show=show) :
                return
            


    '''
        Centers drone camera on target ball horizontally, drone advances toward target (baby).
        
        Continually reads in frames of video and sends them out to be processed.
        Uses output of frame processor to command the drone to move.

        If centering horizontally, the drone will advance after it is sufficiently centered.

        Returns True if image loop exits successfully (drone centers on target, advances to 
        target, follows through and stops) or False if the image loop ends in any other way.

        @param horizontal: True if horizontal centering, False if vertical centering
        @param stop: True if you want this function to naturally end, False if you want it to keep looping forever.
        @param advance: True if you want the drone to advance after horizontal centering.
        @param show: True if you want video output, False otherwise (must be false if running headless on pi without VNC)

    '''
    def center_in_direction(self, horizontal, stop = True, show = False, advance=True) :

        #initialize video capture
        #cap = VideoStream(src=0).start()

        successful_exit = False

        #start frame read loop
        while True :
            #get frame
            frame = self.cap.read()

            # if frame is read correctly ret is True
            if frame is None:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            cmd_vel = None
            stopped = None

            #send frame to be processed and recieve commanded velocity
            if horizontal : 
                cmd_vel, stopped = self.frame_processor.center_horizontally_and_advance(frame, advance=advance ,show=show)
            else :
                cmd_vel, stopped = self.frame_processor.center_vertically(frame,stop_when_centered=stop,show=show)

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
        NOT FULLY TESTED, NOT IN USE FOR MAIN CODE.

        Drone checks in a circle for red ball.
        Drone rotates given amount.
        Drone makes units_per_side units of unit_length forward.
    '''
    def __turn_step__(self, units_per_side, unit_length, turn_amount_degrees, show=False) :
        #check for ball.
        #if self.__circle_seek__(show=show) :
        #    return True
        
        #turn by turn_amount_degrees.
        self.vehicle.send_mavlink(self.__condition_yaw_msg__(turn_amount_degrees))
        print("turning")
        time.sleep(5)

        #move units_per_side units of unit_length forward.
        for _ in range(units_per_side) :
            self.vehicle.send_mavlink(self.__move_forward_meters_msg__(unit_length))
            print("forward")
            time.sleep(5)
    
    '''
        Commands drone to slide sideways a certain distance, while maintaining the direction it's facing.
        direction is 1 for right, -1 for left.
        units length is distance to be moved
        units_per_side is number of times to move
    '''
    def __slide__(self, units_per_side, unit_length, direction, show=False) :

        #move units_per_side units of unit_length forward.
        for _ in range(units_per_side) :
            self.vehicle.send_mavlink(self.__move_sideways_meters_msg__(unit_length, vel_sideways=direction*self.DEFAULT_VELOCITY))
            print("sliding")
            time.sleep(10)
    
    '''
        Checks next 10 frames of video and sees if target appears.
    '''
    def __check_10_frames__(self, show=False) :
        for _ in range(10) :
            frame = self.cap.read()
            if frame is None:
                print("Can't receive frame (stream end?). Exiting ...")
                return True

            if self.frame_processor.process_frame_check_blob(frame, show=show) :
                print("found")
                return True


    # endregion

    # region [Secondary Helpers]

    '''
        NOT TESTED, TEST BEFORE USING.

        Rotates drone in a circle. Periodically checks to see if ball in frame.
    '''
    def __circle_seek__(self, show=False) :
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
                    red_in_frame = self.frame_processor.process_frame_check_blob(frame, show=show)
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
            time.sleep(2)

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
            distance, 0, 0, # x, y, z positions
            vel_forward, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        return msg
    
    '''
        Slides drone sideways by the specified distance w/ specified speed. Maintains yaw.
    '''
    def __move_sideways_meters_msg__(self, distance, vel_sideways):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b000111000101, # type_mask (only speeds enabled)
            0, distance, 0, # x, y, z positions
            0, vel_sideways, 0, # x, y, z velocity in m/s
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
