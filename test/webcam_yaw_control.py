'''
    Continuously reads in frames of video, commmands copter to change yaw rate based on ball 
    horizontal position.

    Author: Robby Rivenbark
'''
from pathlib import Path
import sys
path_root = Path(__file__).parents[1]
sys.path.append(str(path_root))
from vision_control.frame_processor import Frame_Processor
import cv2
from collections import deque
import time
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from imutils.video import VideoStream

# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 3
# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
# Variable to keep track of if joystick to arm has returned to center
rcin_4_center = False

# region [Helpers]

'''
    Constructs and returns SET_POSITION_TARGET_LOCAL_NED message with the given
    forward, downward velocities and yaw rate to cmd_vel
'''
def __move_by_cmd_vel__(cmd_vel):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
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


# region [Set Up]


# Set up option parsing to get connection string and mission plan file
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# aquire connection_string
connection_string = args.connect

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string')

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

print('Succesfully connected to vehicle')

"""
Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
has returned to center for two consecutive seconds.
"""
@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)


if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle.mode = VehicleMode("ALT_HOLD")

# Wait for pilot before proceeding
print('Waiting for safety pilot to arm...')

# Wait until safety pilot arms drone
while not vehicle.armed:
    time.sleep(1)

print('Armed...')
vehicle.mode = VehicleMode("GUIDED")

if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:

    rcin_4_center_once = False
    rcin_4_center_twice = False
    while not rcin_4_center_twice:
        if rcin_4_center:
            if rcin_4_center_once:
                rcin_4_center_twice = True
            else:
                rcin_4_center_once = True
        else:
            rcin_4_center_once = False
        time.sleep(1)
    
    # Takeoff to short altitude
    print("Taking off!")
    vehicle.simple_takeoff(TARGET_ALTITUDE)  # Take off to target altitude

    while True:
         # Break just below target altitude.
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            break
        time.sleep(0.5)
    # yaw north
    #condition_yaw(0)

# endregion

greenLower = (20, 46, 27)#(21, 121, 131)
greenUpper = (50, 255, 255)

frame_processor = Frame_Processor(greenLower, greenUpper, rotating_seek=False)

#initialize video capture
cap = VideoStream(src=0).start()

time.sleep(2.0)

#start frame read loop
while True :
    #get frame
    frame = cap.read()

    # if frame is read correctly ret is True
    if frame is None:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    #send frame to be processed and recieve commanded velocity
    cmd_vel, stopped = frame_processor.center_horizontally_and_advance(frame, show=True, advance=False)

    #command drone to perform specified action
    vehicle.send_mavlink(__move_by_cmd_vel__(cmd_vel))

    print(cmd_vel.yaw_rate)

#release video capture
cap.release()


