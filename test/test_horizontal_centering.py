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

greenLower = (20, 46, 27)#(21, 121, 131)
greenUpper = (50, 255, 255)

frame_processor = Frame_Processor(rotating_seek=False)

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

    #print(cmd_vel.yaw_rate)

#release video capture
cap.stop()

