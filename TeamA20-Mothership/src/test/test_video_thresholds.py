'''
    Continuously reads in frames of video, uses frame processor to track target and output position to terminal.

    Good for testing thresholds on video collected from field.

    Author: Robby Rivenbark
'''
from pathlib import Path
import sys
path_root = Path(__file__).parents[1]
print(path_root)
sys.path.append(str(path_root))
from vision_control.frame_processor import Frame_Processor
import cv2
from collections import deque
import time
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from imutils.video import FileVideoStream
import os

loc = str(os.path.dirname(os.path.realpath(__file__))) + "/test_pics/testvid.webm"
print(loc)

frame_processor = Frame_Processor("default",rotating_seek=False)

#initialize video capture
cap = FileVideoStream(loc).start()

time.sleep(2.0)

#start frame read loop
while True :
    #get frame
    frame = cap.read()

    # if frame is read correctly ret is True
    #if frame is None:
        #print("Can't receive frame (stream end?). Exiting ...")
        #break

    #send frame to be processed and recieve commanded velocity
    cmd_vel, stopped = frame_processor.center_horizontally_and_advance(frame, show=True, advance=False)

    #print(cmd_vel.yaw_rate)

    time.sleep(.01)

#release video capture
cap.stop()

