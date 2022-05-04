'''
    Continuously reads in frames of video, outputs commanded vertical velocities to terminal based on blob position.

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

frame_processor = Frame_Processor("orange",rotating_seek=False)

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
    cmd_vel, stopped = frame_processor.center_vertically(frame, show=True, stop_when_centered=False)

    #print(cmd_vel.yaw_rate)

#release video capture
cap.stop()

