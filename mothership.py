#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Code for the Mothership drone. Currently connects to the TCP address specified when running program with --connect argument, takes off to a target altitude, lands back at the same position, and then disarms to end program
"""

from __future__ import print_function

import math
import time
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Size of square in meters
SQUARE_SIZE = 10
# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 3
# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
# Variable to keep track of if joystick to arm has returned to center
rcin_4_center = False
# in a range of 0 to 1 the percentage of battery needed to fly
BATTERY_SAFE = 0.5
# variable to determine how much of a difference in acceleration two objects can be to not be considered connected and flying together
ACCELERATE_DIFFERENCE = 1
# variable to determine how muhc of a difference in posistion two objects can be and not be considered connected and flying together
POSITION_DIFFERENCE = 1

def drone_connected(drone1, drone2):
    """
    passes two vehicle objects in and determines based on location and acceleration if they are connected and flying as one object. 
    return True if connected, return False if not connected
    """
    pos_difference = abs(drone1.location.global_relative_frame.lat - drone2.location.global_relative_frame.lat) + abs(drone1.location.global_relative_frame.lon - drone2.location.global_relative_frame.lon) + abs(drone1.location.global_relative_frame.alt - drone2.location.global_relative_frame.alt)
    if(pos_difference < POSITION_DIFFERENCE):
        return True
    



def safe_to_fly(drone):
    """
    Gets the current battery voltage, current, and level. Compares battery level to a set stop point called BATTERY_LIMIT. If the battery level is less then this the function will return FALSE, else 	   it will return TRUE
    """
    if(drone.battery.level <= BATTERY_SAFE):
        return False
    else:
        return True

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distanceToWaypoint(coordinates):
    """
    Returns distance between vehicle and specified coordinates
    """
    distance = get_distance_metres(vehicle.location.global_frame, coordinates)
    return distance

def get_location_metres(original_location, dNorth, dEast, altitude):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlat, newlon, altitude)

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

    # delay to wait until yaw of copter is at desired yaw angle
    time.sleep(3)

# Set up option parsing to get connection strings
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string.")
parser.add_argument('--connect2', help="Second vehicle target string.")

args = parser.parse_args()

# aquire connection_strings
connection_string = args.connect
connection_string2 = args.connect2

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string for main vehicle')

if not connection_string2:
    sys.exit('Please specify connection string for second vehicle')

# Connect to the Vehicle
print('Connecting to main vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print('Succesfully connected to main vehicle')

print('Connecting to second vehicle on: %s' %connection_string2)
vehicle2 = connect(connection_string2, wait_ready=True)
print('Succesfully connected to second vehicle')


"""
Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
has returned to center for two consecutive seconds.
"""
@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)

@vehicle2.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)

if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle.mode = VehicleMode("ALT_HOLD")

if vehicle2.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle2.mode = VehicleMode("ALT_HOLD")
    
    
# Wait for pilot before proceeding
print('Waiting for safety pilot to arm both vehicles')

# while loop to test functions
"""while True:
	drone_connected(vehicle, vehicle2)
"""
# Wait until safety pilot arms drone
while not vehicle.armed:
    time.sleep(1)
while not vehicle2.armed:
    time.sleep(1)
    
print('Armed...')
vehicle.mode = VehicleMode("GUIDED")
vehicle2.mode = VehicleMode("GUIDED")

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
        print(drone_connected(vehicle, vehicle2))
        
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            break
        time.sleep(0.5)
    # yaw north
    condition_yaw(0)



print('Landing')
if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
    # Land Copter
    vehicle.mode = VehicleMode("LAND")

if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
    # disarm Rover
    vehicle.armed = False

# Stay connected to vehicle until landed and disarmed
while vehicle.armed:
    time.sleep(1)

print("Done!")

# Close vehicle object before exiting script
vehicle.close()
