#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Code for the Babyship drone. Currently connects to the TCP address specified when running program with --connect argument, takes off to a target altitude, 
sets the parameters for throw mode, sets mode to throw mode, turns off the motors and lets drone catch it self, and then land at a fixed position.

currently the issue is i can not set the mode to throw form guided as its already flying. This may not be an issue if its not powered by its own power but may
need to start teleported in the air to do this.
"""

from __future__ import print_function
import math
from random import seed, random
import time
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Size of square in meters
SQUARE_SIZE = 10
# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 10
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
VELOCITY_DIFFERENCE = 1
# variable to determine how much of a difference in posistion two objects can be and not be considered connected and flying together
POSITION_DIFFERENCE = 1
#height the mothership should be at when picking up the babyship on the ground in meters
PICKUP_HEIGHT = 1
#distance mothership should be from the babyship to begin locating it with the camera in meters
PICKUP_DISTANCE = 15

def send_mothership_to_babyship(mothership, babyship):
    """
    This function takes in two drones "mothership" and "babyship" and sends the mothership to the location of the babyship.
    Function does not end until the vehicle is within the the waypoint limit.
    """
    location_offset_meters = pickup_position(mothership, babyship)
    pickup_coordinates = meter_offset_to_coords(location_offset_meters, vehicle)
    print("sending mothership to babyship general area")

    mothership.simple_goto(LocationGlobalRelative(pickup_coordinates[0], pickup_coordinates[1], pickup_coordinates[2]))

    while distanceToWaypoint(LocationGlobalRelative(pickup_coordinates[0], pickup_coordinates[1], pickup_coordinates[2]), vehicle) > WAYPOINT_LIMIT:
        time.sleep(.5)

    print("Mothership in position to pickup babyship")
    return

def meter_offset_to_coords(offset, drone):
    """
    passes an array "offset" with three values [x, y, z] and adds it to the current gps position of the vehicle drone
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters

    """

    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = offset[0]/earth_radius
    dLon = offset[1]/(earth_radius*math.cos(math.pi*drone.location.global_relative_frame.lat/180))

    #New position in decimal degrees
    newlat = drone.location.global_relative_frame.lat + (dLat * 180/math.pi)
    newlon = drone.location.global_relative_frame.lon + (dLon * 180/math.pi)
    coordinates = [newlat, newlon, offset[2]]
    return coordinates

def drone_connected(drone1, drone2):
    """
    passes two vehicle objects in and determines based on location and acceleration if they are connected and flying as one object. 
    return True if connected, return False if not connected
    """
    pos_difference = abs(drone1.location.global_relative_frame.lat - drone2.location.global_relative_frame.lat) + abs(drone1.location.global_relative_frame.lon - drone2.location.global_relative_frame.lon) + abs(drone1.location.global_relative_frame.alt - drone2.location.global_relative_frame.alt)
    vel_difference = abs(drone1.velocity[0] - drone2.velocity[0]) + abs(drone1.velocity[1] - drone2.velocity[1]) + abs(drone1.velocity[2] - drone2.velocity[2]) 

    if((pos_difference < POSITION_DIFFERENCE) and (vel_difference < VELOCITY_DIFFERENCE)):
        return True

    else:
        return False

def pickup_position(drone1, drone2):
    """
    function to find the exact position mothership should be at when picking up the babyship. 
    """
    relative_lat = drone1.location.global_relative_frame.lat - drone2.location.global_relative_frame.lat - PICKUP_DISTANCE
    relative_lon = drone1.location.global_relative_frame.lon - drone2.location.global_relative_frame.lon

    pickup_location = [relative_lat, relative_lon, PICKUP_HEIGHT]
    return pickup_location

def safe_to_fly(drone):
    """
    Gets the current battery voltage, current, and level. Compares battery level to a set stop point called BATTERY_LIMIT. 
    If the battery level is less then this the function will return FALSE, else it will return TRUE
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

def distanceToWaypoint(coordinates, drone):
    """
    Returns distance between vehicle and specified coordinates as GlobalRelativeFrame objects
    """
    distance = get_distance_metres(drone.location.global_frame, coordinates)
    return distance
    
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
    x = vehicle.battery
    print(x)


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
    condition_yaw(0)

    #random location to test
    #offset = [25*random(), 25*random(), 5]
    #non random location to test
    offset = [-50, -50, 10]
    fly_location = meter_offset_to_coords(offset, vehicle)
    start = time.time()
    
    vehicle.simple_goto(LocationGlobalRelative(fly_location[0], fly_location[1], fly_location[2]))
    print("babyship flying to the location")
    while get_distance_metres(LocationGlobalRelative(fly_location[0], fly_location[1], fly_location[2]), vehicle.location.global_relative_frame) > WAYPOINT_LIMIT:
        time.sleep(.5)
        if(time.time() - int(start) > 100):
            print("babyship timed out")
            break

    print("babyship is in location")

    time.sleep(5)



    #sets the parameters for throw mode, THROW_MOT_START = 0 for no spinning of motors, THROW_TYPE = 1 for dropping from 10 meters up or higher, and NEXT_MODE = 4 for guided mode after stabilize
    """
    vehicle.parameters['THROW_MOT_START']=0
    vehicle.parameters['THROW_TYPE']=1  
    vehicle.parameters['NEXT_MODE']=4
    print("parameters set")

    time.sleep(2)

    vehicle.mode = VehicleMode("THROW")
    print("setting mode throw")


    time.sleep(5)
    """




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
