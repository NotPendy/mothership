#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Code for the Mothership drone. Currently connects to the TCP address specified when running program with --connect argument, connects to the second drone on --connect2 argument takes off to a target altitude, lands back at the same position, and then disarms to end program
"""

import math
import time
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import RPi.GPIO as GPIO
from vision_control.vision_controller import Vision_Controller

servoPIN = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50) # GPIO 25 for PWM with 50Hz



# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 10
# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate, 1 Meter is good as of field test 4/25/22
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
PICKUP_HEIGHT = 2
#distance mothership should be from the babyship to begin locating it with the camera in meters
PICKUP_DISTANCE = 2.5
#max difference in the amount of thrust needed to hover the mothership before release and after retreiving the babyship. Should hypothetically be 0 if babyship is perfectly secured in same position.
HOVER_DIFFERENCE = 0.05
#PWM to hold the servo in lock posistion
HOLD_PWM = 9
#PWM to release the baby
DROP_PWM = 2.5
#PWM to pickup the baby
PICKUP_PWM = 6


def send_mothership_to_babyship():
    """
    function to send the mothership to the babyship by flying above the location of the babyship at PICKUP_HEIGHT meters and then flying south of Babyship at PICKUP_DISTANCE meters.
    Relies on the babyship 
    """
    #flying mother to babyship at PICKUP_HEIGHT meters above
    baby_pickup_location = LocationGlobalRelative(baby.location.global_frame.lat, baby.location.global_frame.lon, PICKUP_HEIGHT)
    mother.simple_goto(baby_pickup_location)
    print("mother flying to baby")
    while(distanceToWaypoint(baby_pickup_location, mother) > WAYPOINT_LIMIT):
        time.sleep(.5)
        
    #updating the offset to the mother's current location - PICKUP_DISTANCE meters south
    offset = [-PICKUP_DISTANCE, 0, TARGET_ALTITUDE]
    fly_location = meter_offset_to_coords(offset, mother)

    baby_pickup_location = LocationGlobalRelative(fly_location[0], fly_location[1], fly_location[2])
    print("mother flying behind baby")
    mother.simple_goto(baby_pickup_location)
    while(distanceToWaypoint(baby_pickup_location, mother) > WAYPOINT_LIMIT):
        time.sleep(.5)
    return

def get_hover(drone):
    """
    this function will determine the amount of thrust needed to hover the drone at its current weight should be between 0.2  and 0.8
    """
    hover = drone.parameters['MOT_THR_HOVER']
    return hover

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

def drone_connected(drone1, drone2, ConnectedThrust):
    """
    passes two vehicle objects in and determines based on location and acceleration if they are connected and flying as one object. 
    return True if connected, return False if not connected
    """
    pos_difference = abs(drone1.location.global_relative_frame.lat - drone2.location.global_relative_frame.lat) + abs(drone1.location.global_relative_frame.lon - drone2.location.global_relative_frame.lon) + abs(drone1.location.global_relative_frame.alt - drone2.location.global_relative_frame.alt)
    vel_difference = abs(drone1.velocity[0] - drone2.velocity[0]) + abs(drone1.velocity[1] - drone2.velocity[1]) + abs(drone1.velocity[2] - drone2.velocity[2]) 
    hov_difference = abs(drone1.parameter['MOT_THR_HOVER'] - ConnectedThrust)


    if((pos_difference < POSITION_DIFFERENCE) and (vel_difference < VELOCITY_DIFFERENCE) and (hov_difference < HOVER_DIFFERENCE)):
        return True

    else:
        return False

def pickup_position(drone1, drone2):
    """
    function to find the exact position mothership should be at when picking up the babyship. 
    """

    relative_lat = (drone1.location.global_relative_frame.lat - drone2.location.global_relative_frame.lat)*111139
    relative_lon = (drone1.location.global_relative_frame.lon - drone2.location.global_relative_frame.lon)*111139
    #for northern hemisphere only
    if(drone1.location.global_relative_frame.lat > drone2.location.global_relative_frame.lat):
        relative_lat = relative_lat*-1
    if(drone1.location.global_relative_frame.lon > drone2.location.global_relative_frame.lon):
        relative_lon = relative_lon*-1

    relative_lon = relative_lon - PICKUP_DISTANCE
    
    pickup_location = [relative_lat, relative_lon, PICKUP_HEIGHT]
    print(pickup_location)
    
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

def climb_alt(drone, altitude):
    """
    send MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT to drone to climb to specified altitude

    Continue on the current course and climb/descend to specified altitude. 
    When the altitude is reached continue to the next command 
    (i.e., don't proceed to the next command until the desired altitude is reached.
    not supported in ardupilot
    """
    #Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
    msg = drone.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT, 0, 1, 0, 0, 0, 0, 0, altitude) #altitude in meters
    #send command to vehicle
    drone.send_mavlink(msg)
    while drone.location.global_frame.alt < altitude*ALTITUDE_REACH_THRESHOLD:
        time.sleep(.5)
    

def condition_yaw(drone ,heading, relative=False):
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
    msg = drone.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    drone.send_mavlink(msg)

    # delay to wait until yaw of copter is at desired yaw angle
    time.sleep(3)

# Set up option parsing to get connection strings
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Mother connection target string.")
parser.add_argument('--connect2', help="Baby connection target string.")

args = parser.parse_args()

# aquire connection_strings
connection_string = args.connect
connection_string2 = args.connect2

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string for mothership')

if not connection_string2:
    sys.exit('Please specify connection string for babyship')

# Connect to the Vehicle
print('Connecting to main vehicle on: %s' % connection_string)
mother = connect(connection_string, wait_ready=True)
print('Succesfully connected to mothership')

print('Connecting to second vehicle on: %s' %connection_string2)
baby = connect(connection_string2, wait_ready=True)
print('Succesfully connected to babyship')


"""
Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
has returned to center for two consecutive seconds.

"""
@mother.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)

@baby.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)

if mother.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    mother.mode = VehicleMode("ALT_HOLD")

if baby.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    baby.mode = VehicleMode("ALT_HOLD")
    

# Wait for pilot before proceeding. Both drones must be armed at about the same time or they quickly disarm.
print('Waiting for safety pilot to arm both vehicles')


# Wait until safety pilot arms drone
while not mother.armed:
    time.sleep(1)
while not baby.armed:
    time.sleep(1)
    
print('Armed...')
mother.mode = VehicleMode("GUIDED")
baby.mode = VehicleMode("GUIDED")

#takes off the mothership to  target altitude and stops once it reaches at least 95% of that height. 
 
if mother.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:

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
    p.start(2.5)
    time.sleep(1)
    mother.simple_takeoff(TARGET_ALTITUDE)  # Take off to target altitude

    while True:
         # Break just below target altitude.
        if mother.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            break
        time.sleep(.5)
    # yaw north
    condition_yaw(mother, 0)
    #untested hover detection code the function drone_connected also does not work without hover
    #MotherCarryingBabyHover = mother.parameters['MOT_THR_HOVER']

    #print("Mother carrying baby hover: %s" % MotherCarryingBabyHover)

"""
code here for mothership dropping the babyship
"""
#mothership will wait for baby to land before attempting to go to its position
print("waiting for baby to be in land mode")
while baby.mode != VehicleMode("LAND"):
    #waits until the mode of the bayship is set to "LAND" so the mothership knows when to go and pick it up
    time.sleep(1) 



#Once babyship is ready to be picked up the mothership is positioned away from babyship to allow the camera to find it

#printing the location of the mothership and babyship for debugging purposes
print("mother lat:", mother.location.global_frame.lat)
print("mother lon:", mother.location.global_frame.lon)
print("baby lat:", baby.location.global_frame.lat)
print("baby lon:", baby.location.global_frame.lon)



#one line function to send mother to a location behind the babyship. Can be repeated as many times as needed after it executes. No timeout is used so if WAYPOINT_LIMIT is not reached code will loop.
send_mothership_to_babyship()


#code for closing the servo
condition_yaw(mother, 0)
print('LANDING NEAR BABY')
if mother.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
    # Land Copter
    mother.mode = VehicleMode("LAND")

mother.mode = VehicleMode("GUIDED")
mother.simple_takeoff(PICKUP_HEIGHT)

"""
put vision stuff to attempt to skewer here
then close servo
"""

# Stay connected to vehicle until landed and disarmed
while mother.armed:
    time.sleep(1)

print("Done!")

# Close vehicle object before exiting script
baby.close()
mother.close()
