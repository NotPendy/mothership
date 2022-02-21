"""
code to run a servo off a rasberry pi using GPIO pin 25

commands to run to install needed libraries
sudo apt-get install python3-rpi.gpio
sudo pip3 install gpiozero
"""


from gpiozero import Servo
from time import sleep

servo = Servo(25)

try:
	while True:
    	servo.min()
    	sleep(0.5)
    	servo.mid()
    	sleep(0.5)
    	servo.max()
    	sleep(0.5)
except KeyboardInterrupt:
	print("Program stopped")