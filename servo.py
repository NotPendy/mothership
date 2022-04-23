"""
code to run a servo off a rasberry pi using GPIO pin 25 and GPIO pin 23

commands to run to install needed libraries
sudo apt-get install python3-rpi.gpio
sudo pip3 install gpiozero
"""


from gpiozero import Servo
from gpiozero import Button
from time import sleep

servo = Servo(25)
limitswitch = Button(23, False)


print("servo going to min posistion (should be releasing baby)")
servo.value = -1
sleep(3)

print("servo going to mid posistion (should be ready to pickup baby)")


print("waiting for limit switch to be pressed (will set servo to closed posistion)")

servo.value = .75

