"""
code to run a servo off a rasberry pi using GPIO pin 25

commands to run to install needed libraries
sudo apt-get install python3-rpi.gpio
sudo pip3 install gpiozero
"""


from gpiozero import Servo
from gpiozero import Button
from time import sleep

servo = Servo(25)
button = Button(23, False)

print("Servo going to min posistion (aka release)")
servo.min()
sleep(1)


print("servo going to mid posistion (should be pickup)")
servo.mid()
sleep(1)

print("wait for button")
button.wait_for_press()
print("got button")

servo.max()
