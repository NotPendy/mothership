'''
  Author: Alex Pendergast
'''

import RPi.GPIO as GPIO
import time

servoPIN = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 25 for PWM with 50Hz
p.start(2.5) # Initialization
try:
  while True:
    p.ChangeDutyCycle(7)
    print("this is 7")
    time.sleep(5)
    p.ChangeDutyCycle(12.5)
    print("this is hold")
    time.sleep(5)
    p.ChangeDutyCycle(2.5)
    print("this is drop")
    time.sleep(5)
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()