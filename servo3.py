import RPi.GPIO as GPIO
import time

servoPIN = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 25 for PWM with 50Hz
p.start(2.5) # Initialization
try:
  while True:
    p.ChangeDutyCycle(5)
    print("this is 5")
    time.sleep(5)
    p.ChangeDutyCycle(7.5)
    print("this is 7.5")
    time.sleep(5)
    p.ChangeDutyCycle(10)
    print("this is 10")
    time.sleep(5)
    p.ChangeDutyCycle(12.5)
    print("this is 12.5")
    time.sleep(5)
    p.ChangeDutyCycle(10)
    print("this is 10")
    time.sleep(5)
    p.ChangeDutyCycle(7.5)
    print("this is 7.5")
    time.sleep(5)
    p.ChangeDutyCycle(5)
    print("this is 5")
    time.sleep(5)
    p.ChangeDutyCycle(2.5)
    print("this is 2.5")
    time.sleep(5)
except KeyboardInterrupt:
  p.stop()
  GPIO.cleanup()