import cv2
import quirc
import time
import csv
from csv import writer
from ordered_set import OrderedSet
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib

pin1 = 5
pin2 = 6
pin3 = 13
pin4 = 19
pin5 = 26
pin6 = 4 


# reset pins
def pin_reset():
    pin1 = 5
    pin2 = 6
    pin3 = 13
    pin4 = 19
    pin5 = 26
    pin6 = 4
    pin_reset = 17
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin1, GPIO.OUT)
    GPIO.setup(pin2, GPIO.OUT)
    GPIO.setup(pin3, GPIO.OUT)
    GPIO.setup(pin4, GPIO.OUT)
    GPIO.setup(pin5, GPIO.OUT)
    GPIO.setup(pin6, GPIO.OUT)
    GPIO.setup(pin_reset, GPIO.OUT)
    
    GPIO.output(pin1, GPIO.LOW)
    GPIO.output(pin2, GPIO.LOW)
    GPIO.output(pin3, GPIO.LOW)
    GPIO.output(pin4, GPIO.LOW)
    GPIO.output(pin5, GPIO.LOW)
    GPIO.output(pin6, GPIO.LOW)
    GPIO.output(pin_reset, GPIO.LOW)
    GPIO.cleanup()
    
pin_reset()   
## STEPPER ROTATE
#define GPIO pins

def motor_on(rotations):
    pin1 = 5
    pin2 = 6
    pin3 = 13
    pin4 = 19
    pin5 = 26
    pin6 = 4 
    pin_reset = 17
    GPIO_pins = (pin1, pin2, pin3) 
    direction= pin5       
    step = pin4
         
    stepper = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "DRV8825")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_reset, GPIO.OUT)
    GPIO.output(pin_reset, GPIO.HIGH) 
    GPIO.setup(pin6, GPIO.OUT)
    GPIO.output(pin6, GPIO.LOW)
    lin_count = 100
    total_count = lin_count * rotations
    i = 0
    try:
        while i in range(total_count):
            GPIO.output(pin6, GPIO.HIGH)
            stepper.motor_go(False, "1/8" , lin_count, .02, False, .05)
            GPIO.output(pin6, GPIO.LOW)
    except KeyboardInterrupt:
        pin_reset()
        exit(1)
    pin_reset()
motor_on(1)
