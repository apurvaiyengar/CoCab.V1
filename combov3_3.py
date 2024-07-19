import cv2
import quirc
import time
import csv
from csv import writer
from ordered_set import OrderedSet
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import stop_all
GPIO.setmode(GPIO.BCM)
## BOARD SET UP
# assign pins
pin1 = 5
pin2 = 6
pin3 = 13
pin4 = 19
pin5 = 26
pin6 = 4
lin1 = 18
lin2 = 23
lin3 = 24
lin4 = 25

# reset pins
def pin_reset():
    pin1 = 5
    pin2 = 6
    pin3 = 13
    pin4 = 19
    pin5 = 26
    pin6 = 4
    pin7 = 17

    lin1 = 18
    lin2 = 23
    lin3 = 24
    lin4 = 25

    GPIO.setmode(GPIO.BCM)

    GPIO.setup(pin1, GPIO.OUT)
    GPIO.setup(pin2, GPIO.OUT)
    GPIO.setup(pin3, GPIO.OUT)
    GPIO.setup(pin4, GPIO.OUT)
    GPIO.setup(pin5, GPIO.OUT)
    GPIO.setup(pin6, GPIO.OUT)
    GPIO.setup(pin7, GPIO.OUT)

    GPIO.setup(lin1, GPIO.OUT)
    GPIO.setup(lin2, GPIO.OUT)
    GPIO.setup(lin3, GPIO.OUT)
    GPIO.setup(lin4, GPIO.OUT)

    GPIO.output(pin1, GPIO.LOW)
    GPIO.output(pin2, GPIO.LOW)
    GPIO.output(pin3, GPIO.LOW)
    GPIO.output(pin4, GPIO.LOW)
    GPIO.output(pin5, GPIO.LOW)
    GPIO.output(pin6, GPIO.LOW)
    GPIO.output(pin7, GPIO.LOW)

    GPIO.output(lin1, GPIO.LOW )
    GPIO.output(lin2, GPIO.LOW )
    GPIO.output(lin3, GPIO.LOW )
    GPIO.output(lin4, GPIO.LOW )
    GPIO.cleanup()
    
pin_reset()   

## MOTOR SET UP

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.LOW)
GPIO.setup(4, GPIO.OUT)
GPIO.output(4, GPIO.LOW)


GPIO.setup(5, GPIO.OUT)
GPIO.output(5, GPIO.HIGH)
GPIO.setup(6, GPIO.OUT)
GPIO.output(6, GPIO.HIGH)
GPIO.setup(13, GPIO.OUT)
GPIO.output(13, GPIO.HIGH)

GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.HIGH)
GPIO.setup(4, GPIO.OUT)
GPIO.output(4, GPIO.HIGH) 

GPIO_pins = (pin1, pin2, pin3) 
direction= pin5       
step = pin4
         
stepper = RpiMotorLib.A4988Nema(direction, step, GPIO_pins, "DRV8825")
time.sleep(1)

## LINEAR FORWARD   
def lin_forward():
    # setting up variables
    lin1 = 18
    lin2 = 23
    lin3 = 24
    lin4 = 25
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(lin1, GPIO.OUT)
    GPIO.setup(lin2, GPIO.OUT)
    GPIO.setup(lin3, GPIO.OUT)
    GPIO.setup(lin4, GPIO.OUT)
    lin_count = 500
    step_sleep = 0.002

    # forward
    try:
        i = 0
        for i in range(lin_count):
            if i%4==0:
                GPIO.output( lin4, GPIO.HIGH )
                GPIO.output( lin3, GPIO.LOW )
                GPIO.output( lin2, GPIO.LOW )
                GPIO.output( lin1, GPIO.LOW )
            elif i%4==1:
                GPIO.output( lin4, GPIO.LOW )
                GPIO.output( lin3, GPIO.LOW )
                GPIO.output( lin2, GPIO.HIGH )
                GPIO.output( lin1, GPIO.LOW )
            elif i%4==2:
                GPIO.output( lin4, GPIO.LOW )
                GPIO.output( lin3, GPIO.HIGH )
                GPIO.output( lin2, GPIO.LOW )
                GPIO.output( lin1, GPIO.LOW )
            elif i%4==3:
                GPIO.output( lin4, GPIO.LOW )
                GPIO.output( lin3, GPIO.LOW )
                GPIO.output( lin2, GPIO.LOW )
                GPIO.output( lin1, GPIO.HIGH )
     
            time.sleep( step_sleep )
     
    except KeyboardInterrupt:
        pin_reset()
        exit(1)
    pin_reset()
    

# linear reverse function
def lin_reverse():
    # setting up variables
    lin1 = 18
    lin2 = 23
    lin3 = 24
    lin4 = 25
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(lin1, GPIO.OUT)
    GPIO.setup(lin2, GPIO.OUT)
    GPIO.setup(lin3, GPIO.OUT)
    GPIO.setup(lin4, GPIO.OUT)
    lin_count = 500
    step_sleep = 0.002

    # forward
    try:
        i = 0
        for i in range(lin_count):
            if i%4==0:
                GPIO.output( lin4, GPIO.LOW )
                GPIO.output( lin3, GPIO.LOW )
                GPIO.output( lin2, GPIO.LOW )
                GPIO.output( lin1, GPIO.HIGH )
            elif i%4==1:
                GPIO.output( lin4, GPIO.LOW )
                GPIO.output( lin3, GPIO.HIGH )
                GPIO.output( lin2, GPIO.LOW )
                GPIO.output( lin1, GPIO.LOW )
            elif i%4==2:
                GPIO.output( lin4, GPIO.LOW )
                GPIO.output( lin3, GPIO.LOW )
                GPIO.output( lin2, GPIO.HIGH )
                GPIO.output( lin1, GPIO.LOW )
            elif i%4==3:
                GPIO.output( lin4, GPIO.HIGH )
                GPIO.output( lin3, GPIO.LOW )
                GPIO.output( lin2, GPIO.LOW )
                GPIO.output( lin1, GPIO.LOW )
     
            time.sleep( step_sleep )
     
    except KeyboardInterrupt:
        pin_reset()
        exit(1)
    pin_reset()

# TESTING DUAL CAMERA 
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap1 = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2) 

## CAMERA SET UP
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

assert cap.isOpened()
assert cap1.isOpened()

# create an empty set for codes
codeSet = OrderedSet()

# for TESTING ONLY, clear csv of contents
open('sampledata.csv', 'w').truncate()

step_counter = 0

stepper.motor_go(False, "1/8", 5, .01, False, .05)
# begin camera feed
while True:
    
    # load the image
    ret, image = cap.read()
    ret1, image1 = cap1.read()
    
    # show the preview
    cv2.imshow("code detector top", image)
    cv2.imshow('code detector bottom', image1)
    # move the motor 
    stepper.motor_go(False, "1/8", 5, .01, False, .05)

    if image.any():
        # take a picture
        cv2.imwrite('testimage.jpg', image)
        # prep the image for decoding
        img = cv2.imread('testimage.jpg', cv2.IMREAD_GRAYSCALE)
        ret, img2 = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # decode the image
        qrCodeDetector = cv2.QRCodeDetector()
        # if the image is decoded correctly, print the text
        if quirc.decode(img):
            decodedData = quirc.decode(img)
            if not decodedData:
                print("No code :((")
            decodedText = decodedData[0][1].payload.decode('utf-8')
            print(decodedText)
            # check if code is the correct format & has already been scanned
            if isinstance(decodedText, str) == True:
                if len(decodedText) == 5 and decodedText[0].isalpha() == True and decodedText[1:4].isdigit() == True:
                    if decodedText not in open('sampledata.csv').read() and decodedText not in codeSet:
                        codeSet.add(decodedText)
                    print("Good code!")
                    lin_forward()
                    time.sleep(1)
                    stepper.motor_go(False, "1/8", 5, .01, False, .05)
                    lin_reverse()
                    
                else:
                    print("Bad code")
                    stepper.motor_go(False, "1/8", 5, .01, False, .05)
            else:
                print("No code")
        # if not, try again after 0.002 sec
        if not quirc.decode(img):
            print("No code :(")
            time.sleep(0.002)

    if image1.any():
        # take a picture
        
        cv2.imwrite('testimage.jpg', image1)
        # prep the image for decoding
        img1 = cv2.imread('testimage.jpg', cv2.IMREAD_GRAYSCALE)
        ret, img3 = cv2.threshold(img1, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        # decode the image
        qrCodeDetector = cv2.QRCodeDetector()
        # if the image is decoded correctly, print the text
        if quirc.decode(img1):
            decodedData1 = quirc.decode(img1)
            if not decodedData1:
                print("No code :((")
            decodedText1 = decodedData1[0][1].payload.decode('utf-8')
            print(decodedText1)
            # check if code is the correct format & has already been scanned
            if isinstance(decodedText1, str) == True:
                if len(decodedText1) == 5 and decodedText1[0].isalpha() == True and decodedText1[1:4].isdigit() == True:
                    if decodedText1 not in open('sampledata.csv').read() and decodedText1 not in codeSet:
                        codeSet.add(decodedText1)
                    print("Good code!")
                    lin_forward()
                    time.sleep(1)
                    stepper.motor_go(False, "1/8", 5, .01, False, .05)
                    lin_reverse()
                    
                else:
                    print("Bad code")
                    stepper.motor_go(False, "1/8", 5, .01, False, .05)
            else:
                print("No code")
        # if not, try again after 0.01 sec
        if not quirc.decode(img1):
            print("No code :(")
            time.sleep(0.002)
           
    if(cv2.waitKey(1) == ord("q")):
        break
# release camera and close all windows
cap.release()
cap1.release()
cv2.destroyAllWindows()

# convert set to a list
codeList = list(codeSet)
print(codeList)

# write data to csv
file = open('sampledata.csv', 'a+', newline = '')
with file:
    csv.writer(file).writerows([[item]for item in codeList])
