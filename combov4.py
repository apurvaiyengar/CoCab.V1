import cv2
import quirc
import time
import csv
from csv import writer
from ordered_set import OrderedSet
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
from threading import Thread

## BOARD SET UP
# assign pins
pin1 = 5
pin2 = 6
pin3 = 13
pin4 = 19
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
    lin1 = 18
    lin2 = 23
    lin3 = 24
    lin4 = 25
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin1, GPIO.OUT)
    GPIO.setup(pin2, GPIO.OUT)
    GPIO.setup(pin3, GPIO.OUT)
    GPIO.setup(pin4, GPIO.OUT)
    GPIO.setup(lin1, GPIO.OUT)
    GPIO.setup(lin2, GPIO.OUT)
    GPIO.setup(lin3, GPIO.OUT)
    GPIO.setup(lin4, GPIO.OUT)
    GPIO.output(pin1, GPIO.LOW)
    GPIO.output(pin2, GPIO.LOW)
    GPIO.output(pin3, GPIO.LOW)
    GPIO.output(pin4, GPIO.LOW)
    GPIO.output(lin1, GPIO.LOW )
    GPIO.output(lin2, GPIO.LOW )
    GPIO.output(lin3, GPIO.LOW )
    GPIO.output(lin4, GPIO.LOW )
    GPIO.cleanup()
    
pin_reset()   

## MOTOR FORWARD

pins = [pin1, pin2, pin3, pin4]
stepper = RpiMotorLib.BYJMotor("MyMotorOne", "Nema")
time.sleep(0.5)

def motor_on(rotations):
    lin_count =50
    total_count = lin_count * rotations
    i = 0
    while i in range(total_count):
        stepper.motor_run(pins , 0.1, lin_count, False, False, "half", .05)

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

####################################
# opening video capture stream
# defining a helper class for implementing multi-threaded processing 
# create an empty set for codes
codeSet = OrderedSet()

# for TESTING ONLY, clear csv of contents
open('sampledata.csv', 'w').truncate()
class WebcamStream :
    def __init__(self):
        
        # opening video capture stream 
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.isOpened()
        fps_input_stream = int(self.cap.get(5))
            
        # reading a single frame from vcap stream for initializing 
        self.grabbed , self.frame = self.cap.read()
        if self.grabbed is False :
            print('[Exiting] No more frames to read')
            exit(0)
# self.stopped is set to False when frames are being read from self.vcap stream 
        self.stopped = True
# reference to the thread for reading next available frame from input stream 
        self.t = Thread(target=self.update, args=())
        self.t.daemon = True # daemon threads keep running in the background while the program is executing 
        
    # method for starting the thread for grabbing next available frame in input stream 
    def start(self):
        self.stopped = False
        self.t.start()
# method for reading next frame 
    def update(self):
        while True :
            if self.stopped is True :
                break
            self.grabbed , self.frame = self.vcap.read()
            if self.grabbed is False :
                print('[Exiting] No more frames to read')
                self.stopped = True
                break 
        self.vcap.release()
# method for returning latest read frame 
    def read(self):
        return self.frame
# method called to stop reading frames 
    def stop(self):
        self.stopped = True
# initializing and starting multi-threaded webcam capture input stream 
webcam_stream = WebcamStream() #  stream_id = 0 is for primary camera 
webcam_stream.start()
# processing frames in input stream
num_frames_processed = 0 
start = time.time()
while True :
    if webcam_stream.stopped is True :
        break
    else :
        frame = webcam_stream.read()
    cv2.imshow('frame' , frame)
    motor_on(1)
    if frame.any():
        # take a picture
        cv2.imwrite('test_image.jpg' , frame)
        # prep the image for decoding
        img = cv2.imread('test_image.jpg', cv2.IMREAD_GRAYSCALE)
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
                    motor_on(1)
                    lin_reverse()
                    #motor_on(96, True)
                else:
                    print("Bad code")
                    motor_on(1)
            else:
                print("No code")
        # if not, try again after 0.002 sec
        if not quirc.decode(img):
            print("No code :(")
            #time.sleep(0.002)
    if(cv2.waitKey(1) == ord("q")):
        break
end = time.time()
webcam_stream.stop() # stop the webcam stream

# closing all windows 

cv2.destroyAllWindows()

#########################

# convert set to a list
codeList = list(codeSet)
print(codeList)

# write data to csv
file = open('sampledata.csv', 'a+', newline = '')
with file:
    csv.writer(file).writerows([[item]for item in codeList])
