import cv2
import quirc
import time
import csv
from ordered_set import OrderedSet
import RPi.GPIO as GPIO
import threading

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
    GPIO.output(lin1, GPIO.LOW)
    GPIO.output(lin2, GPIO.LOW)
    GPIO.output(lin3, GPIO.LOW)
    GPIO.output(lin4, GPIO.LOW)
    GPIO.cleanup()

pin_reset()

## MOTOR FORWARD
def motor_on(rotations):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin1, GPIO.OUT)
    GPIO.setup(pin2, GPIO.OUT)
    GPIO.setup(pin3, GPIO.OUT)
    GPIO.setup(pin4, GPIO.OUT)
    motor_pins = [pin1, pin2, pin3, pin4]
    step_count = 512
    direction = False
    step_sleep = 0.005
    step_sequence = [
        [1, 0, 0, 1],
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1]
    ]
    total_step = step_count * rotations
    step_counter = 0

    try:
        for _ in range(total_step):
            for pin in range(len(motor_pins)):
                GPIO.output(motor_pins[pin], step_sequence[step_counter][pin])
            step_counter = (step_counter + 1) % 8 if not direction else (step_counter - 1) % 8
            time.sleep(step_sleep)
    except KeyboardInterrupt:
        pin_reset()
        exit(1)
    pin_reset()

## LINEAR FORWARD   
def lin_forward():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(lin1, GPIO.OUT)
    GPIO.setup(lin2, GPIO.OUT)
    GPIO.setup(lin3, GPIO.OUT)
    GPIO.setup(lin4, GPIO.OUT)
    lin_count = 500
    step_sleep = 0.002

    try:
        for i in range(lin_count):
            if i % 4 == 0:
                GPIO.output(lin4, GPIO.HIGH)
                GPIO.output(lin3, GPIO.LOW)
                GPIO.output(lin2, GPIO.LOW)
                GPIO.output(lin1, GPIO.LOW)
            elif i % 4 == 1:
                GPIO.output(lin4, GPIO.LOW)
                GPIO.output(lin3, GPIO.LOW)
                GPIO.output(lin2, GPIO.HIGH)
                GPIO.output(lin1, GPIO.LOW)
            elif i % 4 == 2:
                GPIO.output(lin4, GPIO.LOW)
                GPIO.output(lin3, GPIO.HIGH)
                GPIO.output(lin2, GPIO.LOW)
                GPIO.output(lin1, GPIO.LOW)
            elif i % 4 == 3:
                GPIO.output(lin4, GPIO.LOW)
                GPIO.output(lin3, GPIO.LOW)
                GPIO.output(lin2, GPIO.LOW)
                GPIO.output(lin1, GPIO.HIGH)
            time.sleep(step_sleep)
    except KeyboardInterrupt:
        pin_reset()
        exit(1)
    pin_reset()

# linear reverse function
def lin_reverse():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(lin1, GPIO.OUT)
    GPIO.setup(lin2, GPIO.OUT)
    GPIO.setup(lin3, GPIO.OUT)
    GPIO.setup(lin4, GPIO.OUT)
    lin_count = 500
    step_sleep = 0.002

    try:
        for i in range(lin_count):
            if i % 4 == 0:
                GPIO.output(lin4, GPIO.LOW)
                GPIO.output(lin3, GPIO.LOW)
                GPIO.output(lin2, GPIO.LOW)
                GPIO.output(lin1, GPIO.HIGH)
            elif i % 4 == 1:
                GPIO.output(lin4, GPIO.LOW)
                GPIO.output(lin3, GPIO.HIGH)
                GPIO.output(lin2, GPIO.LOW)
                GPIO.output(lin1, GPIO.LOW)
            elif i % 4 == 2:
                GPIO.output(lin4, GPIO.LOW)
                GPIO.output(lin3, GPIO.LOW)
                GPIO.output(lin2, GPIO.HIGH)
                GPIO.output(lin1, GPIO.LOW)
            elif i % 4 == 3:
                GPIO.output(lin4, GPIO.HIGH)
                GPIO.output(lin3, GPIO.LOW)
                GPIO.output(lin2, GPIO.LOW)
                GPIO.output(lin1, GPIO.LOW)
            time.sleep(step_sleep)
    except KeyboardInterrupt:
        pin_reset()
        exit(1)
    pin_reset()

# Camera processing function
def process_camera(cap, codeSet, camera_name):
    while True:
        ret, image = cap.read()
        if not ret:
            continue
        cv2.imshow(camera_name, image)
        if image.any():
            cv2.imwrite('testimage.jpg', image)
            img = cv2.imread('testimage.jpg', cv2.IMREAD_GRAYSCALE)
            ret, img2 = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            if quirc.decode(img):
                decodedData = quirc.decode(img)
                if not decodedData:
                    print("No code :(")
                else:
                    decodedText = decodedData[0][1].payload.decode('utf-8')
                    print(decodedText)
                    if isinstance(decodedText, str):
                        if len(decodedText) == 5 and decodedText[0].isalpha() and decodedText[1:4].isdigit():
                            if decodedText not in open('sampledata.csv').read() and decodedText not in codeSet:
                                codeSet.add(decodedText)
                            print("Good code!")
                            lin_forward()
                            time.sleep(2)
                            motor_on(2)
                            lin_reverse()
                        else:
                            print("Bad code")
                            motor_on(2)
                    else:
                        print("No code")
            else:
                print("No code :(")
                time.sleep(0.01)

        if cv2.waitKey(1) == ord("q"):
            break

# TESTING DUAL CAMERA 
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
cap1 = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2) 
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

codeSet = OrderedSet()
open('sampledata.csv', 'w').truncate()

t1 = threading.Thread(target=process_camera, args=(cap, codeSet, "code detector top"))
t2 = threading.Thread(target=process_camera, args=(cap1, codeSet, "code detector bottom"))

print(t1)

t1.start()
t2.start()


t1.join()
t2.join()

cap.release()
cap1.release()
cv2.destroyAllWindows()

# write data to csv
with open('sampledata.csv', 'a+', newline='') as file:
    csv.writer(file).writerows([[item] for item in list(codeSet)])
