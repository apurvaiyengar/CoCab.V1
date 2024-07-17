from multiprocessing.pool import ThreadPool
import cv2
import quirc
import time
import csv
from csv import writer
from ordered_set import OrderedSet
from RpiMotorLib import RpiMotorLib

def process_frame(image):
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
                    
                else:
                    print("Bad code")
            else:
                print("No code")
        # if not, try again after 0.002 sec
        if not quirc.decode(img):
            print("No code :(")
            #time.sleep(0.002)
pin1 = 5
pin2 = 6
pin3 = 13
pin4 = 19
pins = [pin1, pin2, pin3, pin4]
stepper = RpiMotorLib.BYJMotor("MyMotorOne", "Nema")
time.sleep(0.5)

def motor_on(rotations):
    lin_count =50
    total_count = lin_count * rotations
    i = 0
    while i in range(total_count):
        stepper.motor_run(pins , 0.1, lin_count, False, False, "half", .05)
    

# create an empty set for codes
codeSet = OrderedSet()

# for TESTING ONLY, clear csv of contents
open('sampledata.csv', 'w').truncate()

if __name__ == '__main__':
    # Setup.
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    thread_num = cv2.getNumberOfCPUs()
    pool = ThreadPool(processes=thread_num)
    pending_task = motor_on(1)

    while True:
        # Consume the queue.
        while len(pending_task) > 0 and pending_task[0].ready():
            res = pending_task.popleft().get()
            cv2.imshow('threaded video', res)

        # Populate the queue.
        if len(pending_task) < thread_num:
            ret, image = cap.read()
            cv2.imshow("code detector top", image)
            if ret:
                task = pool.apply_async(process_frame, (image.copy(),))
                pending_task.append(task)

        # Show preview.
        if cv2.waitKey(1) == 27 or not ret:
            break
cap.release()
cv2.destroyAllWindows()

codeList = list(codeSet)
print(codeList)

# write data to csv
file = open('sampledata.csv', 'a+', newline = '')
with file:
    csv.writer(file).writerows([[item]for item in codeList])