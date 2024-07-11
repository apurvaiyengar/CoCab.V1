import cv2
import quirc
import time
import csv
from csv import writer
from ordered_set import OrderedSet
import RPi.GPIO as GPIO
import threading

## BOARD SET UP
# Assign pins
pin1 = 5
pin2 = 6
pin3 = 13
pin4 = 19
lin1 = 18
lin2 = 23
lin3 = 24
lin4 = 25

# Reset pins
def pin_reset():
    global pin1, pin2, pin3, pin4, lin1, lin2, lin3, lin4
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
    GPIO.output(lin1, GPIO.LOW)
    GPIO.output(lin2, GPIO.LOW)
    GPIO.output(lin3, GPIO.LOW)
    GPIO.output(lin4, GPIO.LOW)

## MOTOR FUNCTIONS
def motor_on(rotations):
    global pin1, pin2, pin3, pin4
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin1, GPIO.OUT)
    GPIO.setup(pin2, GPIO.OUT)
    GPIO.setup(pin3, GPIO.OUT)
    GPIO.setup(pin4, GPIO.OUT)
    motor_pins = [pin1, pin2, pin3, pin4]
    step_count = 512
    direction = False
    step_sleep = 0.005
    step_sequence = [[1, 0, 0, 1],
                     [1, 0, 0, 0],
                     [1, 1, 0, 0],
                     [0, 1, 0, 0],
                     [0, 1, 1, 0],
                     [0, 0, 1, 0],
                     [0, 0, 1, 1],
                     [0, 0, 0, 1]]
    total_step = step_count * rotations
    step_counter = 0

    try:
        for _ in range(total_step):
            for pin in range(len(motor_pins)):
                GPIO.output(motor_pins[pin], step_sequence[step_counter][pin])
            if direction:
                step_counter = (step_counter - 1) % 8
            else:
                step_counter = (step_counter + 1) % 8
            time.sleep(step_sleep)
    except KeyboardInterrupt:
        pin_reset()
        exit(1)
    pin_reset()

def lin_forward():
    global lin1, lin2, lin3, lin4
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

def lin_reverse():
    global lin1, lin2, lin3, lin4
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

## CAMERA AND QR CODE PROCESSING FUNCTIONS
def camera_thread(cap, name, codeSet):
    while True:
        ret, image = cap.read()
        if ret:
            # Process image for QR code detection
            img_path = f'testimage_{name}.jpg'
            cv2.imwrite(img_path, image)
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            ret, img2 = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            qrCodeDetector = cv2.QRCodeDetector()
            decodedData = qrCodeDetector.detectAndDecode(img2)
            if len(decodedData) > 0:
                decodedText = decodedData[0]
                print(f"QR Code detected from {name}: {decodedText}")
                # Process the decoded text (example code from your original logic)
                if isinstance(decodedText, str) and len(decodedText) == 5 and decodedText[0].isalpha() and decodedText[1:4].isdigit():
                    if decodedText not in open('sampledata.csv').read() and decodedText not in codeSet:
                        codeSet.add(decodedText)
                        print("Good code!")
                        lin_forward()
                        time.sleep(2)
                        motor_on(2)
                        lin_reverse()
                    else:
                        print("Code already scanned or incorrect format.")
                        motor_on(2)
                else:
                    print("Invalid QR Code format.")
                    motor_on(2)
            else:
                print(f"No QR Code detected from {name}.")
        else:
            print(f"Error reading from {name}")
        time.sleep(0.01)  # Adjust sleep time as needed

def start_camera_threads(codeSet):
    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    cap1 = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2)

    thread1 = threading.Thread(target=camera_thread, args=(cap, "camera1", codeSet))
    thread2 = threading.Thread(target=camera_thread, args=(cap1, "camera2", codeSet))

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

    cap.release()
    cap1.release()
    cv2.destroyAllWindows()

## MAIN FUNCTION
def main():
    pin_reset()
    codeSet = OrderedSet()

    motor_thread = threading.Thread(target=motor_on, args=(1,))
    camera_thread = threading.Thread(target=start_camera_threads, args=(codeSet,))

    motor_thread.start()
    camera_thread.start()

    motor_thread.join()
    camera_thread.join()

    # Convert set to a list
    codeList = list(codeSet)
    print(codeList)

    # Write data to CSV
    with open('sampledata.csv', 'a+', newline='') as file:
        csv.writer(file).writerows([[item] for item in codeList])

    pin_reset()

if __name__ == "__main__":
    main()
