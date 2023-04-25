import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
from tkinter import *
from simple_pid import PID
import numpy as np

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""

# define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = 70
servo1_angle_limit_negative = -40

servo2_angle_limit_positive = 70
servo2_angle_limit_negative = -40

servo3_angle_limit_positive = 70
servo3_angle_limit_negative = -40


def ball_track(key1, queue):
    camera_port = 0
    cap = cv2.VideoCapture(camera_port,cv2.CAP_DSHOW)
    cap.set(3, 1280)
    cap.set(4, 720)

    get, img = cap.read()
    if get:
        h, w, _ = img.shape
    else:
        print("Failed to capture image")
    center = (720, 350)
    #crosshair = cv2.circle(img, (645, 355), 20, (0, 0, 255), 2)
    #circle_image = cv2.circle(img, center, 280, (0, 0, 255), 2) # is now in while loop


    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    #hsvVals = {'hmin': 255, 'smin': 255, 'vmin': 255, 'hmax': 179, 'smax': 255, 'vmax': 255}
    hsvVals = {'hmin': 0, 'smin': 130, 'vmin': 50, 'hmax': 9, 'smax': 255, 'vmax': 255}

    #center_point = [626, 337, 2210] # center point of the plate, calibrated
    center_point = [720, 350, 2210]

    while True:
        get, img = cap.read()
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask)

        circle_image = cv2.circle(img, center, 280, (0, 0, 255), 2)

        Point = center_point

        if countours:

            data = round((countours[0]['center'][0] - center_point[0]) / 10), \
                   round((h - countours[0]['center'][1] - center_point[1]) / 10), \
                   round(int(countours[0]['area'] - center_point[2])/100)

            # position x and y of ball
            p_x = round(countours[0]['center'][0])
            p_y = round(h - countours[0]['center'][1])

            # position x and y of ball compared to center of platform
            print('ball coordinates: ', p_x - center_point[0], p_y - center_point[1])

            queue.put(data)
            #print("The got coordinates for the ball are :", data)
        else:
            data = 'nil' # returns nil if we cant find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)
        cv2.imshow("Image", circle_image)
        cv2.waitKey(1)

def is_ball_in_circle(point, center, radius):
    distance = np.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2)
    return distance <= radius

def servo_control(key2, queue):
    port_id = 'COM6'
    # initialise serial interface
    arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)
    if key2:
        print('Servo controls are initiated')


 #while True:
    #posistion = queue.get()
    #v = list(position)
    #pixel_vector = np.array([v[0]], [v[1]]])

    #zr = [[cos(theta), -sin(theta)],
            #[sin/theta), cos(theta)]]

    #roll = pidx(new_pixel[0])
    #pitch = pidy(new_pixel[1])




    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = -math.radians(float(angle_passed1))
        servo2_angle = math.radians(float(angle_passed2))
        servo3_angle = math.radians(float(angle_passed3))
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    def writeCoord():
        """
        Here in this function we get both coordinate and servo control,
        it is an ideal place to implement the controller
        """
        corrd_info = queue.get()

        if corrd_info == 'nil': # Checks if the output is nil
            print('cant find the ball :(')
        else:
            print('The position of the ball : ', corrd_info[2])

            if (-90 < corrd_info[0] < 90) and (-90 < corrd_info[1] < 90) and (-90 < corrd_info[2] < 90):

                all_angle_assign(corrd_info[0],corrd_info[1],corrd_info[2])
            else:
                all_angle_assign(0,0,0)

    def write_arduino(data):
        print('The angles send to the arduino : ', data)

        arduino.write(bytes(data, 'utf-8'))

    def write_servo():

        testingServo = False

        if testingServo:
            test_1 = 70
            test_2 = 70
            test_3 = -40

            ang1 = math.radians(test_1)
            ang2 = math.radians(test_2)
            ang3 = math.radians(test_3)
        else:
            ang1 = servo1_angle
            ang2 = servo2_angle
            ang3 = servo3_angle

        angles: tuple = (round(math.degrees(ang1), 1),
                         round(math.degrees(ang2), 1),
                         round(math.degrees(ang3), 1))

        print(str(angles))
        write_arduino(str(angles))

    while key2:
        writeCoord()

    root.mainloop()  # running loop

if __name__ == '__main__':

    queue = Queue() # The queue is done inorder for the communication between the two processes.
    key1 = 1 # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target= ball_track, args=(key1, queue)) # initiate ball tracking process
    p2 = mp.Process(target=servo_control,args=(key2, queue)) # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()