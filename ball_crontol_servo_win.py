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
servo1_angle_limit_positive = 90
servo1_angle_limit_negative = -90
servo1_offset = 10

servo2_angle_limit_positive = 90
servo2_angle_limit_negative = -90
servo2_offset = 2

servo3_angle_limit_positive = 90
servo3_angle_limit_negative = -90
servo3_offset = 1

#center = (620, 370)
center_point = [620, 370, 2210]

L = 16
# H = 10
d = 3

def init_pos_matrix():
    return np.array([
        [L / 2, -L / 2, 0],
        [L / (2 * np.sqrt(3)) + d, L / (2 * np.sqrt(3)) + d, -L / (2 * np.sqrt(3)) + d],
        [0, 0, 0]
    ])
def Z_rotation_matrix(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])

def X_rotation_matrix(angle):
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])


def Y_rotation_matrix(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])


def ball_track(key1, queue):
    camera_port = 1
    cap = cv2.VideoCapture(camera_port,cv2.CAP_DSHOW)
    cap.set(3, 1280)
    cap.set(4, 720)

    get, img = cap.read()
    if get:
        h, w, _ = img.shape
    else:
        print("Failed to capture image")

    #crosshair = cv2.circle(img, (645, 355), 20, (0, 0, 255), 2)
    #circle_image = cv2.circle(img, center, 280, (0, 0, 255), 2) # is now in while loop

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    #hsvVals = {'hmin': 255, 'smin': 255, 'vmin': 255, 'hmax': 179, 'smax': 255, 'vmax': 255}
    hsvVals = {'hmin': 0, 'smin': 140, 'vmin': 90, 'hmax': 10, 'smax': 255, 'vmax': 255}

    # Calibrated
    #hsvVals = {'hmin': 0, 'smin': 125, 'vmin': 220, 'hmax': 15, 'smax': 255, 'vmax': 255}

    #center_point = [center[0], center[1], 2210]

    while True:
        get, img = cap.read()
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask)

        #circle_image = cv2.circle(img, center, 300, (0, 0, 255), 2)

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
            data = 'nil'# returns nil if we cant find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)
        #cv2.imshow("Image", circle_image)
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

        pitch_pid = PID(Kp=1, Ki=0.1, Kd=0.05, setpoint=0)
        roll_pid = PID(Kp=1, Ki=0.1, Kd=0.05, setpoint=0)
        heave_pid = PID(Kp=1, Ki=0.1, Kd=0.05, setpoint=0)

        """
        Alt over her er forsøk på implemetansjon av kontroller, og er sikkert
        grunnen til alle feilmeldingene som kommer
        """
        corrd_info = queue.get()

        if corrd_info == 'nil': # Checks if the output is nil
            print('cant find the ball :(')
        else:
            print('The position of the ball : ', corrd_info[2])

            #if (-90 < corrd_info[0] < 90) and (-90 < corrd_info[1] < 90) and (-90 < corrd_info[2] < 90):

            #    all_angle_assign(corrd_info[0],corrd_info[1],corrd_info[2])
            #else:
            #    all_angle_assign(0,0,0)

        print('::: ---------- :', corrd_info[1])
        errors = [0, 0, 0]
        for i in errors:
            if corrd_info != 'i':
                errors[i] = center_point[i] - int(corrd_info[i])

        roll_error = errors[0]
        pitch_error = errors[1]
        heave_error = errors[2]

        #pitch_error = center_point[1] - int(corrd_info[1])
        #roll_error = center_point[0] - int(round(float(corrd_info[0])))
        #heave_error = center_point[2] - int(round(float(corrd_info[2])))

        pitch_output = pitch_pid(pitch_error)
        roll_output = roll_pid(roll_error)
        heave_output = heave_pid(heave_error)

        R = Z_rotation_matrix(0) @ X_rotation_matrix(pitch_output) @ Y_rotation_matrix(roll_output)

        pos_matrix = init_pos_matrix()
        new_pos_matrix = pos_matrix @ R.T
        new_pos_matrix[:, 2] += heave_output

        servo1_angle, servo2_angle, servo3_angle = inverse_kinematics(new_pos_matrix)

        all_angle_assign(servo1_angle, servo2_angle, servo3_angle)

    def inverse_kinematics_single_corner(target_pos):
        l1 = 4
        l2 = 12
        x, y, z = target_pos
        r = np.sqrt(x ** 2 + y ** 2)
        d = np.sqrt((r - d) ** 2 + z ** 2)

        # Calculate the angles using the inverse kinematics equations
        angle1 = np.arctan2(y, x)  # Servo angle for the horizontal rotation
        angle2 = np.arctan2(z, r - d) - np.arccos(
            (l1 ** 2 + d ** 2 - l2 ** 2) / (2 * l1 * d))  # Servo angle for the first arm section
        angle3 = np.pi - np.arccos(
            (l1 ** 2 + l2 ** 2 - d ** 2) / (2 * l1 * l2))  # Servo angle for the second arm section

        return angle1, angle2, angle3


    def write_arduino(data):
        print('The angles send to the arduino : ', data)

        arduino.write(bytes(data, 'utf-8'))

    def write_servo():

        testingServo = False

        if testingServo:
            test_1 = 30
            test_2 = 30
            test_3 = 30

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

        # P = 0.3
        # I = 0.002
        # D = 0.29
        #
        # pidx = PID(P, I, D, setpoint=0)
        # pidx.output_limits = (-15, 15)
        # pidy = PID(P, I, D, setpoint=0)
        # pidy.output_limits = (-15, 15)
        #
        # counter = 0
        #
        # #while True

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