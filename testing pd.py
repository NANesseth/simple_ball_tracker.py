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
import time

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""

# Global variables
previous_pos = None
timestamp = None
prev_sum_roll = 0.0
prev_sum_pitch = 0.0

# define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = 15
servo1_angle_limit_negative = -20
# servo1_offset = -11

servo2_angle_limit_positive = 15
servo2_angle_limit_negative = -20
# servo2_offset = 0  # -8

servo3_angle_limit_positive = 15
servo3_angle_limit_negative = -20
# servo3_offset = 5  # -7

center_point = [635, 380, 2210]  # Must be re-calibrated each time the servo screws are tightened
# center_point = [640, 350, 2210]
center = (center_point[0], center_point[1])
plat_r = 177

L = 200  # np.sqrt(3) * 160
ARM_LENGTH = 40
N_SAMPLES = 5

# Set this to true if you want to see a red circle where the platform should be.
# Useful for calibrating the center of the platform
see_circle = False

# Angle for rotating camera image
# Since the platform is mounted the wrong way, the camera is first turned 180 degrees
camera_angle = 180 - 16

# Calibrated PID values
Kp = 0.35
Ki = 0.2
Kd = 0.22


# A timer class that holds the elapsed time since last reset
# and checks if a specified time interval has passed
class Timer:
    def __init__(self):
        self.previous_time = time.time()

    def reset_timer(self):
        self.previous_time = time.time()

    def timer_interval(self, interval):
        current_time = time.time()
        if current_time - self.previous_time >= interval:
            self.previous_time = current_time
            return True
        else:
            return False


def ball_track(key1, queue):
    ball_sample_timer = Timer()  # Initializing the sample timer for the ball position

    camera_port = 1
    cap = cv2.VideoCapture(camera_port, cv2.CAP_DSHOW)
    cap.set(3, 1280)
    cap.set(4, 720)

    get, img = cap.read()
    if get:
        h, w, _ = img.shape
        img_rotated = cv2.getRotationMatrix2D((w / 2, h / 2), camera_angle, 1)  # Rotating the camera image
    else:
        print("Failed to capture image")

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(
        False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 0, 'smin': 140, 'vmin': 90, 'hmax': 10, 'smax': 255, 'vmax': 255}  # Red

    while True:
        get, img = cap.read()
        img = cv2.warpAffine(img, img_rotated, (w, h))
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask)

        if see_circle:
            circle_image = cv2.circle(img, center, 275, (0, 0, 255), 2)

        if countours:
            # Finds the ball position, with sample frequency of 25
            if ball_sample_timer.timer_interval(0.04):
                ball_sample_timer.reset_timer()

                data = round((countours[0]['center'][0] - center_point[0]) / 10), \
                    round((h - countours[0]['center'][1] - center_point[1]) / 10), \
                    round(int(countours[0]['area'] - center_point[2]) / 100)

                # position x and y of ball
                p_x = round(countours[0]['center'][0])
                p_y = round(h - countours[0]['center'][1])

                queue.put(data)


        else:
            data = 'nil'  # returns nil if we cant find the ball
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        cv2.imshow("Image", imgStack)
        if see_circle:
            cv2.imshow("Image", circle_image)
        cv2.waitKey(1)

def p_prz_bottom_matrix(roll, pitch):
    # The bottom parts of the p_prz matrix separated into constants and calculated for z
    c_1 = ((np.sqrt(3) * L) / 6) * np.sin(pitch) * np.cos(roll)
    c_2 = (L / 2) * np.sin(roll)
    c_3 = (-(np.sqrt(3) * L) / 3) * np.sin(pitch) * np.cos(roll)

    z_1 = - c_1 - c_2
    z_2 = - c_1 + c_2
    z_3 = - c_3

    z = (round(z_1, 3), round(z_2, 3), round(z_3, 3))
    return z


def is_ball_in_circle(point, center, radius):
    # Returns true if the ball is on the platform
    radius = radius + 18
    distance = np.sqrt((point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2)
    return distance <= radius


def get_angle(z):
    # This function is the implementation of the equation explained in the report: Va = arcsin(z/r)
    # Length of servo arms: 4cm and 12cm
    r = 4.0
    # To avoid error if the asin() operator is given a number outside the (-1, 1) range
    z = limit(z, -r, r)

    # The equation is divided into smaller operations. This makes troubleshooting easier
    parenthesis = z / r
    angle = math.asin(parenthesis)

    return angle


def map_range(x, in_min, in_max, out_min, out_max):
    # This function performs scaling on the input variable from one value range to a new
    if x < in_min:
        x = in_min
    elif x > in_max:
        x = in_max
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


def limit(val, min, max):
    # This function limits the variable to a defined value range
    if val < min:
        out = min
    elif val > max:
        out = max
    else:
        out = val
    return out


def get_ball_speed(ball_pos):
    global previous_pos, timestamp
    current_time = time.time()
    if previous_pos is None or timestamp is None:
        speed = (0, 0)
    else:
        time_elapsed = current_time - timestamp + 1e-6 # A very small number is added to avoid dividing by zero at the beginning
        # print("Time elapsed:        ", time_elapsed)

        distance_moved_x = ball_pos[0] - previous_pos[0]
        distance_moved_y = ball_pos[1] - previous_pos[1]

        speed_x = round(distance_moved_x / time_elapsed, 1)
        speed_y = round(distance_moved_y / time_elapsed, 1)

        speed = (speed_x, speed_y)
    timestamp = current_time
    previous_pos = ball_pos
    return speed


# Two PID controllers, one for roll and one for pitch
pid_roll = PID(Kp, Ki, Kd, setpoint=0)
pid_pitch = PID(Kp, Ki, Kd, setpoint=0)

# Output limits
pid_roll.output_limits = (-20, 15)
pid_pitch.output_limits = (-20, 15)


def PID_regulator(ball_pos):
    ball_speed = get_ball_speed(ball_pos)

    # Calculate PID outputs
    roll_output = pid_roll(ball_pos[0])
    pitch_output = pid_pitch(ball_pos[1])

    sum_roll = round(math.radians(-roll_output), 3)
    sum_pitch = round(math.radians(-pitch_output), 3)

    print('PID angles: ', pitch_output)
    print('PID angels: ', math.radians(-roll_output))

    angles = (sum_roll, sum_pitch)
    # print('Sum roll & pitch: ', angles)

    return angles

servo_Ts = 0.1  # Sampling time


def digital_filter_servo_angles(input_signal, initial_condition=0):

    output_signal = [initial_condition]  # initial condition for the filter

    for n in range(1, len(input_signal)):
        y_n = output_signal[n - 1] + servo_Ts * input_signal[n]  # Compute the current output
        output_signal.append(y_n)  # Append the output to the output signal

        return output_signal


def servo_control(key2, queue):
    port_id = 'COM6'
    # initialise serial interface
    arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)
    if key2:
        print('Servo controls are initiated')

    root = Tk()
    root.resizable(0, 0)

    samples = [[] for _ in range(3)]  # Create empty lists for each servo angle

    def all_angle_assign(angle_passed1, angle_passed2, angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = math.radians(float(angle_passed1))
        servo2_angle = math.radians(float(angle_passed2))
        servo3_angle = math.radians(float(angle_passed3))

        servo_angles = (servo1_angle, servo2_angle, servo3_angle)
        # write_servo(servo_angles)

        # Here begins the filtering of the servo angles
        new_sample = servo_angles
        filtered_servo_angles = []

        for i, angle in enumerate(new_sample):
            samples[i].append(angle)

            if len(samples[i]) >= N_SAMPLES:
                filtered_servo_angle = angle - servo_Ts * (angle - samples[i][-2])
                filtered_servo_angles.append(filtered_servo_angle)
            else:
                filtered_servo_angles.append(angle)

        # Had an error where the filtered data also needed to be in a tuple instead of an array
        filtered_servo_angles_tuple = (filtered_servo_angles[0], filtered_servo_angles[1], filtered_servo_angles[2])

        servo1_diff = abs(servo_angles[0] - filtered_servo_angles[0])
        servo2_diff = abs(servo_angles[1] - filtered_servo_angles[1])
        servo3_diff = abs(servo_angles[2] - filtered_servo_angles[2])

        servo_filter_differences = (round(servo1_diff, 3), round(servo2_diff, 3), round(servo3_diff, 3))

        print('Difference between filtered and unfiltered servo angles: ', servo_filter_differences)

        use_filtered_servo_angles = False

        if use_filtered_servo_angles:
            write_servo(filtered_servo_angles_tuple)
        else:
            write_servo(servo_angles)


    ball_pos_mm = [0, 0, 0]

    def writeCoord():

        """
         === Dataflow ===
        Finner filtrert posisjon på ball
        bruker PID kontroller til å regulere posisjonen i pitch- og roll-vinkler
        Finner z for hver servo vha P_prz-matrisen
        Gjør om til servovinkler vha V_a = arcsin(z/r)

        Pass på at plattformen er rotert likt som på modellene. Da er kameraet opp-ned
        """

        corrd_info = queue.get()

        # Notice how the following functions are dependent on whether the ball is on the platform

        if corrd_info == 'nil':  # Checks if the output is nil
            print('Ball: Gone')
            ball_on_platform = False
        else:
            # Here the position of the ball is retrieved and stored as a position in mm relative to the center
            for i in range(len(ball_pos_mm)):

                # Sometimes a specific letter is output instead of a number. Here is the current workaround:
                if corrd_info[i] != 'i' and corrd_info[i] != 'n' and corrd_info[i] != 'l':
                    limit(corrd_info[i], -26, 26)
                    ball_pos_mm[i] = map_range(int(corrd_info[i]), -26, 26, -plat_r, plat_r)  # translate camera pos to mm

            print('ball position in mm: ', ball_pos_mm)

            if is_ball_in_circle(ball_pos_mm, (0, 0), plat_r):
                ball_on_platform = True
                # print('ball_pos_mm[i] : ', ball_pos_mm)
            else:
                ball_on_platform = False
                print('Ball: Off platform')

        if ball_on_platform:
            errors = (- ball_pos_mm[0], - ball_pos_mm[1], - ball_pos_mm[2])

            angles = PID_regulator(errors)

            servos_z = p_prz_bottom_matrix(angles[0], angles[1])

            ang1 = servos_z[0]
            ang2 = servos_z[1]
            ang3 = servos_z[2]

            ang1 = limit(ang1, servo1_angle_limit_negative, servo1_angle_limit_positive)
            ang2 = limit(ang2, servo2_angle_limit_negative, servo2_angle_limit_positive)
            ang3 = limit(ang3, servo3_angle_limit_negative, servo3_angle_limit_positive)

            all_angle_assign(ang1, ang2, ang3)


    def write_arduino(data):
        # print('The angles send to the arduino : ', data)
        arduino.write(bytes(data, 'utf-8'))

    def write_servo(servo_angles):

        offset_angles = (0, 0, 0)

        servo1_offset = offset_angles[0]
        servo2_offset = offset_angles[1]
        servo3_offset = offset_angles[2]

        angles: tuple = (round(math.degrees(servo_angles[0]) + servo1_offset, 1),
                               round(math.degrees(servo_angles[1]) + servo2_offset, 1),
                               round(math.degrees(servo_angles[2]) + servo3_offset, 1))


        # For testing the calibration of the servo offsets
        testing = False

        if testing:
            # Sets the same angle for all the servos
            test_angle = -20
            # Here we set the different offsets and see if the ball stays still
            test_offset = (0 + test_angle, 0 + test_angle, 0 + test_angle)

            write_arduino(str(test_offset))
        else:
            # print('Servo angles: ', angles)
            write_arduino(str(angles))

    while key2:
        writeCoord()

    root.mainloop()  # running loop


if __name__ == '__main__':
    queue = Queue()  # The queue is done inorder for the communication between the two processes.
    key1 = 1  # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target=ball_track, args=(key1, queue))  # initiate ball tracking process
    p2 = mp.Process(target=servo_control, args=(key2, queue))  # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()