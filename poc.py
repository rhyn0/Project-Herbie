from roboclaw_3 import Roboclaw
from motor import Motor, CornerMotor
import time
import math

rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()

POC_ADDR = 0x80
QUAD_CR = CornerMotor(rc, POC_ADDR, 2)
QUAD_DR = Motor(rc, POC_ADDR, 1)

#have to do some roundabout calibration, since code is set for absolute motors still
# QUAD_CR.encoders_per_degree = 2350 / 90 #testing showed it was about 8300 for a full revolution
# QUAD_CR.calibrated = True

CALIBRATION_TIME = 9
CALIBRATION_SPEED = 10
SLOWER_CALIBRATION_SPEED = 5

MAX_CORNER_ENC = 1550
INVALID_ENC = 1600

GOTO_SPEED = 1750
GOTO_FR = 4542
ACCEL = DECCEL = 1000

ANGULAR_RANGE = 90

def encoder_test(corner):
    prev_encoder = corner.encoder_value()
    print("starting encoder value: " + str(prev_encoder))
    start = time.time()
    while time.time() - start < CALIBRATION_TIME:
        curr_encoder = corner.encoder_value()
        print("current encoder value: " + str(curr_encoder))
        if curr_encoder - prev_encoder < 3:
            print("breaking on encoder condition")
            break
        prev_encoder = curr_encoder
        print(prev_encoder)

def timed_test(corner):
    print(time.time())
    while True:
        print("encoder val: " + str(corner.encoder_value()))
        print(time.time())

def calib_test(corner):
    flag = 0
    left_most = 0
    right_most = 0
    centered = 0
    
    # turn to left-most position, store left-most encoder value in global var
    prev_encoder = corner.encoder_value()
    print(prev_encoder)
    corner.set_motor_register_speed("backward", CALIBRATION_SPEED)
    start = time.time()
    while time.time() - start < CALIBRATION_TIME:
        curr_encoder = corner.encoder_value()
        print(curr_encoder)
        if curr_encoder - prev_encoder < 3:
            print("breaking on encoder condition")
            break
        prev_encoder = curr_encoder
        print(prev_encoder)
    #time.sleep(CALIBRATION_TIME) # trying to avoid using sleep since that will mess with the point of having multithread
    corner.stop()
    left_most = corner.encoder_value()
    corner.left_most = left_most
    # turn to right-most position, adding 1550 to running total if the encoder vals wrap
    corner.set_motor_register_speed("forward", CALIBRATION_SPEED)
    # 11/17/20 I don't think this while loop is necessary
    start = time.time()
    while time.time() - start < CALIBRATION_TIME:
        if (
            corner.encoder_value() >= 1500 and flag == 0
        ):  # not using 1550 cuz the enc values change fast, so giving it wide range
            centered += MAX_CORNER_ENC
            flag = 1

    corner.stop()
    right_most = corner.encoder_value()     # store right-most encoder val, calculate center

    corner.right_most = right_most
    

    corner.center = (right_most + left_most) // 2
    
    corner.calibrated = True
    # go to center position
    corner.go_to_center()

    # calculate encoder to angle value
    encoder_range = abs(corner.right_most - corner.left_most)
    corner.encoders_per_degree = encoder_range / ANGULAR_RANGE
    return (left_most, corner.center, right_most)

timed_test(QUAD_CR)
