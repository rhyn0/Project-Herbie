from roboclaw_3 import Roboclaw
from motor import Motor, CornerMotor
from datetime import datetime
import math
import time

rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()

POC_ADDR = 0x80
QUAD_CR = CornerMotor(rc, POC_ADDR, 2)
QUAD_DR = Motor(rc, POC_ADDR, 1)

#have to do some roundabout calibration, since code is set for absolute motors still
# QUAD_CR.encoders_per_degree = 2350 / 90 #testing showed it was about 8300 for a full revolution
# QUAD_CR.calibrated = True

CALIBRATION_TIME = 9
CALIBRATION_SPEED = 20
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
    corner.set_motor_register_speed("backward", CALIBRATION_SPEED)
    #rc.BackwardM2(POC_ADDR, CALIBRATION_SPEED)
    start = time.time()
    time.sleep(0.1)
    while time.time() - start < CALIBRATION_TIME:
        time.sleep(0.01)
        curr_encoder = corner.encoder_value()
        if abs(curr_encoder - prev_encoder) < 3:
            print("breaking on encoder condition")
            corner.stop()
            break
        prev_encoder = curr_encoder
    corner.stop()
    left_most = corner.encoder_value()
    #time to go right
    corner.set_motor_register_speed("forward", CALIBRATION_SPEED)
    #rc.BackwardM2(POC_ADDR, CALIBRATION_SPEED)
    start = time.time()
    time.sleep(0.1)
    while time.time() - start < CALIBRATION_TIME:
        time.sleep(0.01)
        curr_encoder = corner.encoder_value()
        if abs(curr_encoder - prev_encoder) < 3:
            corner.stop()
            break
        prev_encoder = curr_encoder
    corner.stop()
    right_most = corner.encoder_value()
    print("left: " + str(left_most) + "; right: " + str(right_most))

def timed_test(corner):
    print(datetime.now().isoformat(timespec='milliseconds')[-9:])
    while True:
        print("encoder val: " + str(corner.encoder_value()))
        print(datetime.now().isoformat(timespec='milliseconds')[-9:])
        time.sleep(0.001)

def calib_test(corner):
    left_most = 0
    right_most = 0

    while (right_most - left_most) < 1800: # max range is ~2000, run until this is about right
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
        rc.ForwardM2(POC_ADDR, SLOWER_CALIBRATION_SPEED)
        time.sleep(0.5)
        corner.stop()
        prev_encoder = corner.encoder_value()
        print(prev_encoder)
        corner.set_motor_register_speed("backward", CALIBRATION_SPEED + 5)
        start = time.time()
        while time.time() - start < CALIBRATION_TIME:
            curr_encoder = corner.encoder_value()
            print(curr_encoder)
            if curr_encoder - prev_encoder < 3:
                print("breaking on encoder condition, second time")
                break
            prev_encoder = curr_encoder
            print(prev_encoder)
        left_most = corner.encoder_value()
        
        prev_encoder = corner.encoder_value()
        print(prev_encoder)
        corner.set_motor_register_speed("forward", CALIBRATION_SPEED)
        start = time.time()
        while time.time() - start < CALIBRATION_TIME:
            curr_encoder = corner.encoder_value()
            print(curr_encoder)
            if curr_encoder - prev_encoder < 3:
                print("breaking on encoder condition, forward")
                break
            prev_encoder = curr_encoder
            print(prev_encoder)

        corner.stop()
        right_most = corner.encoder_value()     # store right-most encoder val, calculate center

    corner.left_most = left_most
    corner.right_most = right_most
    corner.center = (right_most + left_most) // 2
    
    corner.calibrated = True
    # go to center position
    corner.go_to_center()

    # calculate encoder to angle value
    encoder_range = abs(corner.right_most - corner.left_most)
    corner.encoders_per_degree = encoder_range / ANGULAR_RANGE
    return (left_most, corner.center, right_most)

encoder_test(QUAD_CR)
