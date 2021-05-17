from roboclaw_3 import Roboclaw
from motor import Motor, CornerMotor
from datetime import datetime
import math
import time
import threading

rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()

FL_ADDR = 0x80
BL_ADDR = 0x83
FL_CR = CornerMotor(rc, FL_ADDR, 2)
FL_DR = Motor(rc, FL_ADDR, 1)
BL_CR = CornerMotor(rc, BL_ADDR, 2)
BL_DR = Motor(rc, BL_ADDR, 1)

#have to do some roundabout calibration, since code is set for absolute motors still
# FL_CR.encoders_per_degree = 2350 / 90 #testing showed it was about 8300 for a full revolution
# FL_CR.calibrated = True

CALIBRATION_TIME = 9
CALIBRATION_SPEED = 25
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
    #rc.BackwardM2(FL_ADDR, CALIBRATION_SPEED)
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
    #rc.BackwardM2(FL_ADDR, CALIBRATION_SPEED)
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

def calib_test(corner, lock):
    left_most = 0
    right_most = 0
    runs = 0
    while (right_most - left_most) < 1800 and runs < 3: # max range is ~2000, run until this is about right
        # turn to left-most position, store left-most encoder value in global var
        with lock:
            prev_encoder = corner.encoder_value()
            print("starting encoder value: " + str(prev_encoder))
            print(prev_encoder)
            corner.set_motor_register_speed("backward", CALIBRATION_SPEED)
            start = time.time()
            time.sleep(0.1)
            i = 0
            while time.time() - start < CALIBRATION_TIME:
                time.sleep(0.05)
                curr_encoder = corner.encoder_value()
                if abs(curr_encoder - prev_encoder) < 3:
                    print(corner, "breaking on encoder condition")
                    i += 1
                if i == 3:
                    break
                prev_encoder = curr_encoder
            corner.stop()
            corner.move_distance(80)
            while not corner.move_is_complete():
                pass
            corner.stop()
            prev_encoder = corner.encoder_value()
            print(prev_encoder)
            corner.set_motor_register_speed("backward", CALIBRATION_SPEED)
            start = time.time()
            while time.time() - start < CALIBRATION_TIME:
                time.sleep(0.05)
                curr_encoder = corner.encoder_value()
                if abs(curr_encoder - prev_encoder) < 3:
                    print(corner, "breaking on encoder condition, second time")
                    break
                prev_encoder = curr_encoder
            corner.stop()
            left_most = corner.encoder_value()
        time.sleep(2)
        prev_encoder = left_most
        print(prev_encoder)
        with lock:
            corner.set_motor_register_speed("forward", CALIBRATION_SPEED)
            start = time.time()
            time.sleep(0.1)
            while time.time() - start < CALIBRATION_TIME:
                time.sleep(0.05)
                curr_encoder = corner.encoder_value()
                if abs(curr_encoder - prev_encoder) < 3:
                    print(corner, "breaking on encoder condition, forward")
                    break
                prev_encoder = curr_encoder
                #print(prev_encoder)

            corner.stop()
            right_most = corner.encoder_value()   # store right-most encoder val, calculate center
        runs += 1
        print("left, right", left_most, right_most, "DIFFERENCE: ", right_most - left_most)
        
    corner.left_most = left_most + 50 
    corner.right_most = right_most - 50 # edit range of motion so arms don't hit physical stops in most cases
    corner.center = (right_most + left_most) // 2
    print("center is: ", corner.center)

    corner.calibrated = True
    # go to center position
    with lock:
        corner.go_to_center()
        while not corner.move_is_complete():
            pass

    # calculate encoder to angle value
    encoder_range = abs(corner.right_most - corner.left_most)
    corner.encoders_per_degree = encoder_range / ANGULAR_RANGE
    return (left_most, corner.center, right_most)


lock = threading.Lock()
t1 = threading.Thread(target=FL_CR.calibrate, args=(lock, ))
t2 = threading.Thread(target=BL_CR.calibrate, args=(lock, ))
t1.start()
t2.start()
t1.join()
t2.join()
print("did it work?")
print(FL_CR)
print(BL_CR)

FL_DR.move_distance(1000)
BL_DR.move_distance(1000)
while not BL_DR.move_is_complete():
    pass
FL_DR.stop()
BL_DR.stop()
FL_DR.move_distance(-1000)
BL_DR.move_distance(-1000)
while not BL_DR.move_is_complete():
    pass
FL_DR.stop()
BL_DR.stop()
