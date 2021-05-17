import time
import math
from roboclaw_3 import Roboclaw
from motor import Motor, CornerMotor
import threading


# arc turn constants
R_OUTER = 0.32  # 320 mm dist between corner wheels
R_HEIGHT = 0.29  # 290 mm dist between rover center and front
MAX_TURN = 36
CALIBRATION_SPEED = 30
STOP_THRESHOLD = 100
MOVING_THRESHOLD = 50
SLEEP_TIME = 0.25

# number of seconds per degree for tank turn
# SECONDS_PER_DEGREE = 0.12722222
SECONDS_PER_DEGREE = 39.8 / 360

rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()

# 0x80 -> 128 -> roboclaw #1 wheels 4 & 5 for wheel spin
# 0x81 -> 129 -> roboclaw #2 wheels 6 & 7 for wheel spin
# 0x82 -> 130 -> roboclaw #3 wheels 8 & 9 for wheel spin
# 0x83 -> 131 -> roboclaw #4 wheels 4 & 6 for wheel rotation
# 0x84 -> 132 -> roboclaw #5 wheels 7 & 9 for wheel rotation

RC_ADDR_FL = 0x80
RC_ADDR_FR = 0x81
RC_ADDR_BR = 0x82
RC_ADDR_BL = 0x83
RC_ADDR_MID = 0x84

# Driving motors
WHEEL_FL = Motor(rc, RC_ADDR_FL, 1)  # (rc_addr, mi or m2)
WHEEL_FR = Motor(rc, RC_ADDR_FR, 1)  # (rc_addr, mi or m2)
WHEEL_ML = Motor(rc, RC_ADDR_MID, 1) # MID wheels might be different order (1 - left, 2 - right)
WHEEL_MR = Motor(rc, RC_ADDR_MID, 2)
WHEEL_BL = Motor(rc, RC_ADDR_BL, 1)
WHEEL_BR = Motor(rc, RC_ADDR_BR, 1)

# Articulating / turning motors
CORNER_FL = CornerMotor(rc, RC_ADDR_FL, 2)
CORNER_FR = CornerMotor(rc, RC_ADDR_FR, 2)
CORNER_BL = CornerMotor(rc, RC_ADDR_BL, 2)
CORNER_BR = CornerMotor(rc, RC_ADDR_BR, 2)

CORNERS = [CORNER_FL, CORNER_FR, CORNER_BL, CORNER_BR]

WHEELS = [WHEEL_FL, WHEEL_FR, WHEEL_ML, WHEEL_MR, WHEEL_BL, WHEEL_BR]
WHEELS_LEFT = [WHEEL_FL, WHEEL_ML, WHEEL_BL]
WHEELS_RIGHT = [WHEEL_FR, WHEEL_MR, WHEEL_BR]

ALL_MOTORS = {
    "wheel_fr": WHEEL_FR,
    "wheel_fl": WHEEL_FL,
    "wheel_ml": WHEEL_ML,
    "wheel_mr": WHEEL_MR,
    "wheel_bl": WHEEL_BL,
    "wheel_br": WHEEL_BR,
    "corner_fl": CORNER_FL,
    "corner_fr": CORNER_FR,
    "corner_bl": CORNER_BL,
    "corner_br": CORNER_BR,
}

# helpers
def set_speed_left_side(direction, speed):
    """ sets all left side wheels to given a direction [forward, backward] and speed (m/s)"""
    for wheel in WHEELS_LEFT:
        wheel.set_motor_speed(direction, speed)


def set_speed_right_side(direction, speed):
    """ sets all right side wheels to given a direction [forward, backward] and speed (m/s)"""
    for wheel in WHEELS_RIGHT:
        wheel.set_motor_speed(direction, speed)


def stop_all_wheels():
    for motor in WHEELS:
        motor.stop()

def kill_all():
    print("stopping all motors")
    for wheel in WHEELS:      
        wheel.stop()
    for corner in CORNERS:
        corner.stop()  

def get_register_speed(speed):
    result2 = (0.002 + speed) // 0.0009  # based on graph velocity formula for m/s
    result2 = int(result2)
    if result2 > 127:
        result2 = 127
    return result2  # velo = 0.0009(reg value) - 0.002


def get_time(speed, dist):  # speed in m/s
    howLong = dist / speed  # velo = distance / time
    return howLong


def get_velo_ms(speed):  # converts register value to velocity in m/s
    msSpeed = (0.0009 * speed) - 0.002
    return msSpeed


# for arc turns
def get_inner_velo(degree, outer_speed):
    deg = int(degree)

    if deg > MAX_TURN:
        deg = MAX_TURN

    outer_velo = float(outer_speed)
    percent_diff = 0.9875 * (math.exp(-0.016 * deg))
    inner_velo = percent_diff * outer_velo
    return inner_velo


def get_arc_time(degree, inner_speed):
    deg = int(degree)

    if deg > MAX_TURN:
        deg = MAX_TURN

    rad = math.radians(deg)
    velo = float(inner_speed)
    inner_dist = R_HEIGHT / (math.tan(rad))
    arc_time = (rad * inner_dist) / velo
    return arc_time

# commands

def rotate(motor_name, direction, angle):
    motor = ALL_MOTORS.get(motor_name, None)
    if motor is None:
        print("invalid motor name: " + motor_name)
        return
    angle = int(angle)
    motor.rotate_n_degrees(direction, angle)

def print_encoders():
    for motor_name in ALL_MOTORS.keys():
        value = ALL_MOTORS[motor_name].encoder_value()
        print("%s: %d" % (motor_name,value))


def position(motor_name, position):
    motor = ALL_MOTORS.get(motor_name, None)
    if motor is None:
        print("invalid motor name: " + motor_name)
        return
    position = int(position)
    motor.go_to_position(position)

def distance(motor_name, distance):
    motor = ALL_MOTORS.get(motor_name, None)
    if motor is None:
        print("invalid motor name: " + motor_name)
        return
    distance = int(distance)
    motor.move_distance(distance)

def move_distance_meters(motor_name, distance):
    distance = float(distance)
    COUNT_PER_METER = 47050
    # COUNT_PER_METER_FL = 122000
    COUNT_PER_METER_FL = 92000
    if motor_name == 'all':
        for wheel_name in ALL_MOTORS:
            if 'wheel' in wheel_name:
                if 'fl' in wheel_name:
                    encoder_dist = distance * COUNT_PER_METER_FL
                    motor = ALL_MOTORS.get(wheel_name)
                    encoder_dist = int(encoder_dist)
                    motor.move_distance(encoder_dist, int(4500 * 5)) #move wheel fl faster
                else:
                    encoder_dist = distance * COUNT_PER_METER
                    motor = ALL_MOTORS.get(wheel_name)
                encoder_dist = int(encoder_dist)
                motor.move_distance(encoder_dist, int(1750*1.5))
        # wait until move complete
        # chose MR randomly, but should be representative
        print("started waiting")
        wait_until_move_complete(WHEEL_MR)
        stop_all_wheels()
        # wait_until_stopped(WHEEL_MR)
        # position = WHEEL_MR.encoder_value() + encoder_dist
        # wait_until_position(position,WHEEL_MR)
        print("finished waiting")
                
    else:
        motor = ALL_MOTORS.get(motor_name, None)
        if motor is None:
            print("invalid motor name: " + motor_name)
            return
        if 'fl' in motor_name:
            encoder_dist = int(distance * COUNT_PER_METER_FL)
            motor.move_distance(encoder_dist, int(4500*5))
        else:
            encoder_dist = int(distance * COUNT_PER_METER)
            motor.move_distance(encoder_dist, int(1750*1.5))
        # wait until move complete
        # chose MR randomly, but should be representative
        print("started waiting")
        wait_until_move_complete(motor)
        # wait_until_stopped(motor)
        # position = motor.encoder_value() + encoder_dist
        # wait_until_position(position,motor)
        print("finished waiting")

# moves at 0.1 m/s velocity indefinitely. NOTE - untested as of 2/17/2021
def move_default_velocity(motor_name, direction):
    if direction != "forward" and direction != "backward":
        direction = "forward" # user messed up, default to forward
    if motor_name == 'all':
        for wheel_name in ALL_MOTORS:
            if 'wheel' in wheel_name:
                motor = ALL_MOTORS.get(wheel_name)
                motor.set_motor_speed(direction, 0.01)
    else:
        motor = ALL_MOTORS.get(motor_name, None)
        if motor is None:
            print("invalid motor name: " + motor_name)
            return
        motor.set_motor_speed(direction, 0.01)

# goes clockwise from back right
def calibrate_all():
    lock = threading.Lock()
    threads = [None] * len(CORNERS)
    for i in range(len(CORNERS)):
        threads[i] = threading.Thread(target=CORNERS[i].calibrate, args=(lock, ))
        threads[i].start()
        # t1 = threading.Thread(target=calib_test, args=(FL_CR, lock, ))
        # t2 = threading.Thread(target=calib_test, args=(BL_CR, lock, ))
    for thread in threads:
        thread.join()
    
    wait_until_move_complete(CORNER_BR)

    for cr in CORNERS:
        cr.stop()
    return 0

def calibrate_one(motor_name):
    motor = ALL_MOTORS.get(motor_name, None)
    if motor is None:
        print("invalid motor name: " + motor_name)
        return
    motor.calibrate()
    wait_until_move_complete(motor)
    motor.stop()

# tells all motors to stop
# invoke: kill or k
def kill_all():
    for wheel in WHEELS:
        wheel.stop()
    for corner in CORNERS:
        corner.stop()
    return 0

def rotate_max(motor_name, direction):
    motor = ALL_MOTORS.get(motor_name, None)
    if motor is None:
        print("invalid motor name: " + motor_name)
        return
    if direction == "right":
        motor.go_to_right_most()
    else:
        motor.go_to_left_most()


def recenter():
    for corner in CORNERS:
        corner.go_to_center()
    
    # chose FL randomly, but should be representative
    wait_until_move_complete(CORNER_FR)
    for corner in CORNERS:
        corner.stop()

def wait_until_move_complete(motor):
    while not motor.move_is_complete():
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)

def wait_until_position(position, motor):
    current = motor.encoder_value()
    diff = abs(current - position)
    while diff > STOP_THRESHOLD:
        current = motor.encoder_value()
        diff = abs(current - position)
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)

def wait_until_stopped(motor):
    current = motor.encoder_value()
    time.sleep(SLEEP_TIME)
    new = motor.encoder_value()
    while(abs(current - new) > MOVING_THRESHOLD):
        current = new
        new = motor.encoder_value()
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)

def degrees_to_duration(degrees):
    degrees = float(degrees) # probably not necessary
    if degrees < 0:
        degrees = 0 # minimum of 0 degrees
    elif degrees > 360:
        degrees = degrees % 360 # convert to within 360 degrees
    duration = float(degrees * SECONDS_PER_DEGREE) # convert degrees to time
    return (duration, degrees) # return duration and possibly converted degrees

def tank(direction, degrees):
    (duration, degrees) = degrees_to_duration(degrees)
    speed = 0.05

    print("Tank mode: %s for speed %.2f seconds, or %d degrees" % (direction, duration, int(degrees)))
    if direction == "cw":
        # set speed
        set_speed_left_side("forward", speed)
        set_speed_right_side("backward", speed)
        # wait for duration
        time.sleep(duration)
        # stop motors
        for wheel in WHEELS:
            wheel.stop()
    else:
        set_speed_left_side("backward", speed)
        set_speed_right_side("forward", speed)
        # wait for duration
        time.sleep(duration)
        # stop motors
        for wheel in WHEELS:
            wheel.stop()

def tank_with_turn(direction, degrees):
    print("Turning wheels")
    print("front right")
    CORNER_FR.go_to_left_most()
    print("front left")
    CORNER_FL.go_to_right_most()
    print("back right")
    CORNER_BR.go_to_right_most()
    print("back left")
    CORNER_BL.go_to_left_most()
    wait_until_move_complete(CORNER_BL)
    for cr in CORNERS:
        cr.stop()

    tank(direction, degrees)


# drives rover straight forward at specified speed for specified distance
def forward(speed, dist):
    speed = float(speed)
    dist = float(dist)

    if dist <= 0:
        print("Invalid distance entered: %.2f" % (dist))
        return -1

    howLong = get_time(speed, dist)

    print("Driving forward at %.4f m/s for %.2f meters" % (speed, howLong))
    direction = "forward"
    for wheel in WHEELS:
        wheel.set_motor_speed(direction, speed)
    time.sleep(howLong)
    for wheel in WHEELS:
        wheel.stop()
    return 0

# drives rover straight forward at specified speed for specified distance
def forward_with_stop(speed):
    speed = float(speed)

    print("Driving forward at %.4f m/s" % (speed))
    direction = "forward"
    
    for wheel in WHEELS:
        wheel.set_motor_speed(direction, speed)
    
    stopper = input("stop?  ")
    while stopper != "y" and stopper != "yes":
        stopper = input("stop? (y/n):  ")

    stop_all_wheels()
    return 0


# drives rover straight backward at specified speed for specified distance
def backward(speed, dist):
    speed = float(speed)
    dist = float(dist)

    if dist <= 0:
        print("Invalid distance entered: %.2f" % (dist))
        return -1

    howLong = get_time(speed, dist)

    print("Driving backward at %.4f m/s for %.2f meters" % (speed, howLong))

    direction = "backward"
    for wheel in WHEELS:
        wheel.set_motor_speed(direction, speed)
    time.sleep(howLong)
    for wheel in WHEELS:
        wheel.stop()
    return 0

# drives rover straight backward at specified speed for specified distance
def backward_with_stop(speed):
    speed = float(speed)

    print("Driving backward at %.4f m/s" % (speed))

    direction = "backward"
    for wheel in WHEELS:
        wheel.set_motor_speed(direction, speed)
    
    stopper = input("stop?  ")
    while stopper != "y" and stopper != "yes":
        stopper = input("stop? (y/n):  ")

    stop_all_wheels()
    return 0


# rotates corner wheels to guessed positions for an arc turn
# inner wheel rotation > outer wheel rotation
#   - inner wheels rotated to their fully turned position (36)
#   - outer wheels rotated slightly less than the inner wheels
#     done by reducing wheel rotation speed
#   (what that angle and speed is, we don't know :P)
def generic_turn(direction):
    # need to change this to use go to position
    if direction == "right":  # right turn, left wheels outer, right inner
        CORNER_FL.set_motor_register_speed("forward", 9)
        CORNER_FR.set_motor_register_speed("forward", CALIBRATION_SPEED)
        CORNER_BL.set_motor_register_speed("backward", 9)
        CORNER_BR.set_motor_register_speed("backward", 23)
        # <comment from GROVER:> BR this one rotates beyond its stopper which we want to avoid
    else:  # left turn, right CORNERs outer
        CORNER_FL.set_motor_register_speed("backward", CALIBRATION_SPEED)
        CORNER_FR.set_motor_register_speed("backward", 10)
        CORNER_BL.set_motor_register_speed("forward", CALIBRATION_SPEED)
        CORNER_BR.set_motor_register_speed("forward", 8)

    time.sleep(3)

    for corner in CORNERS:
        corner.stop()

    return 0


# turns wheels according to turning direction using generic function (not user specified)
# directs command to arc turn speeds and which direction (drive direction, forward/backward)
def turn(speed, direction, dist, drive):
    generic_turn(direction)
    distance = float(dist)

    if dist == 0:
        if drive == "forward":
            return arc_turn_forward(direction, speed)
        else:
            return arc_turn_backward(direction, speed)
    else:
        if drive == "forward":
            return arc_turn_forward_dist(direction, speed, distance)
        else:
            return arc_turn_backward_dist(direction, speed, distance)
    return 0


# drives rover forward for arc turns at specified speed until user tells it to stop
# at some point can have a specified distance to turn until, so time limit used from there
def arc_turn_forward(direction, speed):
    outer_speed = float(speed)
    inner_speed = get_inner_velo(MAX_TURN, outer_speed)

    if direction == "right":  # right turn: right inner, left outer
        set_speed_left_side("forward", outer_speed)
        set_speed_right_side("forward", inner_speed)
    else:  # left turn: left inner, right outer
        set_speed_left_side("forward", inner_speed)
        set_speed_right_side("forward", outer_speed)

    stopper = input("stop?  ")
    while stopper != "y" and stopper != "yes":
        stopper = input("stop? (y/n):  ")

    stop_all_wheels()

    return 0


# same as above but backwards
def arc_turn_backward(direction, speed):
    outer_speed = float(speed)
    inner_speed = get_inner_velo(MAX_TURN, outer_speed)

    if direction == "right":  # right turn: right inner, left outer
        set_speed_left_side("backward", outer_speed)
        set_speed_right_side("backward", inner_speed)
    else:  # left turn: left inner, right outer
        set_speed_left_side("backward", inner_speed)
        set_speed_right_side("backward", outer_speed)

    stopper = input("stop?  ")
    while stopper != "y" and stopper != "yes":
        stopper = input("stop?  ")

    stop_all_wheels()

    return 0


# drives rover forward for arc turn at specified speed for specified distance
def arc_turn_forward_dist(direction, speed, dist):
    outer_speed = float(speed)
    inner_speed = get_inner_velo(MAX_TURN, outer_speed)
    timer = get_time(speed, dist)
    if direction == "right":  # right turn: right inner, left outer
        set_speed_left_side("forward", outer_speed)
        set_speed_right_side("forward", inner_speed)
    else:  # left turn: left inner, right outer
        set_speed_left_side("forward", inner_speed)
        set_speed_right_side("forward", outer_speed)

    time.sleep(timer)

    stop_all_wheels()

    return 0


# drives rover backward for arc turn at specified speed for specified distance
def arc_turn_backward_dist(direction, speed, dist):
    outer_speed = float(speed)
    inner_speed = get_inner_velo(MAX_TURN, outer_speed)
    timer = get_time(speed, dist)
    if direction == "right":  # right turn: right inner, left outer
        set_speed_left_side("backward", outer_speed)
        set_speed_right_side("backward", inner_speed)
    else:  # left turn: left inner, right outer
        set_speed_left_side("backward", inner_speed)
        set_speed_right_side("backward", outer_speed)

    time.sleep(timer)

    stop_all_wheels()

    return 0

def autonomous():
    calibrate_all()
    while True:
        move_distance_meters("all", 1) # this will be changed to move until
                                       # an obstacle is detected
        tank_with_turn("cw", 90)
        recenter()

    return 0