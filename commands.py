import time
import math
from roboclaw_3 import Roboclaw
from motor import Motor, CornerMotor
import threading


# arc turn constants
R_OUTER = 0.31  # 310 mm dist between front/back corner wheels
R_OUTER_MID = 0.4 # 400 mm dist between center drive wheels
R_HEIGHT = 0.556 / 2 # 290 mm dist between rover center and front
MAX_TURN = 36
MAX_SPEED_VEL = 0.05
CALIBRATION_SPEED = 30
STOP_THRESHOLD = 100
MOVING_THRESHOLD = 50
SLEEP_TIME = 0.25

# number of seconds per degree for tank turn
# SECONDS_PER_DEGREE = 0.12722222
SECONDS_PER_DEGREE = 39.8 / 360

rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()

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
def set_speed_left_side(direction, cen_speed, speed=None):
    """ sets all left side wheels to given a direction [forward, backward] and speed (m/s)
    Center wheel speed is different to allow arc turns
    """
    if speed is None:
        speed = cen_speed
    for wheel in WHEELS_LEFT:
        if wheel is WHEEL_ML:
            wheel.set_motor_speed(direction, cen_speed)
        else:
            wheel.set_motor_speed(direction, speed)


def set_speed_right_side(direction, cen_speed, speed=None):
    """ sets all right side wheels to given a direction [forward, backward] and speed (m/s)
    Center wheel speed is different to allow arc turns
    """
    if speed is None:
        speed = cen_speed
    for wheel in WHEELS_RIGHT:
        if wheel is WHEEL_MR:
            wheel.set_motor_speed(direction, cen_speed)
        else:
            wheel.set_motor_speed(direction, speed)


def stop_all_wheels():
    """ Stops all drive wheels on call
    """
    for motor in WHEELS:
        motor.stop()

def kill_all():
    """ Stops all wheels on call
    """
    print("stopping all motors")
    for wheel in WHEELS:      
        wheel.stop()
    for corner in CORNERS:
        corner.stop()  


def get_time(speed, dist): 
    """ Simple speed = dist * time calculation
    """ 
    howLong = abs(dist / speed)  # velo = distance / time
    return howLong


def get_velo_ms(speed): 
    """converts register value to velocity in m/s
    Opposite conversion to get_register_speed
    """
    msSpeed = (0.0009 * speed) - 0.002
    return msSpeed


# for arc turns
def get_inner_velo(radius, outer_speed):
    """ Return speed for inner drive wheels. 
    
    Parameters
    -----------
    radius: float
    outer_speed: float
        speed of center wheel on the outside of arc turn
    
    Returns
    ----
    inner_velo: float
        Value for motor addresses 0x81, 0x82 in m/s
    inner_cen_velo: float
        Value for motor address 0x84 in m/s
    """
    inner_velo = outer_speed * (math.sqrt(radius ** 2 - 0.31 * radius + 0.101)\
        / (radius + 0.155))
    inner_cen_velo = outer_speed * ((radius - 0.155) / (radius + 0.155))
    return (inner_velo, inner_cen_velo)


def get_arc_time(degree, inner_speed):
    """     
    Parameters
    -----------
    radius: float
    outer_speed: float
        speed of center wheel on the outside of arc turn
    
    Returns
    ----
    inner_velo: float
        Value for motor addresses 0x81, 0x82 in m/s
    inner_cen_velo: float
        Value for motor address 0x84 in m/s
    """
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
    wait_until_move_complete(CORNER_BR)
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

def forward(speed, dist):
    """ drives rover straight forward at specified speed for specified distance
    """
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

def forward_with_stop(speed):
    """ drives rover straight forward at specified speed for unspecified distance
    Input [y/yes] for it to stop driving.
    """
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

def backward(speed, dist):
    """ drives rover straight backward at specified speed for specified distance
    """
    speed = float(speed)
    dist = float(dist)

    if dist <= 0:
        print("Invalid distance entered: %.2f" % (dist))
        return -1

    howLong = get_time(speed, dist)

    print("Driving backward at %.4f m/s for %.2f meters" % (speed, howLong))

    for wheel in WHEELS:
        wheel.set_motor_speed("backward", speed)
    time.sleep(howLong)
    for wheel in WHEELS:
        wheel.stop()
    return 0


def backward_with_stop(speed):
    """ drives rover straight backward at specified speed for unspecified distance
    Input [y/yes] for it to stop driving.
    """
    speed = float(speed)

    print("Driving backward at %.4f m/s" % (speed))

    for wheel in WHEELS:
        wheel.set_motor_speed("backward", speed)
    
    stopper = input("stop?  ")
    while stopper not in ['y', 'yes']:
        stopper = input("stop? (y/n):  ")

    stop_all_wheels()
    return 0

def turn(radius, direction, dist, drive):
    """ Start an arc turn
    
    Parameters
    ----
    radius: float
    direction: str
        Which way will it turn, 'right' or 'left'
    dist: float
    drive: str
        Which driving direction, 'forward' or 'backward'
    
    Returns
    -----
    int: 0 for completion, -1 for errors
    """
    direction = direction.lower()
    drive = drive.lower()
    if 0.464 > radius or radius > 300 or \
        direction not in ["right", "left"] or \
        drive not in ['forward', 'backward']: 
        return -1 
    set_arc_wheels(direction, radius)
    distance = float(dist)
    return arc_turn_drive(direction, radius, distance, drive)
    

def set_arc_wheels(direction, radius) -> None:
    """ Turns corner wheels to position to make arc turn
    
    Parameters
    ----
    direction: str
        Turn which way, needs to be 'right' or 'left'
    radius: float
        Radius of circle, measured to center rover
    """
    recenter()
    outer_deg = math.degrees(math.atan(R_HEIGHT / (radius + R_OUTER / 2)))
    inner_deg = math.degrees(math.atan(R_HEIGHT / (radius - R_OUTER / 2)))
    CORNER_FL.rotate_n_degrees(direction, outer_deg if direction == 'right' else -1 * inner_deg)
    CORNER_FR.rotate_n_degrees(direction, -1 * outer_deg if direction == 'left' else inner_deg)
    CORNER_BL.rotate_n_degrees(direction, -1 * outer_deg if direction == 'right' else inner_deg)
    CORNER_BR.rotate_n_degrees(direction, outer_deg if direction == 'left' else -1 * inner_deg)

# drives rover forward for arc turn at specified speed for specified distance
def arc_turn_drive(direction, radius, dist, drive):
    """ Process of doing an arc turn. Has constant speed determined by constants at top of file.
    Make use of Motor.move_distance function
    
    Parameters
    -----------
    direction: str
        specifies which way to turn, needs to be either 'right' or 'left'
    radius: float
        distance from center rover to center circle for the arc that is about to be traveled
    dist: float
        arc length to traverse
    drive: str
        which way to spin wheels, 'forward' or 'backward'
        
    Returns
    -----------
    int
        0 on successful completion, -1 if errors in parameters
    """
    out_cen_speed = MAX_SPEED_VEL
    out_speed = out_cen_speed * (math.sqrt(radius ** 2 + 0.31 * radius + 0.101)\
        / (radius + 0.155))
    inner_speed, inner_cen_speed = get_inner_velo(MAX_TURN, out_cen_speed)
    timer = get_time(out_cen_speed, dist)
    if direction == "right":  # right turn: right inner, left outer
        set_speed_left_side(drive, out_cen_speed, out_speed)
        set_speed_right_side(drive, inner_cen_speed, inner_speed)
    else:  # left turn: left inner, right outer
        set_speed_left_side(drive, inner_cen_speed, inner_speed)
        set_speed_right_side(drive, out_cen_speed, out_speed)

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