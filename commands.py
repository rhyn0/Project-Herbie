import time
import math
import threading
from . import roboclaw_3
from . import motor


# arc turn constants
R_OUTER = 0.31  # 310 mm dist between front/back corner wheels
R_OUTER_MID = 0.4  # 400 mm dist between center drive wheels
R_HEIGHT = 0.556 / 2  # 290 mm dist between rover center and front
MAX_TURN = 36
MAX_SPEED_VEL = 0.05
CALIBRATION_SPEED = 30
STOP_THRESHOLD = 100
MOVING_THRESHOLD = 50
SLEEP_TIME = 0.25

# number of seconds per degree for tank turn
# SECONDS_PER_DEGREE = 0.12722222
SECONDS_PER_DEGREE = 39.8 / 360

rc = roboclaw_3.Roboclaw("/dev/ttyS0", 115200)
rc.Open()

RC_ADDR_FL = 0x80
RC_ADDR_FR = 0x81
RC_ADDR_BR = 0x82
RC_ADDR_BL = 0x83
RC_ADDR_MID = 0x84

# Driving motors
WHEEL_FL = motor.Motor(rc, RC_ADDR_FL, 1)  # (rc_addr, mi or m2)
WHEEL_FR = motor.Motor(rc, RC_ADDR_FR, 1)  # (rc_addr, mi or m2)
WHEEL_ML = motor.Motor(
    rc, RC_ADDR_MID, 1
)  # MID wheels might be different order (1 - left, 2 - right)
WHEEL_MR = motor.Motor(rc, RC_ADDR_MID, 2)
WHEEL_BL = motor.Motor(rc, RC_ADDR_BL, 1)
WHEEL_BR = motor.Motor(rc, RC_ADDR_BR, 1)

# Articulating / turning motors
CORNER_FL = motor.CornerMotor(rc, RC_ADDR_FL, 2)
CORNER_FR = motor.CornerMotor(rc, RC_ADDR_FR, 2)
CORNER_BL = motor.CornerMotor(rc, RC_ADDR_BL, 2)
CORNER_BR = motor.CornerMotor(rc, RC_ADDR_BR, 2)

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
    inner_velo = outer_speed * (
        math.sqrt(radius ** 2 - 0.31 * radius + 0.101) / (radius + 0.155)
    )
    inner_cen_velo = outer_speed * ((radius - 0.155) / (radius + 0.155))
    return (inner_velo, inner_cen_velo)


## commands


def rotate(motor_name, direction, angle) -> int:
    """Rotates a single CornerMotor given degrees in given direction
    """
    mtr = ALL_MOTORS.get(motor_name, None)
    if mtr is None or not isinstance(mtr, motor.CornerMotor):
        print(f"Invalid motor name: {motor_name}")
        return -1
    angle = int(angle)
    mtr.rotate_n_degrees(direction, angle)
    return 0


def print_encoders() -> None:
    """Prints out all the encoder register values
    """
    for motor_name in ALL_MOTORS.keys():
        value = ALL_MOTORS[motor_name].encoder_value()
        print(f"{motor_name}: {value}")


def position(motor_name, position) -> int:
    """Moves a single wheel to a certain encoder value
    """
    mtr = ALL_MOTORS.get(motor_name, None)
    if mtr is None:
        print(f"Invalid motor name: {motor_name}")
        return -1
    position = int(position)
    return mtr.go_to_position(position)


def move_distance_meters(motor_name, distance):
    """Moves a singular motor, or all motors, a certain distance in meters
    """
    distance = float(distance)
    COUNT_PER_METER = 47050
    if motor_name == "all":
        for wheel in WHEELS:
            encoder_dist = int(distance * COUNT_PER_METER)
            wheel.move_distance(encoder_dist, int(1750 * 1.5))
        # wait until move complete
        # chose MR randomly, but should be representative
        wait_until_move_complete(WHEEL_BR)
        stop_all_wheels()
    else:
        mtr = ALL_MOTORS.get(motor_name, None)
        if mtr is None:
            print(f"Invalid motor name: {motor_name}")
            return -1
        encoder_dist = int(distance * COUNT_PER_METER)
        mtr.move_distance(encoder_dist, int(1750 * 1.5))
        # wait until move complete
        wait_until_move_complete(mtr)


#
def move_default_velocity(motor_name, direction) -> int:
    """Moves a specific wheel or all of them at 0.1 m/s velocity indefinitely. 
    NOTE - untested as of 2/17/2021
    """
    direction = direction.lower()
    if direction not in ["forward", "backward"]:
        direction = "forward"  # user messed up, default to forward
    if motor_name == "all":
        for wheel in WHEELS:
            wheel.set_motor_speed(direction, 0.01)
    else:
        mtr = ALL_MOTORS.get(motor_name, None)
        if mtr is None:
            print(f"Invalid motor name: {motor_name}")
            return -1
        mtr.set_motor_speed(direction, 0.01)
    return 0


def calibrate_all():
    """Spawns individual threads for each Roboclaw controller and calibrates each
    Uses multithread.Lock to share the resource of single TXD/RXD line
    """
    lock = threading.Lock()
    thrds: list[threading.Thread] = []
    for i in range(len(CORNERS)):
        thrds.append(threading.Thread(target=CORNERS[i].calibrate, args=(lock,)))
        thrds[i].start()
        # t1 = threading.Thread(target=calib_test, args=(FL_CR, lock, ))
        # t2 = threading.Thread(target=calib_test, args=(BL_CR, lock, ))
    for thread in thrds:
        thread.join()

    # wait_until_move_complete(CORNER_BR)
    wait_until_all_complete()

    for cr in CORNERS:
        cr.stop()
    return 0


def calibrate_one(motor_name) -> int:
    """Calibrate a single CornerMotor, no need to pass a Lock
    
    Returns
    ----
    0
        success
    -1
        errors
    """
    mtr = ALL_MOTORS.get(motor_name, None)
    if mtr is None:
        print("invalid motor name: " + motor_name)
        return -1
    if isinstance(mtr, motor.CornerMotor):
        mtr.calibrate()
        wait_until_move_complete(motor)
        mtr.stop()
        return 0
    else:
        print(f"Invalid motor: {motor_name}")
        return -1


def kill_all():
    """Stops all motors
    """
    for wheel in ALL_MOTORS.values():
        wheel.stop()
    return 0


def rotate_max(motor_name: str, direction: str) -> int:
    """Chooses a single corner wheel to rotate to the maximum value
    Corner wheel must be selected, must be calibrated
    Direction must be right or left
    """
    direction = direction.lower()
    mtr = ALL_MOTORS.get(motor_name, None)
    if mtr is None or not isinstance(mtr, motor.CornerMotor):
        print(f"Invalid motor name: {motor_name}")
        return -1
    if direction == "right":
        return mtr.go_to_right_most()
    elif direction == "left":
        return mtr.go_to_left_most()
    else:
        return -1


def recenter():
    for corner in CORNERS:
        corner.go_to_center()

    # chose FL randomly, but should be representative
    # wait_until_move_complete(CORNER_BR)
    wait_until_all_complete()
    for corner in CORNERS:
        corner.stop()


def wait_until_all_complete():
    """Waits until every motor is done moving before returning
    """
    while True:
        if all([mtr.move_is_complete() for mtr in ALL_MOTORS.values()]):
            break
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)


def wait_until_move_complete(motor):
    """Waits until a specific motor is done moving before returning
    """
    while not motor.move_is_complete():
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)


def wait_until_position(position, motor):
    """Waits until a specific motor moves within a certain threshold
    to a certain encoder position
    
    UNUSED as of 6/6/2021
    """
    current = motor.encoder_value()
    diff = abs(current - position)
    while diff > STOP_THRESHOLD:
        current = motor.encoder_value()
        diff = abs(current - position)
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)


def wait_until_stopped(motor):
    """Waits until a specific motor stops
    
    UNUSED as of 6/6/2021
    """
    current = motor.encoder_value()
    time.sleep(SLEEP_TIME)
    new = motor.encoder_value()
    while abs(current - new) > MOVING_THRESHOLD:
        current = new
        new = motor.encoder_value()
        time.sleep(SLEEP_TIME)
    # extra sleep for good measure
    time.sleep(SLEEP_TIME)


def degrees_to_duration(degrees):
    """If degrees is less than or equal to 0, returns 0. 
    """
    degrees = float(degrees)  # probably not necessary
    if degrees < 0:
        degrees = 0  # minimum of 0 degrees
    elif degrees > 360:
        degrees = degrees % 360  # convert to within 360 degrees
    duration = float(degrees * SECONDS_PER_DEGREE)  # convert degrees to time
    return (duration, degrees)  # return duration and possibly converted degrees


def tank(direction, degrees):
    """Executes the actual wheel spinning to cause rover to rotate
    """
    direction = direction.lower()
    duration, degrees = degrees_to_duration(degrees)
    speed = 0.05
    if degrees == 0:
        print(
            "Attempted to turn 0 or negative amount of degrees, input positive values only"
        )
        return -1
    print(
        f"Tank mode: {direction} for speed {duration:.2f} seconds, or {int(degrees)} degrees"
    )
    if direction == "cw":
        # set speed
        set_speed_left_side("forward", speed)
        set_speed_right_side("backward", speed)
        # wait for duration
        time.sleep(duration)
        # stop motors
        for wheel in WHEELS:
            wheel.stop()
    elif direction == "ccw":
        set_speed_left_side("backward", speed)
        set_speed_right_side("forward", speed)
        # wait for duration
        time.sleep(duration)
        # stop motors
        for wheel in WHEELS:
            wheel.stop()
    else:
        return -1


def tank_with_turn(direction, degrees):
    """Sets the CornerWheels to proper alignment to do the tankturn
    """
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
        print(f"Invalid distance entered: {dist:.2f}. Positive values only.")
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

    stopper = input("stop?  ").lower()
    while stopper not in ["y", "yes"]:
        stopper = input("stop? (y/n):  ").lower()

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
    while stopper not in ["y", "yes"]:
        stopper = input("stop? (y/n):  ")

    stop_all_wheels()
    return 0


def turn(drive, direction, radius, dist) -> int:
    """ Start an arc turn
    Command format: arc drive direction radius dist
    
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
    radius = float(radius)
    if not all([motor.calibrated for motor in CORNERS]):
        print("Motors not calibrated")
        return -1
    if (
        0.464 > radius
        or radius > 300
        or direction not in ["right", "left"]
        or drive not in ["forward", "backward"]
    ):
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
    CORNER_FL.rotate_n_degrees(
        direction, outer_deg if direction == "right" else -1 * inner_deg
    )
    CORNER_FR.rotate_n_degrees(
        direction, -1 * outer_deg if direction == "left" else inner_deg
    )
    CORNER_BL.rotate_n_degrees(
        direction, -1 * outer_deg if direction == "right" else inner_deg
    )
    CORNER_BR.rotate_n_degrees(
        direction, outer_deg if direction == "left" else -1 * inner_deg
    )


# drives rover forward for arc turn at specified speed for specified distance
def arc_turn_drive(direction, radius, dist, drive) -> int:
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
    out_speed = out_cen_speed * (
        math.sqrt(radius ** 2 + R_OUTER * radius + 0.101) / (radius + 0.155)
    )
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


def autonomous() -> None:
    calibrate_all()
    while True:
        move_distance_meters("all", 1)  # this will be changed to move until
        # an obstacle is detected
        tank_with_turn("cw", 90)
        recenter()

