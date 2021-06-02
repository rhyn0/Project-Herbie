# OLD Grover Code

*Keeping for Posterity*

```python
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

def get_register_speed(speed):
    result2 = (0.002 + speed) // 0.0009 
    result2 = int(result2)
    if result2 > 127:
        result2 = 127
    return result2 

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

# drives rover backward for arc turn at specified speed for specified distance
def arc_turn_backward(direction, speed, dist):
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
```
