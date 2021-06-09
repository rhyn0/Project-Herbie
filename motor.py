import time
import contextlib

CALIBRATION_TIME = 9
CALIBRATION_SPEED = 25
SLOWER_CALIBRATION_SPEED = 5

MAX_CORNER_ENC = 1550
INVALID_ENC = 1600

GOTO_SPEED = 1750
GOTO_FR = 4542
ACCEL = DECCEL = 1000
ANGULAR_RANGE = 90  # was 72 for Herbie Mk1


class Motor:
    def __init__(self, rc, rc_addr, motor_ndx):
        self.rc = rc
        self.rc_addr = rc_addr
        self.motor_ndx = motor_ndx

    def go_to_position(self, position):
        if self.motor_ndx == 1:
            self.rc.SpeedAccelDeccelPositionM1(
                self.rc_addr, ACCEL, GOTO_SPEED, DECCEL, position, 1
            )
        else:
            self.rc.SpeedAccelDeccelPositionM2(
                self.rc_addr, ACCEL, GOTO_SPEED, DECCEL, position, 1
            )

    def move_distance(self, distance, speed=GOTO_SPEED):
        # if distance negative, make speed negative
        if distance < 0:
            speed *= -1
            distance *= -1
        if self.motor_ndx == 1:
            self.rc.SpeedDistanceM1(self.rc_addr, speed, distance, 1)
        else:
            self.rc.SpeedDistanceM2(self.rc_addr, speed, distance, 1)

    """# NOTE - untested as of 2/17/2021
    def move_velocity(self, direction, speed=GOTO_SPEED):
        # if direction negative, make speed negative
        if direction < 0:
            speed *= -1
        if self.motor_ndx == 1:
            self.rc.SpeedM1(self.rc_addr,speed)
        else:
            self.rc.SpeedM2(self.rc_addr,speed)"""

    def encoder_value(self):
        if self.motor_ndx == 1:
            response = self.rc.ReadEncM1(self.rc_addr)
        else:
            response = self.rc.ReadEncM2(self.rc_addr)

        return response[1]

    def set_encoder_value(self, value):
        if self.motor_ndx == 1:
            response = self.rc.SetEncM1(self.rc_addr, value)
        else:
            response = self.rc.SetEncM2(self.rc_addr, value)

        return response[1]

    def stop(self):
        self.set_motor_register_speed("forward", 0)
        self.set_motor_register_speed("backward", 0)

    def set_motor_speed(self, direction, speed):
        """sets the speed of given a motor macro [WHEEL_FR, etc.] a direction [forward, backward] and a speed (m/s)"""
        reg_speed = self.calculate_reg_speed(speed)
        self.set_motor_register_speed(direction, reg_speed)

    def set_motor_register_speed(self, direction, reg_speed):
        if reg_speed > 80:
            print("WARNING: speed to high %d" % reg_speed)
            return
        if direction == "forward":
            if self.motor_ndx == 1:
                self.rc.ForwardM1(self.rc_addr, reg_speed)
            else:
                self.rc.ForwardM2(self.rc_addr, reg_speed)
        else:
            if self.motor_ndx == 1:
                self.rc.BackwardM1(self.rc_addr, reg_speed)
            else:
                self.rc.BackwardM2(self.rc_addr, reg_speed)

    def move_is_complete(self):
        buffer = self.rc.ReadBuffers(self.rc_addr)
        if buffer[self.motor_ndx] == 128:
            return True
        return False

    @staticmethod
    def calculate_reg_speed(speed):
        result2 = (0.002 + speed) // 0.0009  # based on graph velocity formula for m/s
        result2 = int(result2)
        if result2 > 127:
            result2 = 127
        return result2  # velo = 0.0009(reg value) - 0.002


class CornerMotor(Motor):
    def __init__(self, rc, rc_addr, motor_ndx):
        super().__init__(rc, rc_addr, motor_ndx)
        self.center = 0
        self.left_most = 0
        self.right_most = 0
        self.calibrated = False
        self.encoders_per_degree = None  # ratio of encoder values

    def __repr__(self):
        return "Addr: {}, left: {}, right: {}, calibrated?: {}".format(
            self.rc_addr, self.left_most, self.right_most, self.calibrated
        )

    def go_to_position(self, position):
        """Overrided to ensure position doesn't go out of bounds
        """
        if not self.calibrated:
            print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
            return -1
        if position > self.right_most:
            print("position (%d) out of range, using rightmost instead" % position)
            position = self.right_most
        if position < self.left_most:
            print("position (%d) out of range, using leftmost instead" % position)
            position = self.left_most

        if self.motor_ndx == 1:
            self.rc.SpeedAccelDeccelPositionM1(
                self.rc_addr, ACCEL, GOTO_SPEED, DECCEL, position, 1
            )
        else:
            self.rc.SpeedAccelDeccelPositionM2(
                self.rc_addr, ACCEL, GOTO_SPEED, DECCEL, position, 1
            )

    def go_to_left_most(self) -> int:
        if not self.calibrated:
            print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
            return -1
        self.go_to_position(self.left_most)
        return 0

    def go_to_right_most(self) -> int:
        if not self.calibrated:
            print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
            return -1
        self.go_to_position(self.right_most)
        return 0

    def go_to_center(self) -> int:
        if not self.calibrated:
            print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
            return -1
        self.go_to_position(self.center)
        return 0

    def rotate_n_degrees(self, direction, angle) -> int:
        if not self.calibrated:
            print(f"Cannot perform action, motor RC-{self.rc_addr} is not calibrated!")
            return -1
        direction = direction.lower()
        encoder_distance = int(self.encoders_per_degree * angle)
        current_position = self.encoder_value()

        if direction == "right":
            target_position = current_position + encoder_distance
        elif direction == "left":
            target_position = current_position - encoder_distance
        else:
            print(f"Direction {direction} is not a valid way of turning")
            return -1
        print(f"Rotating {direction} for {angle} degrees")
        self.go_to_position(target_position)
        return 0

    def calibrate(self, lock=contextlib.nullcontext()):
        """Calibrates a wheel(s), if lock is a threading.Lock will allow
        for multiple wheel calibration in almost multithreaded fashion.
        If only one wheel is being calibrated, lock is an optional argument
        """
        left_most = 0
        right_most = 0
        runs = 0
        while (
            right_most - left_most
        ) < 1800 and runs < 3:  # max range is ~2000, run until this is about right
            # turn to left-most position, store left-most encoder value in global var
            with lock:
                prev_encoder = self.encoder_value()
                self.set_motor_register_speed("backward", CALIBRATION_SPEED)
                start = time.time()
                time.sleep(0.1)
                i = 0
                while time.time() - start < CALIBRATION_TIME:
                    time.sleep(0.05)
                    curr_encoder = self.encoder_value()
                    if abs(curr_encoder - prev_encoder) < 3:
                        # print(self, "breaking on encoder condition")
                        i += 1
                    if i == 3:
                        break
                    prev_encoder = curr_encoder
                self.stop()
                self.move_distance(80)
                while not self.move_is_complete():
                    pass
                self.stop()
                prev_encoder = self.encoder_value()
                self.set_motor_register_speed("backward", CALIBRATION_SPEED)
                start = time.time()
                while time.time() - start < CALIBRATION_TIME:
                    time.sleep(0.05)
                    curr_encoder = self.encoder_value()
                    if abs(curr_encoder - prev_encoder) < 3:
                        # print(self, "breaking on encoder condition, second time")
                        break
                    prev_encoder = curr_encoder
                self.stop()
                left_most = self.encoder_value()
            time.sleep(2)
            prev_encoder = left_most
            with lock:
                self.set_motor_register_speed("forward", CALIBRATION_SPEED)
                start = time.time()
                time.sleep(0.1)
                while time.time() - start < CALIBRATION_TIME:
                    time.sleep(0.05)
                    curr_encoder = self.encoder_value()
                    if abs(curr_encoder - prev_encoder) < 3:
                        # print(self, "breaking on encoder condition, forward")
                        break
                    prev_encoder = curr_encoder
                    # print(prev_encoder)

                self.stop()
                right_most = (
                    self.encoder_value()
                )  # store right-most encoder val, calculate center
            runs += 1
            # print("left, right", left_most, right_most, "DIFFERENCE: ", right_most - left_most)

        # edit range of motion so arms don't hit physical stops in most cases
        self.left_most = left_most + 50
        self.right_most = right_most - 50

        self.center = (self.right_most + self.left_most) // 2

        self.calibrated = True
        # go to center position
        with lock:
            self.go_to_position(self.center)
            while not self.move_is_complete():
                pass
            self.stop()

        # calculate encoder to angle value
        encoder_range = abs(self.right_most - self.left_most)
        self.encoders_per_degree = encoder_range / ANGULAR_RANGE
        return (self.left_most, self.center, self.right_most)
