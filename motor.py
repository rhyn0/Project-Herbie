import time
CALIBRATION_TIME = 9
CALIBRATION_SPEED = 10
SLOWER_CALIBRATION_SPEED = 5

MAX_CORNER_ENC = 1550
INVALID_ENC = 1600

GOTO_SPEED = 1750
GOTO_FR = 4542
ACCEL = DECCEL = 1000

ANGULAR_RANGE = 72
class Motor:
    def __init__(self, rc, rc_addr, motor_ndx):
        self.rc = rc
        self.rc_addr = rc_addr
        self.motor_ndx = motor_ndx

    def go_to_position(self, position):
        if self.motor_ndx == 1:
            self.rc.SpeedAccelDeccelPositionM1(self.rc_addr,ACCEL,GOTO_SPEED,DECCEL,position,1)
        else:
            self.rc.SpeedAccelDeccelPositionM2(self.rc_addr,ACCEL,GOTO_SPEED,DECCEL,position,1)

    def move_distance(self, distance, speed=GOTO_SPEED):
        # if distance negative, make speed negative 
        if distance < 0:
            speed *= -1
            distance *= -1
        if self.motor_ndx == 1:
            self.rc.SpeedDistanceM1(self.rc_addr,speed,distance,1)
        else:
            self.rc.SpeedDistanceM2(self.rc_addr,speed,distance,1)

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
        self.set_motor_register_speed('forward', 0)
        self.set_motor_register_speed('backward', 0)

    def set_motor_speed(self, direction, speed):
        """sets the speed of given a motor macro [WHEEL_FR, etc.] a direction [forward, backward] and a speed (m/s)"""
        reg_speed = self.calculate_reg_speed(speed)
        self.set_motor_register_speed(direction,reg_speed)

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
        self.center_raw = 0
        self.total = 0
        self.calibrated = False
        self.encoders_per_degree = None   # ratio of encoder values 

    def go_to_position(self, position):
        ''' overrided to ensure position doesn't go out of bounds '''
        if position > self.right_most:
            print("position (%d) out of range, using rightmost instead" % position)
            position = self.right_most
        if position < self.left_most:
            print("position (%d) out of range, using leftmost instead" % position)
            position = self.left_most

        if self.motor_ndx == 1:
            self.rc.SpeedAccelDeccelPositionM1(self.rc_addr,ACCEL,GOTO_SPEED,DECCEL,position,1)
        else:
            self.rc.SpeedAccelDeccelPositionM2(self.rc_addr,ACCEL,GOTO_SPEED,DECCEL,position,1)
    
    def go_to_left_most(self):
        if not self.calibrated:
            print("Cannot perform action, motor is not calibrated!")
            return
        self.go_to_position(self.left_most)
        
    def go_to_right_most(self):
        if not self.calibrated:
            print("Cannot perform action, motor is not calibrated!")
            return
        self.go_to_position(self.right_most)
    
    def go_to_center(self):
        if not self.calibrated:
            print("Cannot perform action, motor is not calibrated!")
            return
        self.go_to_position(self.center)
    
    def rotate_n_degrees(self, direction, angle):
        if not self.calibrated:
            print("Cannot perform action, motor is not calibrated!")
            return
        
        encoder_distance = int(self.encoders_per_degree * angle)
        current_position = self.encoder_value()
        
        if direction == "right":
            target_position = current_position + encoder_distance
        else:
            target_position = current_position - encoder_distance

        print("Rotating %s for %d degrees" % (direction, angle))
        self.go_to_position(target_position)



    def calibrate(self):
        flag = 0
        left_most = 0
        right_most = 0
        centered = 0
        

        # turn to left-most position, store left-most encoder value in global var
        prev_encoder = self.encoder_value()
        print(prev_encoder)
        self.set_motor_register_speed("backward", CALIBRATION_SPEED)
        start = time.time()
        while time.time() - start < CALIBRATION_TIME:
            curr_encoder = self.encoder_value()
            print(curr_encoder)
            if curr_encoder - prev_encoder < 3:
                print("breaking on encoder condition")
                break
            prev_encoder = curr_encoder
            print(prev_encoder)
        #time.sleep(CALIBRATION_TIME) # trying to avoid using sleep since that will mess with the point of having multithread
        self.stop()
        left_most = self.encoder_value()
        self.left_most = left_most
        # turn to right-most position, adding 1550 to running total if the encoder vals wrap
        self.set_motor_register_speed("forward", CALIBRATION_SPEED)
        # 11/17/20 I don't think this while loop is necessary
        start = time.time()
        while time.time() - start < CALIBRATION_TIME:
            if (
                self.encoder_value() >= 1500 and flag == 0
            ):  # not using 1550 cuz the enc values change fast, so giving it wide range
                centered += MAX_CORNER_ENC
                flag = 1

        self.stop()
        right_most = self.encoder_value()     # store right-most encoder val, calculate center

        self.right_most = right_most
        

        self.center = (right_most + left_most) // 2
        
        self.calibrated = True
        # go to center position
        self.go_to_center()

        # calculate encoder to angle value
        encoder_range = abs(self.right_most - self.left_most)
        self.encoders_per_degree = encoder_range / ANGULAR_RANGE
        return (left_most, self.center, right_most)
