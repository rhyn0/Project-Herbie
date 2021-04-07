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
QUAD_CR.encoders_per_degree = 2350 / 90 #testing showed it was about 8300 for a full revolution
QUAD_CR.calibrated = True

def calib_test():
    tup = QUAD_CR.calibrate()
    print(tup)

calib_test()
