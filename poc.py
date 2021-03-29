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
QUAD_CR.encoders_per_degree = 8300 / 360 #testing showed it was about 8300 for a full revolution
QUAD_CR.calibrated = True
while True:
    QUAD_CR.rotate_n_degrees("right", 45)
    QUAD_CR.rotate_n_degrees("left", 45)


