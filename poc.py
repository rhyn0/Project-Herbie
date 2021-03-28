from roboclaw_3 import Roboclaw
from motor import Motor, CornerMotor
import time
import math

rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()

POC_ADDR = 0x80
QUAD_CR = CornerMotor(rc, POC_ADDR, 2)
QUAD_DR = Motor(rc, POC_ADDR, 1)


QUAD_CR.move_distance(0.1, 0.1)
QUAD_DR.move_distance(0.1, 0.1)
