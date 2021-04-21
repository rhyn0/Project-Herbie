from roboclaw_3 import Roboclaw
from motor import *
import time
import math

rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()

POC_ADDR = 128
QUAD_CR = CornerMotor(rc, POC_ADDR, 2)
QUAD_DR = Motor(rc, POC_ADDR, 1)
QUAD_CR.left_most = -1000
QUAD_CR.right_most = 1000
QUAD_CR.center = 0
QUAD_CR.encoders_per_degree = 2000.0 / 80
QUAD_CR.calibrated = True
rc.BackwardM2(POC_ADDR, 30)
time.sleep(1)
rc.BackwardM2(POC_ADDR, 0)
print(rc.ReadEncM2(POC_ADDR))
print(QUAD_CR.left_most)
# QUAD_CR.go_to_left_most()
# QUAD_CR.move_is_complete()
rc.SpeedAccelDeccelPositionM2(POC_ADDR,300,500,300,800,1)
i = 0
while i < 20:
    print("Ispeed", rc.ReadISpeedM2(POC_ADDR))
    print("Enc Speed", rc.ReadSpeedM2(POC_ADDR))
    time.sleep(1)
    i += 1