from motor import Motor, CornerMotor
import time
from roboclaw_3 import Roboclaw

rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()

ADDR = 128
CR = CornerMotor(rc, ADDR, 2)
CR.left_most = -1000
CR.right_most = 300
CR.center = -350
CR.calibrated = True
CR.encoders_per_degree = 1300.0 / 30
CR.rotate_n_degrees("left", 5)
while not CR.move_is_complete():
    continue
print("done")
