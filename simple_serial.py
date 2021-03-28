from roboclaw_3 import Roboclaw
from time import sleep

address = 0x80
rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()

while True:
    print("M1")
    rc.ForwardM1(address, 120)
    sleep(2)
    rc.ForwardM1(address, 0)
    sleep(1)
    print("M2")
    rc.ForwardM2(address, 120)
    sleep(2)
    rc.ForwardM2(address, 0)
    sleep(1)
