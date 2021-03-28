from time import sleep
import sys
from os import path

from roboclaw_3 import Roboclaw

if __name__ == "__main__":
    address = int(sys.argv[1])
    rc = Roboclaw("/dev/ttyS0", 115200)
    rc.Open()

    print(rc.ReadVersion(address))
    print(rc.ReadEncM1(address))
