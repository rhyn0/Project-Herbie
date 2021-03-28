from roboclaw_3 import Roboclaw

def main():

    RC1 = 0
    RC2 = 1
    RC3 = 2
    RC4 = 3
    RC5 = 4

    rc = Roboclaw("/dev/ttyS0",115200)
    rc.Open()

    address = [0x80,0x81,0x82,0x83,0x84]

    rc.ForwardM1(address[RC1], 0)
    rc.ForwardM2(address[RC1], 0)
    rc.ForwardM1(address[RC2], 0)
    rc.ForwardM2(address[RC2], 0)
    rc.ForwardM1(address[RC3], 0)
    rc.ForwardM2(address[RC3], 0)
    rc.ForwardM1(address[RC4], 0)
    rc.ForwardM2(address[RC4], 0)
    rc.ForwardM1(address[RC5], 0)
    rc.ForwardM2(address[RC5], 0)
    
    
if __name__ == "__main__":
    main()


