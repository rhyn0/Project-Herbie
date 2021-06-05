import commands

MAX_TURN = 36  # max L / R corner wheel rotation

""" 'manual' for avail commands / command cookbook """
""" 
to add new func / command: 
    - map its verbage to its corresponding function in commands.py in parseCommand
"""


class COMMAND_BOOK:
    def __init__(self):
        pass

    # dumps this info when user enters help cuz I'm dumb and can't remember every command and its field
    def print_message(self):
        print("Available commands")
        print("<> -> optional parameter")
        print("do NOT include <> in actual command")
        print("")
        print("calibrate")
        print("calibrates all wheels")
        print("")
        print("calibrate fr")
        print("calibrates front right wheel")
        print("calibrate br")
        print("calibrates back right wheel")
        print("calibrate fl")
        print("calibrates front left wheel")
        print("calibrate bl")
        print("calibrates back left wheel")
        print("")
        print("forward speed distance")
        print("drives rover straight forward")
        print("speed: 0 - 0.1 m/s")
        print("distance: any value above 0 in m")
        print("")
        print("backward speed distance")
        print("drives rover straight backward")
        print("speed: 0 - 0.1 m/s")
        print("distance: any value above 0 in m")
        print("")
        print("arc turn drive direction turn direction speed <distance>")
        print("drive direction: forward or backward")
        print("turn direction: right or left")
        print("speed: 0 - 0.1 m/s")
        print("optional distance: any value above 0 in m")
        print("")
        print("tank <spin direction> <duration>")
        print("moves the rover like a tank")
        print("spin direction: cw or ccw")
        print("duration: int seconds\n")

        print("recenter")
        print("recenters corner wheels")
        print("")
        print("For even more info see DESCRIPTIONS file")
        print("")

    def parseCommand(self, command):  # try-except commented out for debugging purposes
        # try:
        if command[0] == "calibrate" and len(command) == 1:
            return commands.calibrate_all()

        elif command[0] == "calibrate" and len(command) > 1:
            return commands.calibrate_one(command[1])

        elif command[0] == "arc" and len(command) == 5:
            return commands.turn(command[1], command[2], command[3], command[4])

        
        #elif command[0] == "arc" and len(command) == 6:
        #    return commands.turn(command[4], command[3], command[5], command[2])

        elif command[0] == "recenter":
            return commands.recenter()

        elif command[0] == "forward" and len(command) == 2:
            return commands.forward_with_stop(command[1])

        elif command[0] == "forward":
            return commands.forward(command[1], command[2])

        elif command[0] == "backward" and len(command) == 2:
            return commands.backward_with_stop(command[1])

        elif command[0] == "backward":
            return commands.backward(command[1], command[2])

        elif command[0] == "tank":
            return commands.tank(command[1], command[2])
        elif command[0] == "tankturn":
            return commands.tank_with_turn(command[1], command[2])
        elif command[0] == "encoders":
            return commands.print_encoders()
        elif command[0] == "position":
            # position <wheel_fr> <position>
            return commands.position(command[1], command[2])

        elif command[0] == 'distance':
            # position <wheel_fr> <distance>
            return commands.distance(command[1], command[2])
        elif command[0] == 'meters':
            # position <wheel_fr> <distance>
            return commands.move_distance_meters(command[1], command[2])
        elif command[0] == 'velocity':
            # velocity <wheel_fr> <direction>
            return commands.move_default_velocity(command[1], command[2])
        elif command[0] == "auto":
            return commands.autonomous()
        elif command[0] == 'rotate':
            # rotate <wheel> <direction> <angle>
            return commands.rotate(command[1], command[2], command[3])
        elif command[0] == 'max':
            return commands.rotate_max(command[1], command[2])
        elif command[0] == "stop":
            return commands.kill_all()

        else:
            print("Unrecognized Command")
            return -1
        return 0
        # except:
        # commands.kill_all()
        # print("ERROR")
        # return -1
