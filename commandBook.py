import commands

MAX_TURN = 36  # max L / R corner wheel rotation

""" 'manual' for avail commands / command cookbook """
""" 
to add new func / command: 
    - map its verbage to its corresponding function in commands.py in parseCommand
"""

# dumps this info when user enters help cuz I'm dumb and can't remember every command and its field
def print_message():
    with open("DESCRIPTIONS.txt", "r") as f:
        for line in f:
            print(line, end='')


def command_calibrate(tokn: list):
    """Handles all versions of a calibrate command
    
    Returns
    ----
    -1 
        when input command does not conform to any template
    0 
        otherwise
    """
    if type(tokn) is not list:
        print("Programmer Error: How did you do this?")
        return -1
    if len(tokn) == 0:  # assume a calibrate all
        return commands.calibrate_all()
    for m in tokn:
        commands.calibrate_one(m)
    return 0


def command_backward(tokn: list):
    """Handles all versions of backward commands
    Types: {'backward speed', DEPRECATED: 'backward speed distance'}
    
    Returns
    ----
    -1 
        when input command does not conform to any template
    0 
        otherwise
    """
    if type(tokn) is not list:
        print("Programmer Error: How did you do this?")
        return -1
    if len(tokn) == 1:
        # backward speed
        return commands.backward_with_stop(tokn[0])
    elif len(tokn) == 2:
        # backward speed distance
        # DEPRECATED! Uses time based command!
        return commands.backward(tokn[0], tokn[1])
    else:
        return -1


def command_forward(tokn: list):
    """Handles all versions of forward commands. 
    Types: {'forward speed', DEPRECATED: 'forward speed distance'}
    
    Returns
    ----
    -1 
        when input command does not conform to any template
    0 
        otherwise
    """
    if type(tokn) is not list:
        print("Programmer Error: How did you do this?")
        return -1
    if len(tokn) == 1:
        # forward speed
        return commands.forward_with_stop(tokn[0])
    elif len(tokn) == 2:
        # forward speed distance
        # DEPRECATED! Uses time based command!
        return commands.forward(tokn[0], tokn[1])
    else:
        return -1


def parseCommand(command):  # try-except commented out for debugging purposes
    # try:
    if command[0] == "calibrate":
        return command_calibrate(command[1:])

    elif command[0] == "arc" and len(command) == 5:
        return commands.turn(command[1], command[2], command[3], command[4])

    elif command[0] == "recenter":
        return commands.recenter()

    elif command[0] == "forward":
        return command_forward(command[1:])

    elif command[0] == "backward":
        return command_backward(command[1:])

    elif command[0] == "tank":
        # format: tank direction degrees. direction: {'cw', 'ccw'}
        # Note: This command does NOT set the wheels to proper orientation
        # DEPRECATED?: Uses time based command, needs another math proof.
        if len(command) != 3:
            return -1
        return commands.tank(command[1].lower(), command[2])

    elif command[0] == "tankturn":
        if len(command) != 3:
            return -1
        return commands.tank_with_turn(command[1], command[2])

    elif command[0] == "encoders":
        return commands.print_encoders()

    elif command[0] == "position":
        # format: position motor-name encoder-value
        if len(command) != 3:
            return -1
        return commands.position(command[1], command[2])

    elif command[0] == "meters":
        # format: meters motor-name/all distance
        if len(command) != 3:
            return -1
        return commands.move_distance_meters(command[1], command[2])

    elif command[0] == "velocity":
        # format: velocity motor-name/all direction
        if len(command) != 3:
            return -1
        return commands.move_default_velocity(command[1], command[2])

    # elif command[0] == "auto":
    #     # format: auto
    #     return commands.autonomous()

    elif command[0] == "rotate":
        # format: rotate wheel-name direction angle
        if len(command) != 4:
            return -1
        return commands.rotate(command[1], command[2], command[3])

    elif command[0] == "max":
        # format: max wheel-name direction
        if len(command) != 3:
            return -1
        return commands.rotate_max(command[1], command[2])

    elif command[0] == "stop":
        return commands.kill_all()

    else:
        print("Unrecognized Command")
        return -1

