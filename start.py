from os import path
from . import commands
from . import commandBook


""" what user sees: prompt loop where user can send commands to move rover """
"""
-sit in > loop until q or Q entered
-to see command list type help
-should handle most invalid inputs nicely, but you never know what a user can do
-all commands are NOT case sensitive
"""

# use this repository for reference on history
# https://github.com/danishprakash/dash/blob/master/dash.c


def runHistory():
    fp = open("history.txt", "r")
    lines = ""
    for x in fp:
        lines += x
    # lines = fp.readLines()
    lines = lines.split("\n")
    for i in range(0, len(lines)):
        print(lines[i])

    choice = input("\n <0>: Quit\t<#line>: Execute command\t<-1>: clear history\n")

    if choice.isdigit():
        choice = int(choice)
    else:
        print("Enter a numerical value next time")
        return

    if choice == 0:
        fp.close()
        return
    elif choice == -1:
        fp.close()
        fp = open("history.txt", "w")
        fp.close()
        return
    else:
        command = lines[choice - 1].split(" ")
        command.pop(0)
        commandBook.parseCommand(command)


def addHistory(line):
    fp = open("history.txt", "a+")
    j = 0
    fp.write("%d. " % getHistoryLine())
    while j < len(line):
        if j > 0:
            fp.write(" ")
        fp.write("%s" % line[j])
        j += 1
    fp.write("\n")
    fp.close()


def getHistoryLine():
    file = open("history.txt", "r")
    Counter = 1
    Content = file.read()
    CoList = Content.split("\n")

    for i in CoList:
        if i:
            Counter += 1
    return Counter


def fileInput(file):
    f = open(file, "r")
    for line in f:
        line = line.strip()
        line = line.lower()
        # if(len(command) != 0):
        # think we can skip the above, since EOF won't run and won't format
        # instruction set with blank lines

        command = line.split(" ")
        if command[0] == "calibrate" and len(command) == 1:
            if commandBook.parseCommand(command) == -1:
                print("For more info, enter help")
            continue
        if command[0] == "help":
            commandBook.print_message()
        else:
            if commandBook.parseCommand(command) == -1:
                print("For more info, enter help")
        print("")


def main():
    command = input("> ")
    while command != "Q" and command != "q":
        if path.isfile(command):
            fileInput(command)
            command = input("> ")
            continue
        command = command.strip()
        command = command.lower()
        if len(command) != 0:
            command = command.split()
            if command[0] == "help":
                commandBook.print_message()
            elif command[0] == "history":
                runHistory()
                print("")
                command = input("> ")
                continue
            else:
                addHistory(command)
                if commandBook.parseCommand(command) == -1:
                    print("For more info, enter help")
        print("")
        command = input("> ")

    fp = open("history.txt", "w")
    fp.close()
    commands.kill_all()
    print("bye bye")
    return 0


if __name__ == "__main__":
    main()
