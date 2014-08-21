#! /usr/bin/env python

from Maestor import maestor 
import time

robot = maestor()

def main():
    robot.command("Disable", "RSR")
    while True:
        Rpos = float(robot.getProperties("RSR", "position"))
        Lpos = -1 * Rpos
        robot.setProperty("LSR", "position", Lpos)
        time.sleep(.15)

if __name__ == "__main__":
    main()
