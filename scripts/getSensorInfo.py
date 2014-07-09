#! /usr/bin/env python

from Maestor import maestor
import time

def main():
    robot = maestor()

    robot.command("InitializeSensors", "")

    for i in range(0, 1000):
        print "LWT " + robot.getProperties("LWT", "m_x m_y f_z")
        print "RWT " + robot.getProperties("RWT", "m_x m_y f_z")
        time.sleep(.25)



if __name__ == '__main__':
    main()
