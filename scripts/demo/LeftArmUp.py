#!/usr/bin/env python

from Maestor import maestor

robot = maestor()

def main():
    setVelocities()
    LeftArmUp()
    LeftArmDown()
    LeftElbowUp()
    LeftElbowDown()
    resetVelocities()

def setVelocities():
    robot.setProperties("LSR LEP", "velocity velocity", "1 1")

def resetVelocities():
    robot.setProperties("LSR LEP", "velocity velocity", ".3 .3")

def LeftArmUp():
    robot.setProperty("LSR", "position", .75)
    robot.waitForJoint("LSR")

def LeftArmDown():
    robot.setProperty("LSR", "position", 0)
    robot.waitForJoint("LSR")

def LeftElbowUp():
    robot.setProperty("LEP", "position", -.6)
    robot.waitForJoint("LEP")
    
def LeftElbowDown():
    robot.setProperty("LEP", "position", 0)
    robot.waitForJoint("LEP")

if __name__ == '__main__':
    main()