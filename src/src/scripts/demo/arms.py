#!/usr/bin/env python

from Maestor import maestor
import time

robot = maestor()

def main():
    setVelocities()
    ArmUp()
    ArmDown()
    ElbowUp()
    ElbowDown()
    twist()
    resetVelocities()

def setVelocities():
    robot.setProperties("LSR LEP RSR REP WST", "velocity velocity velocity velocity velocity", ".5 .5 .5 .5 .5")

def resetVelocities():
    robot.setProperties("LSR LEP RSR REP WST", "velocity velocity velocity velocity velocity", ".3 .3 .3 .3 .3")

def ArmUp():
    robot.setProperties("LSR RSR", "position position", ".75 -.75")
    robot.waitForJoint("LSR")
    robot.waitForJoint("RSR")

def ArmDown():
    robot.setProperties("LSR RSR", "position position", "0 0")
    robot.waitForJoint("LSR")
    robot.waitForJoint("RSR")

def ElbowUp():
    robot.setProperties("LEP REP", "position position", "-.6 -.6")
    robot.waitForJoint("LEP")
    robot.waitForJoint("REP")
    
def ElbowDown():
    robot.setProperties("LEP REP", "position position", "0 0")
    robot.waitForJoint("LEP")
    robot.waitForJoint("REP")

def twist():
    robot.setProperty("WST", "position", -.25)
    robot.waitForJoint("WST")
    time.sleep(.25)
    robot.setProperty("WST", "position", .25)
    robot.waitForJoint("WST")
    time.sleep(.25)
    robot.setProperty("WST", "position", 0)
    robot.waitForJoint("WST")


if __name__ == '__main__':
    main()
