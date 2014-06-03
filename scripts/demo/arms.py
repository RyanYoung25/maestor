#!/usr/bin/env python

from Maestor import maestor

robot = maestor()

def main():
    setVelocities()
    ArmUp()
    ArmDown()
    ElbowUp()
    ElbowDown()
    resetVelocities()

def setVelocities():
    robot.setProperties("LSR LEP RSR REP", "velocity velocity velocity velocity", "1 1 1 1")

def resetVelocities():
    robot.setProperties("LSR LEP RSR REP", "velocity velocity velocity velocity", ".3 .3 .3 .3")

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

if __name__ == '__main__':
    main()