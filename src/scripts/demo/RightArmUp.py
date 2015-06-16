#!/usr/bin/env python

from Maestor import maestor

robot = maestor()

def main():
    setVelocities()
    RightArmUp()
    RightArmDown()
    RightElbowUp()
    RightElbowDown()
    resetVelocities()

def setVelocities():
    robot.setProperties("RSR REP", "velocity velocity", "1 1")

def resetVelocities():
    robot.setProperties("RSR REP", "velocity velocity", ".3 .3")

def RightArmUp():
    robot.setProperty("RSR", "position", -.75)
    robot.waitForJoint("RSR")

def RightArmDown():
    robot.setProperty("RSR", "position", 0)
    robot.waitForJoint("RSR")

def RightElbowUp():
    robot.setProperty("REP", "position", -.6)
    robot.waitForJoint("REP")
    
def RightElbowDown():
    robot.setProperty("REP", "position", 0)
    robot.waitForJoint("REP")

if __name__ == '__main__':
    main()