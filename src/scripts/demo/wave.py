#!/usr/bin/env python

from Maestor import maestor

robot = maestor()

def main():
    setVelocities()
    wave()
    resetVelocities()

def setVelocities():
    robot.setProperties("RSP RSR RSY REP NKY", "velocity velocity velocity velocity velocity", "1 1 1 1 .5")

def resetVelocities():
    robot.setProperties("RSP RSR RSY REP NKY", "velocity velocity velocity velocity velocity", ".3 .3 .3 .3 .3")

def wave():
    robot.setProperties("RSY RSR", "position position", "-1.57 -1.35")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    for i in range(0, 3):
        robot.setProperty("REP", "position", -1.7)
        robot.waitForJoint("REP")
        robot.setProperty("REP", "position", -.7)
        robot.waitForJoint("REP")
    robot.setProperties("RSY RSR REP", "position position position", "0 0 0")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("REP")



if __name__ == '__main__':
    main()
