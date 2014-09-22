#!/usr/bin/env python

from Maestor import maestor

robot = maestor()

def main():
    setVelocities()
    bendDown()
    doTheRobot()
    standUp()
    resetVelocities()

def setVelocities():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "velocity velocity velocity velocity velocity velocity", ".3 .3 .6 .6 .3 .3")
    robot.setProperties("RSP RSR RSY REP NKY", "velocity velocity velocity velocity velocity", "1 1 1 1 1")

def resetVelocities():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "velocity velocity velocity velocity velocity velocity", ".3 .3 .3 .3 .3 .3")
    robot.setProperties("RSP RSR RSY REP NKY", "velocity velocity velocity velocity velocity", ".3 .3 .3 .3 .3")

def bendDown():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "position position position position position position", "-0.267199 -0.267199 0.535107 0.535107 -0.267199 -0.267199")
    robot.waitForJoint("RHP")

def doTheRobot():
#    robot.setProperty("NKY", "position", -.65)
 #   robot.waitForJoint("NKY")
    robot.setProperties("RSY RSR", "position position", "1.57 -1.35")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    for i in range(0, 2):
        robot.setProperty("REP", "position", -1.7)
        robot.waitForJoint("REP")
        robot.setProperty("REP", "position", 0)
        robot.waitForJoint("REP")
    robot.setProperties("RSY RSR NKY", "position position position", "0 0 0")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
  #  robot.waitForJoint("NKY")


def standUp():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "position position position position position position", "0 0 0 0 0 0")
    robot.waitForJoint("RHP")

if __name__ == '__main__':
    main()
