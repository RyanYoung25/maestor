#!/usr/bin/env python

from Maestor import maestor

robot = maestor()

def main():
    setVelocities()
    bendDown()
    waveArms()
    standUp()
    resetVelocities()

def setVelocities():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "velocity velocity velocity velocity velocity velocity", ".3 .3 .6 .6 .3 .3")
    robot.setProperties("RSP RSR RSY REP LSP LSR LSY LEP NKY", "velocity velocity velocity velocity velocity velocity velocity velocity velocity", "1 1 1 1 1 1 1 1 1")

def resetVelocities():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "velocity velocity velocity velocity velocity velocity", ".3 .3 .3 .3 .3 .3")
    robot.setProperties("RSP RSR RSY REP LSP LSR LSY LEP NKY", "velocity velocity velocity velocity velocity velocity velocity velocity velocity", ".3 .3 .3 .3 .3 .3 .3 .3 .3")

def bendDown():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "position position position position position position", "-0.267199 -0.267199 0.535107 0.535107 -0.267199 -0.267199")
    robot.waitForJoint("RHP")

def waveArms():
    #Raise arms up
    robot.setProperties("RSR LSR RSY LSY", "position position position position", "-1.85 1.85 -1.7 1.7")
    robot.waitForJoint("RSR")
    robot.waitForJoint("LSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("LSY")
    #Loop through waving elbows
    for i in range(0, 3):
        robot.setProperties("REP LEP", "position position", "-1 -1")
        robot.waitForJoint("REP")
        robot.waitForJoint("LEP")
        robot.setProperties("REP LEP", "position position", "0 0")
        robot.waitForJoint("REP")
        robot.waitForJoint("LEP")
    #Lower arms
    robot.setProperties("RSR LSR RSY LSY", "position position position position", "0 0 0 0")
    robot.waitForJoint("RSR")
    robot.waitForJoint("LSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("LSY")

def standUp():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "position position position position position position", "0 0 0 0 0 0")
    robot.waitForJoint("RHP")

if __name__ == '__main__':
    main()
