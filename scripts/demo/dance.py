#!/usr/bin/env python

from Maestor import maestor

robot = maestor()

def main():
    setVelocities()
    bendDown()
    for i in range(0, 2):
        wiggle()
    standUp()
    resetVelocities()

def setVelocities():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "velocity velocity velocity velocity velocity velocity", ".3 .3 .6 .6 .3 .3")
    robot.setProperties("RHR LHR RAR LAR", "velocity velocity velocity velocity", ".1 .1 .1 .1")
    robot.setProperties("RSP RSR RSY REP NKY", "velocity velocity velocity velocity velocity", "1 1 1 1 1")

def resetVelocities():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "velocity velocity velocity velocity velocity velocity", ".3 .3 .3 .3 .3 .3")
    robot.setProperties("RHR LHR RAR LAR", "velocity velocity velocity velocity", ".3 .3 .3 .3")
    robot.setProperties("RSP RSR RSY REP NKY", "velocity velocity velocity velocity velocity", ".3 .3 .3 .3 .3")

def bendDown():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "position position position position position position", "-0.267199 -0.267199 0.535107 0.535107 -0.267199 -0.267199")
    robot.waitForJoint("RHP")

def wiggle():
    #Wiggle left
    robot.setProperties("RAR LAR RHR LHR", "position position position position", ".05 .05 -.05 -.05")
    robot.waitForJoint("RAR")
    robot.waitForJoint("RHR")
    robot.waitForJoint("LAR")
    robot.waitForJoint("LHR")
    #Wiggle right
    robot.setProperties("RAR LAR RHR LHR", "position position position position", "-.05 -.05 .05 .05")
    robot.waitForJoint("RAR")
    robot.waitForJoint("RHR")
    robot.waitForJoint("LAR")
    robot.waitForJoint("LHR")

def standUp():
    #straight 
    robot.setProperties("RAR LAR RHR LHR", "position position position position", "0 0 0 0")
    robot.waitForJoint("RAR")
    robot.waitForJoint("RHR")
    robot.waitForJoint("LAR")
    robot.waitForJoint("LHR")
    #stand
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "position position position position position position", "0 0 0 0 0 0")
    robot.waitForJoint("RHP")

if __name__ == '__main__':
    main()
