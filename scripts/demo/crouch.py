#!/usr/bin/env python

from Maestor import maestor
import time

robot = maestor()

def main():
    setVelocities()
    bendDown()
    time.sleep(3)
    standUp()
    resetVelocities()

def setVelocities():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "velocity velocity velocity velocity velocity velocity", ".3 .3 .6 .6 .3 .3")

def resetVelocities():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "velocity velocity velocity velocity velocity velocity", ".3 .3 .3 .3 .3 .3")

def bendDown():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "position position position position position position", "-0.267199 -0.267199 0.535107 0.535107 -0.267199 -0.267199")
    robot.waitForJoint("RHP")

def standUp():
    robot.setProperties("RHP LHP RKP LKP RAP LAP", "position position position position position position", "0 0 0 0 0 0")
    robot.waitForJoint("RHP")

if __name__ == '__main__':
    main()
