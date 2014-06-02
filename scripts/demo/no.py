#!/usr/bin/env python

from Maestor import maestor

robot = maestor()

def main():
    setVelocities()
    KnodRight()
    KnodLeft()
    KnodRight()
    KnodLeft()
    KnodRight()
    KnodLeft()
    headZero()
    resetVelocities()

def setVelocities():
    robot.setProperties("NKY", "velocity", "2")

def resetVelocities():
    robot.setProperties("NKY", "velocity", ".3")

def KnodLeft():
    robot.setProperties("NKY", "position", "-.65")
    robot.waitForJoint("NKY")

def KnodRight():
    robot.setProperties("NKY", "position", ".65")
    robot.waitForJoint("NKY")

def headZero():
    robot.setProperties("NKY", "position", "0")
    robot.waitForJoint("NKY")
    


if __name__ == '__main__':
    main()
