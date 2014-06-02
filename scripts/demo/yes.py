#!/usr/bin/env python

from Maestor import maestor

robot = maestor()

def main():
    setVelocities()
    KnodDown()
    KnodUp()
    KnodDown()
    KnodUp()
    KnodDown()
    KnodUp()
    headZero()
    resetVelocities()

def setVelocities():
    robot.setProperties("NK1 NK2", "velocity velocity", "2 2")

def resetVelocities():
    robot.setProperties("NK1 NK2", "velocity velocity", ".3 .3")

def KnodUp():
    robot.setProperties("NK1 NK2", "position position", "4 4")
    robot.waitForJoint("NK1")

def KnodDown():
    robot.setProperties("NK1 NK2", "position position", "-4 -4")
    robot.waitForJoint("NK1")

def headZero():
    robot.setProperties("NK1 NK2", "position position", "0 0")
    robot.waitForJoint("NK1")
    


if __name__ == '__main__':
    main()