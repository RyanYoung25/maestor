#! /usr/bin/env python
from Maestor import maestor

robot = maestor()

def crouch():
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ")

def WaxOn():
    robot.setProperties("RSP RSR RSY REP RWY", "position position position position position", "-.98 0 .81 -1.24 1.44")
    robot.waitForJoint("RSP")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("REP")
    robot.waitForJoint("RWY")

    robot.setProperty("RSY", "position", 0)
    robot.waitForJoint("RSY")

    robot.setProperties("RSP RSR RSY REP RWY", "position position position position position", "0 0 0 0 0")
    robot.waitForJoint("RSP")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("REP")
    robot.waitForJoint("RWY")


def WaxOff():
    robot.setProperties("LSP LSR LSY LEP LWY", "position position position position position", "-.98 0 -.81 -1.24 -1.44")
    robot.waitForJoint("LSP")
    robot.waitForJoint("LSR")
    robot.waitForJoint("LSY")
    robot.waitForJoint("LEP")
    robot.waitForJoint("LWY")

    robot.setProperty("LSY", "position", 0)
    robot.waitForJoint("LSY")

    robot.setProperties("LSP LSR LSY LEP LWY", "position position position position position", "0 0 0 0 0")
    robot.waitForJoint("LSP")
    robot.waitForJoint("LSR")
    robot.waitForJoint("LSY")
    robot.waitForJoint("LEP")
    robot.waitForJoint("LWY")

def stand():
    robot.setProperties("RFZ LFZ", "position position", "-.56 -.56")
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ")

def main():
    crouch()
    for i in range(0, 3):
        WaxOn()
        WaxOff()    
    stand()

if __name__ == '__main__':
    main()