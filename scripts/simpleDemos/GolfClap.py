#! /usr/bin/env python
from Maestor import maestor

robot = maestor()

def crouch():
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ")

def prepareToClap():
    robot.setProperties("RSP RSR RSY REP", "position position position position", "-1.16 0.0 1.30 -1.48")
    robot.setProperties("LWY LWY", "position velocity", "2.56 2")
    robot.setProperties("LSP LSR LSY LEP", "position position position position", "-1.32 0 -1.62 -1.05")
    robot.waitForJoint("RSP")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("REP")
    robot.waitForJoint("LSP")
    robot.waitForJoint("LSR")
    robot.waitForJoint("LSY")
    robot.waitForJoint("LEP")
    robot.waitForJoint("LWY")

def clap():
    for i in xrange(0, 5):
        robot.setProperty("RSP", "position", -1.06)
        robot.waitForJoint("RSP")
        robot.setProperty("RSP", "position", -1.16)
        robot.waitForJoint("RSP")
    

def moveArmsDown():
    robot.setProperties("RSP RSR RSY REP", "position position position position", "0 0 0 0")
    robot.setProperties("LSP LSR LSY LEP LWY", "position position position position position", "0 0 0 0 0")
    robot.setProperty("LWY", "velocity", 1)
    robot.waitForJoint("RSP")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("REP")
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
    prepareToClap()
    clap()
    moveArmsDown()
    stand()

if __name__ == '__main__':
    main()