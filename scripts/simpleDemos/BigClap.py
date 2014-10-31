#! /usr/bin/env python
from Maestor import maestor

robot = maestor()

def crouch():
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ")

def prepareToClap():
    robot.setProperties("RSP RSR RSY REP", "position position position position", "-1.46 0 .82 -1.42")
    robot.setProperties("LSP LSR LSY LEP", "position position position position", "-1.46 0 -.82 -1.42")
    robot.waitForJoint("RSP")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("REP")
    robot.waitForJoint("LSP")
    robot.waitForJoint("LSR")
    robot.waitForJoint("LSY")
    robot.waitForJoint("LEP")

def clap():
    for i in xrange(0, 5):
        robot.setProperty("RSY", "position", .92)
        robot.setProperty("LSY", "position", -.92)
        robot.waitForJoint("RSP")
        robot.waitForJoint("LSY")
        robot.setProperty("RSY", "position", .82)
        robot.setProperty("LSY", "position", -.82)
        robot.waitForJoint("RSP")
        robot.waitForJoint("LSY")

def twistAndClap():
    for i in xrange(0, 3):
        robot.setProperty("WST", "position", -.45)
        robot.waitForJoint("WST")
        clap()
        robot.setProperty("WST", "position", .45)
        robot.waitForJoint("WST")
        clap()
    robot.setProperty("WST", "position", 0)
    robot.waitForJoint("WST")
    

def moveArmsDown():
    robot.setProperties("RSP RSR RSY REP", "position position position position", "0 0 0 0")
    robot.setProperties("LSP LSR LSY LEP", "position position position position", "0 0 0 0")
    robot.waitForJoint("RSP")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("REP")

def stand():
    robot.setProperties("RFZ LFZ", "position position", "-.56 -.56")
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ")

def main():
    crouch()
    prepareToClap()
    twistAndClap()
    moveArmsDown()
    stand()

if __name__ == '__main__':
    main()