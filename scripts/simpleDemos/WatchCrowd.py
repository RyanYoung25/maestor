#! /usr/bin/env python
from Maestor import maestor

robot = maestor()

def crouch():
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ")

def moveArmUp():
    robot.setProperties("RSP RSR RSY REP", "position position position position", "-1.7 -.33 .93 -1.73")
    robot.waitForJoint("RSP")
    robot.waitForJoint("RSR")
    robot.waitForJoint("RSY")
    robot.waitForJoint("REP")

def twist():
    for i in xrange(0, 3):
        robot.setProperty("WST", "position", -.45)
        robot.waitForJoint("WST")
        robot.setProperty("WST", "position", .45)
        robot.waitForJoint("WST")
    robot.setProperty("WST", "position", 0)
    robot.waitForJoint("WST")

def moveArmDown():
    robot.setProperties("RSP RSR RSY REP", "position position position position", "0 0 0 0")
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
    moveArmUp()
    twist()
    moveArmDown()
    stand()

if __name__ == '__main__':
    main()