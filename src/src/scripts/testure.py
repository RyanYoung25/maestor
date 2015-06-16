#! /usr/bin/env python
from Maestor import maestor

robot = maestor()
#initialization and enabling. 
#skipping the initialization because homing is currently not perfect. 
#The testure gesture in python
robot.setProperties("RSR RSR", "velocity position", ".50 -1.35")
robot.waitForJoint("RSR")

robot.setProperties("RSY RSY", "velocity position", ".50 1.57")
robot.waitForJoint("RSY")
robot.setProperty("REP", "velocity", .5)

for i in range(0, 5):
    robot.setProperty("REP", "position", -1.7)
    robot.waitForJoint("REP")
    robot.setProperty("REP", "position", 0)
    robot.waitForJoint("REP")

robot.setProperties("RSY RSY", "velocity position", ".5 -1.57")
robot.waitForJoint("RSY")

for i in range(0, 5):
    robot.setProperty("REP", "position", -1.7)
    robot.waitForJoint("REP")
    robot.setProperty("REP", "position", 0)
    robot.waitForJoint("REP")

robot.setProperties("RSY RSR", "position position", "0 0")
robot.waitForJoint("RSR")
robot.waitForJoint("RSY")
