#! /usr/bin/env python

from Maestor import maestor 
import time

robot = maestor()

def setVelocities():
    robot.setProperties("RFY LFY", "velocity velocity", ".05 .05")

def resetVelocities():
    robot.setProperties("RFY LFY", "velocity velocity", ".02 .02")

def crouch():
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ") 

def stand():
    robot.setProperties("RFZ LFZ", "position position", "-.56 -.56")
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ")

def HipToCenter():
    #X direction
    robot.setProperties("RFX LFX", "position position", "0 0")
    #Y direction "now with 20% more sway"
    robot.setProperties("RFY LFY", "position position", "0 0")
    #X direction
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    waitForX()
    waitForY()
    waitForZ()

def HipToLeft():
    '''Hip To left sway control'''
    robot.setProperties("RFY LFY", "position position", ".09 .09")
    waitForY()

def raiseLeg():
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.485")
    waitForZ()

def bounce():
    for i in range(0, 5):
        robot.setProperty("RFZ", "position", -.54)
        robot.waitForJoint("RFZ")
        robot.setProperty("RFZ", "position", -.52)
        robot.waitForJoint("RFZ")
    robot.setProperty("RFZ", "position", -.52)
    robot.waitForJoint("RFZ")

def lowerLeg():
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    waitForZ()

def waitForX():
    robot.waitForJoint("RFX")
    robot.waitForJoint("LFX")

def waitForY():
    robot.waitForJoint("RFY")
    robot.waitForJoint("LFY")

def waitForZ():
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ")



def main():
    #setVelocities()
    crouch()
    HipToLeft()
    raiseLeg()
    bounce()
    lowerLeg()
    HipToCenter()
    stand()
    #resetVelocities()

if __name__ == '__main__':
    main()
    
