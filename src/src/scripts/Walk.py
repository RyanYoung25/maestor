#! /usr/bin/env python

from Maestor import maestor 

robot = maestor()

def setVelocities():
    robot.setProperties("RFY LFY", "velocity velocity", ".05 .05")

def resetVelocities():
    robot.setProperties("RFY LFY", "velocity velocity", ".02 .02")

def crouch():
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    robot.waitForJoint("RFZ")
    robot.waitForJoint("LFZ") 

def FirstRightFootUp():
    #Y direction "now with 20% more sway"
    robot.setProperties("RFY LFY", "position position", "-.09 -.09")
    waitForY()
    #X direction
    robot.setProperties("RFX LFX", "position position", ".05 0")
    #Z direction
    robot.setProperties("RFZ LFZ", "position position", "-.485 -.52")
    waitForX()
    waitForZ()

def FirstRightFootDown():
    #X direction
    robot.setProperties("RFX LFX", "position position", ".05 -.05")
    #Y direction "now with 20% more sway"
    robot.setProperties("RFY LFY", "position position", "0 0")
    #X direction
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    waitForX()
    waitForY()
    waitForZ()


def FirstLeftFootUp():
    #Y direction "now with 20% more sway"
    robot.setProperties("RFY LFY", "position position", ".09 .09")
    waitForY()
    #X direction
    robot.setProperties("RFX LFX", "position position", "0 .05")
    #X direction
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.485")
    waitForX()
    waitForY()
    waitForZ()


def FirstLeftFootDown():
    #X direction
    robot.setProperties("RFX LFX", "position position", "-.05 .05")
    #Y direction "now with 20% more sway"
    robot.setProperties("RFY LFY", "position position", "0 0")
    #X direction
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    waitForX()
    waitForY()
    waitForZ()

def RightFootUp():
    #Y direction "now with 20% more sway"
    robot.setProperties("RFY LFY", "position position", "-.09 -.09")
    waitForY()
    #X direction
    robot.setProperties("RFX LFX", "position position", "0 0")
    #X direction
    robot.setProperties("RFZ LFZ", "position position", "-.485 -.52")
    waitForX()
    waitForY()
    waitForZ()

def RightFootDown():
    #X direction
    robot.setProperties("RFX LFX", "position position", ".05 -.05")
    #Y direction "now with 20% more sway"
    robot.setProperties("RFY LFY", "position position", "0 0")
    #X direction
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    waitForX()
    waitForY()
    waitForZ()

def LeftFootUp():
    #Y direction "now with 20% more sway"
    robot.setProperties("RFY LFY", "position position", ".09 .09")
    waitForY()
    #X direction
    robot.setProperties("RFX LFX", "position position", "0 0")
    #X direction
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.485")
    waitForX()
    waitForY()
    waitForZ()

def LeftFootDown():
    #X direction
    robot.setProperties("RFX LFX", "position position", "-.05 .05")
    #Y direction "now with 20% more sway"
    robot.setProperties("RFY LFY", "position position", "0 0")
    #X direction
    robot.setProperties("RFZ LFZ", "position position", "-.52 -.52")
    waitForX()
    waitForY()
    waitForZ()

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
    
def standUp():
    robot.setProperties("RFZ LFZ", "position position", "-.56 -.56")
    waitForX()
    waitForY()
    waitForZ()


def HipToLeft():
    '''Hip To left sway control'''

def HipToRight():
    '''Hip to right sway control'''

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
    setVelocities()
    crouch()
    FirstLeftFootUp()
    FirstLeftFootDown()
    RightFootUp()
    RightFootDown()
    LeftFootUp()
    LeftFootDown()
    RightFootUp()
    HipToCenter()
    standUp()
    resetVelocities()

if __name__ == '__main__':
    main()
    
