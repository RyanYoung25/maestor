import roslib; roslib.load_manifest('maestor')
import rospy
import sys
import subprocess
import time
from maestor.srv import *

class maestor:

    def __init__(self):
        rospy.init_node("maestor_commands", anonymous=True)
        rospy.wait_for_service("initRobot")
        rospy.wait_for_service("setProperties")
        rospy.wait_for_service("command")
        rospy.wait_for_service("requiresMotion")
        rospy.wait_for_service("getProperties")
        rospy.wait_for_service("loadTrajectory")
        rospy.wait_for_service("ignoreFrom")
        rospy.wait_for_service("ignoreAllFrom")
        rospy.wait_for_service("unignoreFrom")
        rospy.wait_for_service("unignoreAllFrom")
        rospy.wait_for_service("setTrigger")
        rospy.wait_for_service("extendTrajectory")
        rospy.wait_for_service("startTrajectory")
        rospy.wait_for_service("stopTrajectory")
        rospy.wait_for_service("setProperty")
        self.shouldWait = False
        print "All services are available"
    
    def initRobot(self, path):
        try:
            init_robot = rospy.ServiceProxy("initRobot", initRobot)
            init_robot(path)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def setProperties(self, names, properties, values):
        try:
            service = rospy.ServiceProxy("setProperties", setProperties)
            service(names, properties, values)

            #If should wait for the joints
            if self.shouldWait:
                jointList = names.split(" ")
                self.waitForJointList(jointList)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def setProperty(self, name, prop, value):
        try:
            service = rospy.ServiceProxy("setProperty", setProperty)
            service(name, prop, value)

            if self.shouldWait:
                self.waitForJoint(name)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def command(self, name,target):
        try:
            service = rospy.ServiceProxy("command", command)
            service(name, target)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def requiresMotion(self, name):
        try:
            service = rospy.ServiceProxy("requiresMotion", requiresMotion)
            res = service(name)
            return res.requiresMotion
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def getProperties(self, names, properties):
        try:
            service = rospy.ServiceProxy("getProperties", getProperties)
            res = service(names, properties)
            return res.properties
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def loadTrajectory(self, name, path, read):
        try:
            service = rospy.ServiceProxy("loadTrajectory", loadTrajectory)
            res = service(name, path, read)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def ignoreFrom(self, name, col):
        try:
            service = rospy.ServiceProxy("ignoreFrom", ignoreFrom)
            res = service(name, col)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def ignoreAllFrom(self, name):
        try:
            service = rospy.ServiceProxy("ignoreAllFrom", ignoreAllFrom)
            res = service(name)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def unignoreFrom(self, name, col):
        try:
            service = rospy.ServiceProxy("unignoreFrom", unignoreFrom)
            res = service(name, col)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def unignoreAllFrom(self, name):
        try:
            service = rospy.ServiceProxy("unignoreAllFrom", unignoreAllFrom)
            res = service(name)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def setTrigger(self, name, frame, target):
        try:
            service = rospy.ServiceProxy("setTrigger", setTrigger)
            res = service(name, frame, target)
            return res.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def extendTrajectory(self, name, path):
        try:
            service = rospy.ServiceProxy("extendTrajectory", extendTrajectory)
            service(name, path)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def startTrajectory(self, name):
        try:
            service = rospy.ServiceProxy("startTrajectory", startTrajectory)
            service(name)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def stopTrajectory(self, name):
        try:
            service = rospy.ServiceProxy("stopTrajectory", stopTrajectory)
            service(name)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def waitForJoint(self, name):
        while self.requiresMotion(name):
            pass

    def waitForJointList(self, jointList):
        #Wait for a list of joints, each element of the list 
        # must be a string
        for joint in jointList:
            waitForJoint(joint)

    def defaultWaitForJoint(self, shouldWait=False):
        #If should wait becomes true, make it so that 
        # every time you move a joint you block until 
        # it stops
        self.shouldWait = shouldWait

    #robot.waitForJointList(["RSP", "LSP", "RSR", "LSR"])

