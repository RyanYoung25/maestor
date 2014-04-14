import roslib; roslib.load_manifest('maestor')
import rospy
import sys
import subprocess
from maestor.srv import *

class maestorClient:

    def __init__(self):
        print "Init"
        rospy.init_node("maestor_commands")
        print "Waiting for services"
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
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def setProperty(self, name, prop, value):
        try:
            service = rospy.ServiceProxy("setProperty", setProperty)
            service(name, prop, value)
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
            service(name)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def getProperties(self, names, properties):
        try:
            service = rospy.ServiceProxy("getProperties", getProperties)
            service(names, properties)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def loadTrajectory(self, name, path, read):
        try:
            service = rospy.ServiceProxy("loadTrajectory", loadTrajectory)
            service(name, path, read)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def ignoreFrom(self, name, col):
        try:
            service = rospy.ServiceProxy("ignoreFrom", ignoreFrom)
            service(name, col)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def ignoreAllFrom(self, name):
        try:
            service = rospy.ServiceProxy("ignoreAllFrom", ignoreAllFrom)
            service(name)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def unignoreFrom(self, name, col):
        try:
            service = rospy.ServiceProxy("unignoreFrom", unignoreFrom)
            service(name, col)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def unignoreAllFrom(self, name):
        try:
            service = rospy.ServiceProxy("unignoreAllFrom", unignoreAllFrom)
            service(name)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def setTrigger(self, name, frame, target):
        try:
            service = rospy.ServiceProxy("setTrigger", setTrigger)
            service(name, frame, target)
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

