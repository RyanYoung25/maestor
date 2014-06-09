#! /usr/bin/env python
import rlcompleter
import readline
import rosservice

readline.parse_and_bind("tab: complete")

def ls():
    return ['/command', '/setProperties', '/setTrigger', '/ignoreAllFrom', '/initRobot', '/fib', '/requiresMotion', '/getProperties', '/unignoreAllFrom', '/startTrajectory', '/loadTrajectory', '/setProperty', '/extendTrajectory', '/stopTrajectory', '/unignoreFrom', '/ignoreFrom']

def loadTrajectory (name, path, read):
    return rosservice.call_service("/loadTrajectory", [name, path, read])[1]

def command (name, target):
    return rosservice.call_service("/command", [name, target])[1]

def unignoreAllFrom (name):
    return rosservice.call_service("/unignoreAllFrom", [name])[1]

def setProperties (names, properties, values):
    return rosservice.call_service("/setProperties", [names, properties, values])[1]

def getProperties (name, properties):
    return rosservice.call_service("/getProperties", [name, properties])[1]

def requiresMotion (name):
    return rosservice.call_service("/requiresMotion", [name])[1]

def setTrigger (name, frame, target):
    return rosservice.call_service("/setTrigger", [name, frame, target])[1]

def extendTrajectory (name, path):
    return rosservice.call_service("/extendTrajectory", [name, path])[1]

def ignoreAllFrom (name):
    return rosservice.call_service("/ignoreAllFrom", [name])[1]

def initRobot (path):
    return rosservice.call_service("/initRobot", [path])[1]

def startTrajectory (name):
    return rosservice.call_service("/startTrajectory", [name])[1]

def ignoreFrom (name, col):
    return rosservice.call_service("/ignoreFrom", [name, col])[1]

def unignoreFrom (name, col):
    return rosservice.call_service("/unignoreFrom", [name, col])[1]

def setProperty (name, property, value):
    return rosservice.call_service("/setProperty", [name, property, value])[1]

def fib (num):
    return rosservice.call_service("/fib", [num])[1]

def stopTrajectory (name):
    return rosservice.call_service("/stopTrajectory", [name])[1]

