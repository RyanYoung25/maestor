#! /usr/bin/env python

from Maestor import maestor
from time import time

def main():
    robot = maestor()

    robot.command("InitializeSensors", "")
    print "SensorName Time X_rot Y_rot X_acc Y_acc Z_acc"
    for i in range(0, 10000):
        print "IMU " + str(time()) + " " + robot.getProperties("IMU", "x_rot y_rot x_acc y_acc z_acc")
        #print "IMU" + robot.getProperties("IMU", "x_acc y_acc z_acc")




if __name__ == '__main__':
    main()
