#! /usr/bin/env python

from Maestor import maestor
from time import time

def main():
	robot = maestor()

	robot.command("InitializeSensors", "")

	for i in range(0, 1000):
		print "LAT " + str(time()) + " " + robot.getProperties("LAT", "m_x m_y f_z")
		print "RAT " + str(time()) + " " + robot.getProperties("RAT", "m_x m_y f_z")
		#print "IMU" + robot.getProperties("IMU", "x_acc y_acc z_acc")




if __name__ == '__main__':
	main()