#! /usr/bin/env python
from Maestor import maestor
import sys

joint = ""
minLim = 0
maxLim = 0

def main():
	#create the maestor robot object
	robot = maestor()
	robot.setProperty(joint, "velocity", .3) #constant velocity 
	robot.setProperty(joint, "position", minLim) #start at the min lim

	for i in range(0, 10):
		robot.setProperty(joint, "position", maxLim)
		robot.waitForJoint(joint)
		robot.setProperty(joint, "position", minLim)
		robot.waitForJoint(joint)

if __name__ == '__main__':
	if len(sys.argv) == 4:
		joint = sys.argv[1]
		minLim = float(sys.argv[2])
		maxLim = float(sys.argv[3])
	else:
		print "Needs args, joint, min, max"
		sys.exit()
	main()
