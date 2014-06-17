#! /usr/bin/env python

from Maestor import maestor 

def main():
	robot = maestor()
	for i in range(0, 200):
		print robot.getProperties("ZMP", "")



if __name__ == '__main__':
	main()