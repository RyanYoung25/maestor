#! /usr/bin/env python

from Maestor import maestor 
import time

robot = maestor()

def main():
    robot.command("Disable", "RSR")
    robot.command("Disable", "RSP")
    robot.command("Disable", "REP")
    robot.setProperties("LSR LSP LEP", "velocity velocity velocity", ".2 .2 .2")
    while True:
        Rsrpos = float(robot.getProperties("RSR", "position"))
        Rsppos = float(robot.getProperties("RSP", "position"))
        Reppos = float(robot.getProperties("REP", "position"))
        Lsrpos = -1 * Rsrpos
        Lsppos = Rsppos
        Leppos = Reppos
        robot.setProperty("LSR", "position", Lsrpos)
        robot.setProperty("LSP", "position", Lsppos)
        robot.setProperty("LEP", "position", Leppos)
        time.sleep(.05)
