/*
Copyright (c) 2013, Drexel University, iSchool, Applied Informatics Group
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "PowerControlBoard.h"

/**
 * The power control board constructor. Allows you to calculate consumed power on the robot.
 */
PowerControlBoard::PowerControlBoard(){

    powerLookup.open(LOOKUP_TABLE_PATH);
    powerUsed = 0;

    a_nought["IDLE"] = -.008779441 / 60;

    a_nought["RSR"] = -0.036720559 / 60;
    a_nought["RSY"] = -0.001107553 / 60;
    a_nought["REP"] = -0.013902377 / 60;

}

/**
 * Set the initial power consumed
 * @param initialPower [The initial power consumed
 */
void PowerControlBoard::setInitialPower(double initialPower){
    powerUsed = initialPower;
}

/**
 * Get the total power that has been used so far
 * @return The total power that has been used
 */
double PowerControlBoard::getTotalPowerUsed(){
    return powerUsed;
}

/**
 * Add the power a joint used from a start time to an end time
 * @param  joint The joint that used the power
 * @param  from  The start time
 * @param  to    The end time
 * @return       True on success
 */
bool PowerControlBoard::addMotionPower(string joint, double from, double to){

    string line;
    char dataJoint[10];
    double dataFrom = 0;
    double dataTo = 0;
    double delta = 0;
    int scanned = 0;
    do {
        getline(powerLookup, line, '\n');

        scanned = sscanf(line.c_str(), "%s %lf %lf %lf",
            &dataJoint, &dataFrom, &dataTo, &delta);
        if (strcmp(dataJoint, joint.c_str()) == 0 && from == dataFrom && to == dataTo){
            powerUsed += delta;
            powerLookup.close();
            powerLookup.open(LOOKUP_TABLE_PATH);
            return true;
        }
    } while (scanned == 4);
    powerLookup.close();
    powerLookup.open(LOOKUP_TABLE_PATH);
    return false;
}

/**
 * Add the power a joint used over a certian period
 * @param joint  The joint that was used
 * @param period The period of time it was used
 */
void PowerControlBoard::addMotionPower(string joint, double period){
    if (a_nought.count(joint) == 1)
        powerUsed += a_nought[joint] * period;
}
