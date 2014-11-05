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

/*
 * Walking meta joint
 * 
 * This is the meta joint used to control walking. 
 * The joint overrides the standard interpolation function and keeps the locations for 
 * all of the leg metajoints. 
 * 
 *
 * Author: Ryan
 */

#ifndef BALANCECONTROLLER_H_
#define BALANCECONTROLLER_H_

#include <map>
#include <string>
#include <queue>
#include <iostream>
#include <fstream>
#include "Names.h"
#include "HuboState.h"
#include "RobotComponent.h"
#include "Interpolable.h"
#include "MetaJointController.h"
#include "MetaJoint.h"

class BalanceController{
private:

    typedef Names::Properties Properties;
    typedef Names::Commands Commands;

    std::ofstream logfile;
    enum SupportPhase {LEFT_FOOT, RIGHT_FOOT, BOTH_FEET};
    bool initialized;
    double InitZmp[2]; // initial ZMP
    double BaseDSP[2][2]; //offset baseline
    // Balance Information
    SupportPhase phase; 
    double zmp[6];
    double filteredZMP[6];
    double dampingGain[6]; 
    double InitRefAngles[2][3]; // Right:0 Left:1 / X:0 Y:1 Z:2
    double hipPitchOffsets[2];  // Right:0 Left:1 
    double ControlDSP[2][2];    // Right:0 Left:1 / X:0 Y:1
    double Damping[4];   // RAP:0 RAR:1 LAP:2 LAR:3
    // The hubo state which has all of the metajoints and sensors
    HuboState* state; 

    // The metajoints for balancing and calculating
    string balanceComponents[9];
    // Initialization check
    bool allComponentsFound();
    // Calculation methods
    // Carried over from Robot Control. Get the property on joint named "name"
    double get(string name, string property);
    // Carried over from Robot Control. Set the property on the joint named "name" as the value
    void set(string name, string property, double value);
    // sets the interpolation offset for a joint. This value is added at each interpolation step, essentially changing the speed of the joint. 
    // It is only used on meta joints not physical joints meaning that the physical joints will never interpolate too fast. This is a good thing
    void setOffset(string name, double offset);
    // Tells you if you are standing on the Left foot, Right foot, or Both feet
    void getCurrentSupportPhase();
    // I think it stands for Digital Signal Processing, none the less it generates the offsets for the X, Y, and Z coordinates.
    // this is the key player in balancing 
    void DSPControl(); 
    // Not really used, controls the damping of the foot. Was used for gait generation but not anymore.
    double DampingControl(); 
    // Initialize the ZMP values and potentially some initial offsets for the ankle rolls. But we aren't there yet
    void ZMPInitialization(); 
    // Calculates the ZMP positions. These are then used in the DSP controller to calculate offsets
    void ZMPcalculation();
    // Carried over from Robot Control. Checks to see if the is at it's goal position. If it isn't it requires motion
    bool requiresMotion(string name);
public:
    BalanceController();
    virtual ~BalanceController();

    // Retrive all of the components that we need to monitor and control inorder to keep balanced. 
    void initBalanceController(HuboState& theState);
    double getZMP(int value); // 0:X 1:Y  Filtered
    void setBaseline();// Take the current values from the DSP control function and set those as the baseline zero. Future values are modified by this. 
    void Balance();    // move joints to balance the robot, stablize the zmp over the support polygon. 
    // Balance has two differnt ways of controlling the joints, if the joint is in motion because someone else set it's position then the function will only
    // alter the interpolation steps using the offsets it generates. If the joint is not moving then it attempts to balance it's self 
    // by setting new positions based off of offsets and the current joint position.  


};

#endif /* BALANCECONTROLLER_H_ */
