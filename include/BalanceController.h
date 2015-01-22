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

#define ROLL_LIMIT 0.18
#define PITCH_LIMIT 0.50

#include <map>
#include <string>
#include <queue>
#include <iostream>
#include <fstream>
#include "Names.h"
#include "HuboState.h"
#include "RobotComponent.h"
#include "RobotControl.h"
#include "Interpolable.h"
#include "MetaJointController.h"
#include "MetaJoint.h"

class RobotControl;

class BalanceController{
private:

    typedef Names::Properties Properties;
    typedef Names::Commands Commands;

    std::ofstream logfile;
    enum SupportPhase {LEFT_FOOT, RIGHT_FOOT, BOTH_FEET};
    double BaseDSP[2][2]; //offset baseline
    // Balance Information
    SupportPhase phase; 
    double zmp[6];
    double filteredZMP[6];
    double dampingGain[6]; 
    double ControlDSP[2][2];    // Right:0 Left:1 / X:0 Y:1
    double Damping[4];   // RAP:0 RAR:1 LAP:2 LAR:3
    //double Landing[4];   // RAP:0 RAR:1 LAP:2 LAR:3

    //Landing information 
    
    double lReference;
    double oldRError;
    double oldPError;
    double timeStep;

    double AlphaX;
    double AlphaY;

    double smoothMX;
    double smoothMY;

    // The robot which has all of the metajoints and sensors and runs the show
    HuboState* state;

    // The robot control object 
    RobotControl* robotController;
    
    // CALCULATION METHODS:
        
    // Tells you if you are standing on the Left foot, Right foot, or Both feet
    void getCurrentSupportPhase();
    
    // I think it stands for Digital Signal Processing, none the less it generates the offsets for the X, Y, and Z coordinates.
    // this is the key player in balancing 
    void DSPControl(); 

    // Carried over from Robot Control. Get the property on joint named "name"
    double get(string name, string property);
    // Carried over from Robot Control. Set the property on the joint named "name" as the value
    void set(string name, string property, double value);

    // Calculates the ZMP positions. These are then used in the DSP controller to calculate offsets
    void ZMPcalculation();

    // The landing controller, This will be filled in with data from the force torque sensors and handle the foot landing on
    // uneven surfaces. This should create offsets for the ankle rolls and pitches.
    // This will set the  
    
    double runPD(double P, double D, double PastError, double Error);
    //Carried over from Robot Control. Checks to see if a joint needs to move
    bool requiresMotion(string name);

public:
    BalanceController(RobotControl& controller);
    virtual ~BalanceController();

    // Retrive all of the components that we need to monitor and control inorder to keep balanced. 
    void initBalanceController(HuboState& theRobot);
    void landingControl();
    double getZMP(int value); // 0:X 1:Y  Filtered
    void setBaseline();// Take the current values from the DSP control function and set those as the baseline zero. Future values are modified by this. 
    void Balance();    // move joints to balance the robot, stablize the zmp over the support polygon. 
    // Balance has two differnt ways of controlling the joints, if the joint is in motion because someone else set it's position then the function will only
    // alter the interpolation steps using the offsets it generates. If the joint is not moving then it attempts to balance it's self 
    // by setting new positions based off of offsets and the current joint position.  


};

#endif /* BALANCECONTROLLER_H_ */
