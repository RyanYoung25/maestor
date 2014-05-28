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
    double get(string name, string property);
    void set(string name, string property, double value);
    void getCurrentSupportPhase();
    void DSPControl();
    //void vibrationControl();
    double DampingControl();
    void ZMPInitialization();
    void ZMPcalculation();
public:
    BalanceController();
    virtual ~BalanceController();

    // Retrive all of the components that we need to monitor and control inorder to keep balanced. 
    void initBalanceController(HuboState& theState);
    double getZMP(int value); // 0:X 1:Y  Filtered 
    bool isBalanced(); // Boolean value to say if the robot is balanced or not
    void Balance();    // move joints to balance the robot, stablize the zmp over the support polygon. 
    // Balance will not drastically move the robot and will only move the legs that are involved in support phase. 
    // This means that if the robot is moving the right foot up and the support phase is detected to be on the left foot
    // the balance controller will only move the right ankle and hip treating the robot like an inverted pendulum where the 
    // ankle is the pin. If we are in a double support phase the robot will move both to keep balanced. 
    // Balance is a closed loop function where it only moves based off of the error from the sensor readings and the current values of the metajoints. 


};

#endif /* BALANCECONTROLLER_H_ */
