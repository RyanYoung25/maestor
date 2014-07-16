/*
 * LowerBodyLeg.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: solisknight
 */

#include "../include/LowerBodyLeg.h"

const int LowerBodyLeg::NUM_PARAMETERS = 6;
const int LowerBodyLeg::NUM_CONTROLLED = 6;

//Expected Parameter Indices
const int LowerBodyLeg::FOOT_X = 0;
const int LowerBodyLeg::FOOT_Y = 1;
const int LowerBodyLeg::FOOT_Z = 2;
const int LowerBodyLeg::FOOT_YAW = 3;
const int LowerBodyLeg::FOOT_ROLL = 4;
const int LowerBodyLeg::FOOT_PITCH = 5;

//Expected Controlled Indices
const int LowerBodyLeg::HIP_YAW = 0;
const int LowerBodyLeg::HIP_ROLL = 1;
const int LowerBodyLeg::HIP_PITCH = 2;
const int LowerBodyLeg::KNEE_PITCH = 3;
const int LowerBodyLeg::ANKLE_PITCH = 4;
const int LowerBodyLeg::ANKLE_ROLL = 5;

//Constants which should be parameterized
const double LowerBodyLeg::LENGTH_THIGH = .28;
const double LowerBodyLeg::LENGTH_CALF = .28;
const double LowerBodyLeg::HIP_ROLL_UPPER = .54;
const double LowerBodyLeg::HIP_ROLL_LOWER = -.54;
const double LowerBodyLeg::HIP_PITCH_UPPER = 1.91;
const double LowerBodyLeg::HIP_PITCH_LOWER = -1.91;
const double LowerBodyLeg::KNEE_PITCH_UPPER = 2.6;
const double LowerBodyLeg::KNEE_PITCH_LOWER = -.06;
const double LowerBodyLeg::ANKLE_PITCH_UPPER = 1.69;
const double LowerBodyLeg::ANKLE_PITCH_LOWER = -1.29;
const double LowerBodyLeg::ANKLE_ROLL_UPPER = .19;
const double LowerBodyLeg::ANKLE_ROLL_LOWER = -.19;


LowerBodyLeg::LowerBodyLeg(bool global) : MetaJointController(NUM_PARAMETERS, NUM_CONTROLLED) {
    globalFlag = global;
}

LowerBodyLeg::~LowerBodyLeg() {}

// Set foot posiion and orientation in hip yaw reference frame
void LowerBodyLeg::setInverse(){
    checkGoalsReached();
    if(!updated){
        unsetAll();
        return;
    }

    if (!allSet()) //wait for all joints
        return;

    double foot_x = 0;
    double foot_y = 0;
    double foot_z = 0;
    double foot_yaw = 0;

    //getForward();
    //Trying something out
    /*parameters[HIP_YAW]->set(VELOCITY, 0.02);
    parameters[HIP_PITCH]->set(VELOCITY, 0.02);
    parameters[HIP_ROLL]->set(VELOCITY, 0.02);
    parameters[KNEE_PITCH]->set(VELOCITY, 0.02);
    parameters[ANKLE_ROLL]->set(VELOCITY, 0.02);
    parameters[ANKLE_PITCH]->set(VELOCITY, 0.02);*/

    parameters[FOOT_X]->get(INTERPOLATION_STEP, foot_x);
    parameters[FOOT_Y]->get(INTERPOLATION_STEP, foot_y);
    parameters[FOOT_Z]->get(INTERPOLATION_STEP, foot_z);
    parameters[FOOT_YAW]->get(INTERPOLATION_STEP, foot_yaw);

    if(globalFlag){
        // Convert position to hip roll reference frame and set joint angles
        setHipRollXYZ(cos(foot_yaw)*foot_x + sin(foot_yaw)*foot_y, cos(foot_yaw)*foot_y - sin(foot_yaw)*foot_x, foot_z, foot_yaw);
    }
    else{
        setHipRollXYZ(foot_x, foot_y, foot_z, foot_yaw);
    }

    unsetAll();
}

// Get foot posiion in hip yaw reference frame forward kinematics based on joint angles
void LowerBodyLeg::getForward(){
    vector<double> v = getHipRollXYZ();
    
    if(globalFlag){
        double hip_yaw = 0;
        controlledJoints[HIP_YAW]->get(GOAL,hip_yaw);
        // Convert position to hip yaw reference frame
        parameters[FOOT_X]->set(META_VALUE, cos(hip_yaw)*v[0] - sin(hip_yaw)*v[1]);
        parameters[FOOT_Y]->set(META_VALUE, sin(hip_yaw)*v[0] - cos(hip_yaw)*v[1]);
        parameters[FOOT_Z]->set(META_VALUE, v[2]);
        //cout << "x: " << cos(hip_yaw)*v[0] - sin(hip_yaw)*v[1] << " y: " << sin(hip_yaw)*v[0] - cos(hip_yaw)*v[1] << " z: " << v[2] << endl;
    }
    else{

        parameters[FOOT_X]->set(META_VALUE, v[0]); 
        parameters[FOOT_Y]->set(META_VALUE, v[1]);
        parameters[FOOT_Z]->set(META_VALUE, v[2]);
        //cout << "x: " << v[0] << " y: " << v[1] << " z: " << v[2] << endl;
    }

}

// Calculate joint angles to achieve desired position in hip roll reference fram
void LowerBodyLeg::setHipRollXYZ(double foot_x, double foot_y, double foot_z, double foot_yaw){
    //cout << foot_x << " " << foot_y << " " << foot_z << endl;
    double foot_roll    = 0;
    double foot_pitch   = 0;

    parameters[FOOT_ROLL]->get(INTERPOLATION_STEP, foot_roll);
    parameters[FOOT_PITCH]->get(INTERPOLATION_STEP, foot_pitch);

    double hip_roll = 0;
    double hip_pitch = 0;
    double knee_pitch = 0;
    double ankle_pitch = 0;
    double ankle_roll = 0;

    knee_pitch = M_PI - acos(

            ( LENGTH_THIGH * LENGTH_THIGH + LENGTH_CALF * LENGTH_CALF - ( foot_x * foot_x + foot_y * foot_y + foot_z * foot_z ) )
                    / (2.0f * LENGTH_THIGH * LENGTH_CALF)

            );


    hip_roll = atan2( foot_y, -foot_z );

    hip_pitch = asin(

            (
                -sin(knee_pitch) * LENGTH_CALF * (-foot_y * sin(hip_roll) + foot_z * cos(hip_roll) )
                + foot_x * ( cos(knee_pitch) * LENGTH_CALF + LENGTH_THIGH)
            ) /
            (
                -foot_x * foot_x - (foot_y * sin(hip_roll) - foot_z * cos(hip_roll)) * (foot_y * sin(hip_roll) - foot_z * cos(hip_roll) )
            )

    );

    ankle_roll = -hip_roll;

    ankle_pitch = -hip_pitch - knee_pitch;

    if ( isnan(foot_yaw) || isnan(hip_roll) || isnan(hip_pitch) || isnan(knee_pitch) || isnan(ankle_pitch) || isnan(ankle_roll) ){
        cout << "Error: Inverse solver returned NaN" << endl;
        return;
    }

    if (hip_roll < HIP_ROLL_LOWER || hip_roll > HIP_ROLL_UPPER || hip_pitch < HIP_PITCH_LOWER || hip_pitch > HIP_PITCH_UPPER || knee_pitch < KNEE_PITCH_LOWER || knee_pitch > KNEE_PITCH_UPPER || ankle_pitch < ANKLE_PITCH_LOWER || ankle_pitch > ANKLE_PITCH_UPPER || ankle_roll < ANKLE_ROLL_LOWER || ankle_roll > ANKLE_ROLL_UPPER){
        cout << "Error: One or more joints out of joint limits" << endl;
        return;
    }

    controlledJoints[HIP_YAW]->set(GOAL, foot_yaw);
    controlledJoints[HIP_ROLL]->set(GOAL, hip_roll);
    controlledJoints[HIP_PITCH]->set(GOAL, hip_pitch);
    controlledJoints[KNEE_PITCH]->set(GOAL, knee_pitch);
    controlledJoints[ANKLE_PITCH]->set(GOAL, ankle_pitch);
    controlledJoints[ANKLE_ROLL]->set(GOAL, ankle_roll);
}

void LowerBodyLeg::checkGoalsReached(){
    double pos;
    double goal;
    for(int i = 0; i < 6; i ++){
        parameters[i]->get(POSITION, pos);
        parameters[i]->get(GOAL, goal);
        if(fabs(pos - goal) > .000001){
            return;
        }
    }
    for(int i = 0; i < 6; i ++){
        parameters[i]->set(INTERPOLATION_STEP, 0);
    }
    goalsReached();
}

// Calculate position in hip roll reference frame from joint angles
vector<double> LowerBodyLeg::getHipRollXYZ(){
    double R = 0; // Hip roll
    double P = 0; // Hip pitch
    double K = 0; // Knee pitch

    controlledJoints[HIP_ROLL]->get(GOAL, R);
    controlledJoints[HIP_PITCH]->get(GOAL, P);
    controlledJoints[KNEE_PITCH]->get(GOAL, K);

    double sR = sin(R);
    double sP = sin(P);
    double sK = sin(K);
    double cR = cos(R);
    double cP = cos(P);
    double cK = cos(K);

    double z = LENGTH_CALF*(cR*sP*sK - cR*cP*cK) - LENGTH_THIGH*cR*cP;
    double y = LENGTH_CALF*(sR*cP*cK - sR*sP*sK) + LENGTH_THIGH*sR*cP;
    double x = -LENGTH_CALF*(sP*cK + cP*sK) - LENGTH_THIGH*sP;

    vector<double> v;
    v.push_back(x);
    v.push_back(y);
    v.push_back(z);
    return v;
}
