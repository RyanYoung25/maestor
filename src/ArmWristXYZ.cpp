/**
 * The arm meta joint controller. This uses a simpilfied 
 * Arm inverse kinematics that John Maloney derived. He worked
 * really hard on it and deserves a ton of credit for it. 
 */

#include "../include/ArmWristXYZ.h"

const int ArmWristXYZ::NUM_PARAMETERS = 3;
const int ArmWristXYZ::NUM_CONTROLLED = 4;

//Expected Parameter Indices
const int ArmWristXYZ::WRIST_X = 0;
const int ArmWristXYZ::WRIST_Y = 1;
const int ArmWristXYZ::WRIST_Z = 2;

//Expected Controlled Indices
const int ArmWristXYZ::SHOULDER_YAW = 0;
const int ArmWristXYZ::SHOULDER_PITCH = 1;
const int ArmWristXYZ::SHOULDER_ROLL = 2;
const int ArmWristXYZ::ELBOW_PITCH = 3;

//Constants which should be parameterized
const double ArmWristXYZ::UPPER_ARM_Z = 0.182;
const double ArmWristXYZ::UPPER_ARM_X = 0.02;
const double ArmWristXYZ::LOWER_ARM_Z = 0.164;
const double ArmWristXYZ::LOWER_ARM_X = 0.022;
const double ArmWristXYZ::ARM_MIN_REACH = 0.1;
const double ArmWristXYZ::REP_OFFSET = 0.0411;
const double ArmWristXYZ::SHOULDER_PITCH_UPPER = 2.96;
const double ArmWristXYZ::SHOULDER_PITCH_LOWER = -2.96;
const double ArmWristXYZ::ELBOW_PITCH_UPPER = .2;
const double ArmWristXYZ::ELBOW_PITCH_LOWER = -2.3;

/**
 * Create an Arm metajoint controller object and pass in a boolean flag if  
 * it is a left arm. 
 */
ArmWristXYZ::ArmWristXYZ(bool left) : MetaJointController(NUM_PARAMETERS, NUM_CONTROLLED) {
    isLeft = left;
    SHOULDER_ROLL_UPPER = .59;
    SHOULDER_ROLL_LOWER = -2.61;
    SR_OFFSET = 0.2618;
    SR_IK_OFFSET = SR_OFFSET + atan(UPPER_ARM_X/UPPER_ARM_Z);
    jointsSet = false;
    if(isLeft){
        SHOULDER_ROLL_UPPER = -SHOULDER_ROLL_LOWER;
        SHOULDER_ROLL_LOWER = -SHOULDER_ROLL_UPPER;
        SR_OFFSET = -SR_OFFSET;
        shoulder_yaw = -1.57;
    }
    else{
        shoulder_yaw = 1.57;
    }
}

/**
 * Destructor
 */
ArmWristXYZ::~ArmWristXYZ() {}

/**
 * Set the inverse of the arms. This calculates the inverse kinematics of 
 * the controlled joints based off of the X Y and Z meta joints. 
 */
void ArmWristXYZ::setInverse(){

    if(jointsSet){
        checkGoalsReached();
    }

    if(!updated){
        unsetAll();
        return;
    }
    
    if (!allSet()){ //wait for all joints
        return;
    }

   
    double wrist_x = 0.0; 
    double wrist_y = 0.0; 
    double wrist_z = 0.0; 

    parameters[WRIST_X]->get(INTERPOLATION_STEP, wrist_x);
    parameters[WRIST_Y]->get(INTERPOLATION_STEP, wrist_y);
    if(isLeft){
        wrist_y = -wrist_y;
    }
    parameters[WRIST_Z]->get(INTERPOLATION_STEP, wrist_z);

    double shoulder_pitch = 0;
    double shoulder_roll = 0;
    double elbow_pitch = 0;

    double radius = sqrt(wrist_x*wrist_x + wrist_y*wrist_y + wrist_z*wrist_z);   //line in 3d space
    double U = sqrt(UPPER_ARM_X*UPPER_ARM_X + UPPER_ARM_Z*UPPER_ARM_Z);          //Length of the upper arm
    double L = sqrt(LOWER_ARM_X*LOWER_ARM_X + UPPER_ARM_Z*UPPER_ARM_Z);          //Length of the lower arm

    if(radius > L + U || radius < ARM_MIN_REACH){
        cout << "Error: Position is out of arm's reach" << endl;
        unsetAll();
        return;
    }

    // Angle between arm and xz planes
    double xz_angle = atan2(wrist_x, -wrist_z);
    // Angle in y direction of radius vector in plane of the arm
    double plane_angle = asin(wrist_y/radius);

    //  Angles of triangle formed by the two halves of the arm and the xyz vector
    //  determined by law of cosines. Name is a_[opposite leg of triangle]
    double a_lower = acos
                        (
                            (U*U + radius*radius - L*L) /
                            (2*U*radius)
                        );
    double a_radius = acos
                        (
                            (U*U + L*L - radius*radius) /
                            (2*U*L)
                        );

    shoulder_pitch = -xz_angle;
    shoulder_roll = plane_angle + SR_IK_OFFSET - a_lower;
    if(isLeft){
        shoulder_roll = -shoulder_roll;
    }
    elbow_pitch = REP_OFFSET + a_radius - M_PI;

    // Not entirely sure if this check is necessary
    if(isnan(shoulder_pitch) || isnan(shoulder_roll) || isnan(elbow_pitch)){
        cout << "Error: inverse solver returned NaN" << endl;
        unsetAll();
        return;
    }

    // Check that joint angles are within limits
    if(shoulder_roll > SHOULDER_ROLL_UPPER || shoulder_roll < SHOULDER_ROLL_LOWER || shoulder_pitch > SHOULDER_PITCH_UPPER || shoulder_pitch < SHOULDER_PITCH_LOWER || elbow_pitch > ELBOW_PITCH_UPPER || elbow_pitch < ELBOW_PITCH_LOWER){
        cout << "Error: One or more joints out of joint limits" << endl;
        unsetAll();
        return;
    }


    controlledJoints[SHOULDER_YAW]->set(GOAL, shoulder_yaw);
    controlledJoints[SHOULDER_PITCH]->set(GOAL, shoulder_pitch);
    controlledJoints[SHOULDER_ROLL]->set(GOAL, shoulder_roll);
    controlledJoints[ELBOW_PITCH]->set(GOAL, elbow_pitch);
    jointsSet = true;

    unsetAll();
}

/**
 * Check to see if the arm joints made it to their goals. 
 * If they did we don't need to control them until we are updated. 
 */
void ArmWristXYZ::checkGoalsReached(){
    double pos;
    double goal;
    for(int i = 0; i < 4; i ++){
        controlledJoints[i]->get(POSITION, pos);
        controlledJoints[i]->get(GOAL, goal);
        if(fabs(pos - goal) > .001){
            return;
        }
    }
    cout << "Joints set set to false" << endl;
    jointsSet = false;
    goalsReached();
}

/**
 * Calculate the forward kinematics based off of the controlled
 * joints positions. Might have a small bug. 
 */
void ArmWristXYZ::getForward(){
    double P = 0;
    double R = 0;
    double Y = 0;
    double E = 0;

    controlledJoints[SHOULDER_PITCH]->get(GOAL, P);
    controlledJoints[SHOULDER_ROLL]->get(GOAL, R);
    R = R - SR_OFFSET;
    controlledJoints[SHOULDER_YAW]->get(GOAL, Y);
    controlledJoints[ELBOW_PITCH]->get(GOAL, E);
    E = E - REP_OFFSET;

    double L = sqrt(LOWER_ARM_Z*LOWER_ARM_Z + LOWER_ARM_X*LOWER_ARM_X);
    double sP= sin(P);
    double sR= sin(R);
    double sY= sin(Y);
    double sE= sin(E);
    double cP= cos(P);
    double cR= cos(R);
    double cY= cos(Y);
    double cE= cos(E);

    double zPos = -1*(L*(cE*cP*cR + sE*(cP*sR*sY - sP*cY)) + UPPER_ARM_X*(sP*cY - cP*sR*sY) + UPPER_ARM_Z*cP*cR);
    double xPos = -1*(L*(cE*sP*cR + sE*(sP*sR*sY + cP*cY)) - UPPER_ARM_X*(cP*cY + sP*sR*sY) + UPPER_ARM_Z*sP*cR);
    double yPos = L*(cE*sR - sE*cR*sY) + UPPER_ARM_X*cR*sY + UPPER_ARM_Z*sR;

    // Shoulder pitch reference frame axes do not match that of hubo's reference frame

    parameters[WRIST_X]->set(META_VALUE, xPos);
    parameters[WRIST_Y]->set(META_VALUE, yPos);
    parameters[WRIST_Z]->set(META_VALUE, zPos);
}
