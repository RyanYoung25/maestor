/**
 * The neck roll and pitch meta joint. This allows you to set 
 * the NKP and NKR joints and have them work in the expected way. 
 * This assumes that hubo ach is configured correctly to allow the neck to be commanded correctly.
 */

#include "NeckRollPitch.h"

const int NeckRollPitch::NUM_PARAMETERS = 2;
const int NeckRollPitch::NUM_CONTROLLED = 2;

//Expected Parameter Indices
const int NeckRollPitch::ROLL = 0;
const int NeckRollPitch::PITCH = 1;

//Expected Controlled Indices
const int NeckRollPitch::NECK1 = 0;
const int NeckRollPitch::NECK2 = 1;


/**
 * Constructor extended as a meta joint controller
 */
NeckRollPitch::NeckRollPitch() : MetaJointController(NUM_PARAMETERS, NUM_CONTROLLED) {}

/**
 * Destructor
 */
NeckRollPitch::~NeckRollPitch() {}

/**
 * Calculate and set the inverse for all of the controlled joints based off
 * of the meta joints positions. 
 */
void NeckRollPitch::setInverse(){
    if (!allSet())
        return;

    double roll = 0;
    double pitch = 0;

    parameters[ROLL]->get(INTERPOLATION_STEP, roll);
    parameters[PITCH]->get(INTERPOLATION_STEP, pitch);

    //Credit to William Hilton
    double NK1pos = ((-.378874 * roll) + (.322581 * pitch)) * 2 * M_PI;
    double NK2pos = ((.378874 * roll) + (.322581 * pitch)) * 2 * M_PI;

    controlledJoints[NECK1]->set(GOAL, NK1pos);
    controlledJoints[NECK2]->set(GOAL, NK2pos);

    unsetAll();
}

/**
 * Get the forward position of the meta joints based off of the positions of the 
 * controlled joints
 */
void NeckRollPitch::getForward(){
    double NK1pos = 0;
    double NK2pos = 0;

    controlledJoints[NECK1]->get(POSITION, NK1pos);
    NK1pos /= (2 * M_PI);
    controlledJoints[NECK2]->get(POSITION, NK2pos);
    NK2pos /= (2 * M_PI);

    //Credit to William Hilton
    double neckRoll = (-1.3197 * NK1pos) + (1.3197 * NK2pos);
    double neckPitch = (1.55 * NK1pos) + (1.55 * NK2pos);

    parameters[ROLL]->set(META_VALUE, neckRoll);
    parameters[PITCH]->set(META_VALUE, neckPitch);
}
