/*
 * NeckRollPitch.h
 *
 *  Created on: Mar 4, 2014
 *      Author: solisknight
 */

#ifndef NECKROLLPITCH_H_
#define NECKROLLPITCH_H_

#include "MetaJoint.h"
#include "RobotComponent.h"
#include "Names.h"

class NeckRollPitch : public MetaJointController {
private:
    static const int NUM_PARAMETERS;
    static const int NUM_CONTROLLED;

    //Expected Parameter Indices
    static const int ROLL;
    static const int PITCH;

    //Expected Controlled Indices
    static const int NECK1;
    static const int NECK2;

public:
    NeckRollPitch();
    virtual ~NeckRollPitch();

    void setInverse();
    void getForward();
};

#endif /* NECKROLLPITCH_H_ */
