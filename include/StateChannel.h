/*
 * StateChannel.h
 *
 *  Created on: Oct 17, 2013
 *      Author: maestro
 */

#ifndef STATECHANNEL_H_
#define STATECHANNEL_H_

#define REFERENCECHANNEL_H_

#include <string>
#include <iostream>
#include "Names.h"
//Hubo-Ach Includes
#include <stdint.h>
#include <sys/types.h>
#include "Singleton.h"
#include "ach.h"
#include "hubo.h"

#define FT_RW 0
#define FT_LW 1
#define FT_RA 2
#define FT_LA 3
#define LEFT_IMU 0
#define RIGHT_IMU 1
#define BODY_IMU 2

using std::string;
using std::cout;
using std::endl;

class StateChannel : public Singleton<StateChannel> {
    friend class Singleton<StateChannel>;
private:

    typedef ach_channel_t AchChannel;
    typedef struct hubo_state State;

    static const char *urdf_joint_names[];

    AchChannel huboStateChannel;
    State currentReference;

    bool errored;

    int indexLookup(string &joint);

protected:
    StateChannel();


public:

    ~StateChannel();

    void load();
    bool getMotorProperty(string &name, PROPERTY property, double& result);
    bool getIMUProperty(string &name, PROPERTY property, double& result);
    bool getFTProperty(string &name, PROPERTY property, double& result);
};

#endif /* STATECHANNEL_H_ */
