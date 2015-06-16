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

#ifndef HUBO_FT_COUNT
#define HUBO_FT_COUNT 4
#endif

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
    ~StateChannel();

public:

    void load();
    bool getMotorProperty(string &name, PROPERTY property, double& result);
    bool getMotorProperty(int board, PROPERTY property, double& result);
    bool getIMUProperty(string &name, PROPERTY property, double& result);
    bool getIMUProperty(int board, PROPERTY property, double& result);
    bool getFTProperty(string &name, PROPERTY property, double& result);
    bool getFTProperty(int board, PROPERTY property, double& result);
};

#endif /* STATECHANNEL_H_ */
