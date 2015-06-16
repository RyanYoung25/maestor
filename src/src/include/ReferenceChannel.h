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
 * ReferenceChannel.h
 *
 *  Created on: Oct 17, 2013
 *      Author: maestro
 */

#ifndef REFERENCECHANNEL_H_
#define REFERENCECHANNEL_H_

#include <string>
#include <iostream>
//Hubo-Ach Includes
#include <stdint.h>
#include <sys/types.h>
#include "Singleton.h"
#include "ach.h"
#include "hubo.h"
#include <iostream>

using std::string;
using std::cout;
using std::endl;

class ReferenceChannel : public Singleton<ReferenceChannel> {
    friend class Singleton<ReferenceChannel>;

public:

    typedef ach_channel_t AchChannel;
    typedef struct hubo_ref Reference;
    typedef hubo_mode_type_t Mode;

private:
    static const char *urdf_joint_names[];

    AchChannel huboReferenceChannel;
    Reference currentReference;

    bool errored;

    int indexLookup(string &joint);

protected:
    ReferenceChannel();

public:

    ~ReferenceChannel();

    void load(); // Load most recent data
    void setReference(string &joint, double rad, hubo_mode_type_t mode);
    void update(); // Save modified data
};

#endif /* REFERENCECHANNEL_H_ */
