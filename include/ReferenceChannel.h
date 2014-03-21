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
