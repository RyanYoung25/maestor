/*
 * CommandChannel.h
 *
 *  Created on: Oct 15, 2013
 *      Author: maestro
 */

#ifndef COMMANDCHANNEL_H_
#define COMMANDCHANNEL_H_

#include <string>
#include <iostream>
//Hubo-Ach Includes
#include <stdint.h>
#include <sys/types.h>
#include "Singleton.h"
#include "ach.h"
#include "hubo.h"

using std::string;
using std::cerr;
using std::cout;
using std::endl;


class CommandChannel : public Singleton<CommandChannel>{
    friend class Singleton<CommandChannel>;
private:

    typedef ach_channel_t AchChannel;
    typedef hubo_board_cmd_t BoardCommand;

    static const char *urdf_joint_names[];

    AchChannel huboBoardCommandChannel;


    int indexLookup(std::string &joint);

protected:
    CommandChannel();
    //void init();

public:

    ~CommandChannel();

    bool enable(string joint = "all");
    bool disable(string joint = "all");
    bool home(string joint = "all");
    bool reset(string &joint);
    bool initializeSensors();

};

#endif /* COMMANDCHANNEL_H_ */
