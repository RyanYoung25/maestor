#ifndef SIMCHANNELS_H_
#define SIMCHANNELS_H_

#include <string>
#include <iostream>

//Hubo-Ach Includes
#include <stdint.h>
#include <sys/types.h>
#include "ach.h"
#include "hubo.h"
#include "Singleton.h"

class SimChannels : public Singleton<SimChannels>{
    friend class Singleton<SimChannels>;

public:
    typedef ach_channel_t AchChannel;
    typedef hubo_virtual_t Virtual;

private:
    	AchChannel huboToSimChannel;
        AchChannel huboFromSimChannel;
    	hubo_virtual_t H_virtual;

public:
        void load();

protected:
    	SimChannels();
        ~SimChannels();

};

#endif /* SIMCHANNELS_H_*/
