#include "../include/SimChannels.h"

SimChannels::SimChannels(){
	memset(&H_virtual, 0, sizeof(H_virtual));
    
    int r = ach_open(&huboToSimChannel, HUBO_CHAN_VIRTUAL_TO_SIM_NAME, NULL);
    if (ACH_OK != r)
        std::cerr << "Error! To Sim Channel failed with state " << r << std::endl;
   
    r = ach_open(&huboFromSimChannel, HUBO_CHAN_VIRTUAL_FROM_SIM_NAME, NULL);
    if (ACH_OK != r)
        std::cerr << "Error! From Sim Channel failed with state " << r << std::endl;
}

SimChannels::~SimChannels(){}

void SimChannels::load(){
	size_t fs;
	memset( &H_virtual, 0, sizeof(H_virtual));

    int r = ach_get(&huboFromSimChannel, &H_virtual, sizeof(H_virtual), &fs, NULL, ACH_O_WAIT);
}
