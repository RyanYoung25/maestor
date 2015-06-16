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
#include "MotorBoard.h"
#include <iostream>

/**
 * Create a motorboard object
 */
MotorBoard::MotorBoard(){
    MotorBoard(DEFAULT_CHANNELS);
}

/**
 * Create a motor board with a specified number of channels
 *
 * @param channels  The number of channels to make on the motor board
 */
MotorBoard::MotorBoard(int channels){
    this->motors = vector<HuboMotor*>(channels);
    this->channels = channels;
}

/**
 * Make a motor board from a motor board reference
 *
 * @param rhs   The motor board reference
 */
MotorBoard::MotorBoard(const MotorBoard& rhs){
    this->channels = rhs.channels;
    this->motors = rhs.motors;
}

/**
 * Add a motor to the motorboard
 * @param motor   The motor to add
 * @param channel The channel to add it to
 */
void MotorBoard::addMotor(HuboMotor* motor, int channel){
    //std::cout << "added motor to position: " << &(this->motors[channel]) << std::endl;
    this->motors[channel] = motor; 
}

/**
 * Remove a motor from the motor board
 * @param motor The motor to remove
 */
void MotorBoard::removeMotor(HuboMotor* motor){
    for (int i = 0; i < this->channels; i++){
        if (this->motors[i] == motor){
            this->motors[i] = NULL;
        }
    }  
}

/**
 * Remove a motor from the motor board by it's channel number
 * @param channel The channel number of the motor to remove
 */
void MotorBoard::removeMotor(int channel){
    this->motors[channel] = NULL;
}

/**
 * Get the motor by it's channel number
 * @param  channel The channel number 
 * @return         The motor that corresponds to that channel
 */
HuboMotor* MotorBoard::getMotorByChannel(int channel){
    return this->motors[channel];
}

/**
 * Get the number of channels on the motor board
 * @return The number of channels on the board
 */
int MotorBoard::getNumChannels(){
    return channels;
}

/**
 * Check if any motors on the board require motion. If they do return true
 * @return True if motors on the board require motion
 */
bool MotorBoard::requiresMotion(){
    for (int i = 0; i < channels; i++)
        if (motors[i]->requiresMotion()) return true;
    return false;
}

/**
 * Check if a particular motor on the board requires motion 
 * @param  channel The channel of the motor
 * @return         True if the motor requires motion
 */
bool MotorBoard::requiresMotion(int channel){
    assert(channel < channels);
    return motors[channel]->requiresMotion();
}
