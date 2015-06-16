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
#include "IMUBoard.h"

/**
 * Constructor for the IMU Board
 */
IMUBoard::IMUBoard(){
    boardNum = -1;
}

/**
 * Get a property from this specific IMU board.
 * @param  property The property to get
 * @param  value    A pointer to store the value in 
 * @return          True on success
 */
bool IMUBoard::get(PROPERTY property, double &value){
    switch (property){
    case X_ACCEL:
    case Y_ACCEL:
    case Z_ACCEL:
    case X_ROTAT:
    case Y_ROTAT:
        if ( ( boardNum != -1 ? !stateChannel->getIMUProperty(boardNum, property, value) : !stateChannel->getIMUProperty(getName(), property, value) ) ){
            cout << "Error getting " << Names::getName(property) << " from " << getName() << endl;
            return false;
        }
        break;
    default:
        return false;
    }
    return true;
}

/**
 * There are no properties to set on an IMU but it needs to be here
 * @param  property 
 * @param  value    
 * @return          False. Always
 */
bool IMUBoard::set(PROPERTY property, double value){
    return false;
}

/**
 * Set the IMU board number
 * @param boardNum The new number to set the IMU board number to
 */
void IMUBoard::setBoardNum(int boardNum){
    this->boardNum = boardNum;
}


