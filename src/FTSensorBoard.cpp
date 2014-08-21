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
#include "FTSensorBoard.h"

/**
 * Create a Force Torque sensor board
 */
FTSensorBoard::FTSensorBoard(){
    boardNum = -1;
}

/**
 * Get the property off of the force torque sensor. Directly from hubo ach. 
 * @param  property The property to get
 * @param  value    A pointer to store the value in 
 * @return          True on success
 */
bool FTSensorBoard::get(PROPERTY property, double& value){
    switch (property){
    case M_X:
    case M_Y:
    case F_Z:
        // Chooses whether to use the name of the board or the board number to request the property from the state channel
        // Prints an error if the request fails
        if ( ( boardNum != -1 ? !stateChannel->getFTProperty(boardNum, property, value) : !stateChannel->getFTProperty(getName(), property, value) ) ){
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
 * Set a property on the force torque sensor. But there are none for you to set
 * so you can't. :( sorry
 * @param  property The property that you wanted to set. 
 * @param  value    The value you wanted to set it to.
 * @return          Sadness
 */
bool FTSensorBoard::set(PROPERTY property, double value){
    return false;
}

/**
 * Set the board number of the FT board
 * @param boardNum The number you want to set it to. 
 */
void FTSensorBoard::setBoardNum(int boardNum){
    this->boardNum = boardNum;
}
