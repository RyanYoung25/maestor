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

FTSensorBoard::FTSensorBoard(){
    boardNum = -1;
}

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

bool FTSensorBoard::set(PROPERTY property, double value){
    return false;
}

void FTSensorBoard::setBoardNum(int boardNum){
    this->boardNum = boardNum;
}
