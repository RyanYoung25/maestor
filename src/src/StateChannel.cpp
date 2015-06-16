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

/**
 *  The ACH State Channel. This is used to get all of the robot information from hubo-ach. 
 */

#include "StateChannel.h"
//The array of all the joint names
const char *StateChannel::urdf_joint_names[] = {
        "WST", "NKY", "NK1", "NK2",
        "LSP", "LSR", "LSY", "LEP", "LWY", "LWR", "LWP",
        "RSP", "RSR", "RSY", "REP", "RWY", "RWR", "RWP",
        "UNUSED1",
        "LHY", "LHR", "LHP", "LKP", "LAP", "LAR",
        "UNUSED2",
        "RHY", "RHR", "RHP", "RKP", "RAP", "RAR",
        "RF1", "RF2", "RF3", "RF4", "RF5",
        "LF1", "LF2", "LF3", "LF4", "LF5",
        "unknown1", "unknown2", "unknown3", "unknown4", "unknown5", "unknown6", "unknown7", "unknown8"};

/**
 * Look up a joint's index by it's joint name
 * @param  joint Joint name
 * @return       index of the joint in hubo-ach
 */
int StateChannel::indexLookup(string &joint) {
    if (joint.length() != 3)
        return -1;
    int best_match = -1;

    for (int i = 0; i < HUBO_JOINT_COUNT; i++) {
        if (strcmp(joint.c_str(), urdf_joint_names[i]) == 0)
            best_match = i;
    }
    return best_match;
}

/**
 * Constructor to open up the state channel. 
 */
StateChannel::StateChannel() {
    errored = false;
    int r = ach_open(&huboStateChannel, HUBO_CHAN_STATE_NAME, NULL);
    if (ACH_OK != r && ACH_MISSED_FRAME != r && ACH_STALE_FRAMES != r){
        cout << "Error! State Channel failed with state " << r << endl;
        errored = true;
    }
}

/**
 * Destructor 
 */
StateChannel::~StateChannel() { }

/**
 * Load up the state channel. This reads everything off of the channel each time.
 */
void StateChannel::load(){
    if (errored)
        return;

    State temp;
    memset( &temp, 0, sizeof(temp));
    size_t fs;
    int r = ach_get(&huboStateChannel, &temp, sizeof(temp), &fs, NULL, ACH_O_LAST);

    if(ACH_OK != r && ACH_MISSED_FRAME != r && ACH_STALE_FRAMES != r) {
        cout << "Error! State Channel failed with state " << r << endl;
        errored = true;
        return;
    } else if (ACH_STALE_FRAMES != r){
        if (sizeof(temp) != fs) {
            cout << "Error! File size inconsistent with state struct! fs = " << fs << " sizeof currentReference: " << sizeof(temp) << endl;
            errored = true;
            return;
        }
    } else
        return;
    currentReference = temp;
}

/**
 * Get a motor property off of the state channel data. This is a wraper to the real 
 * getMotorProperty that takes an integer board number. This takes a string joint
 * name. 
 * 
 * @param  name     Joint name
 * @param  property The property to get
 * @param  result   A pointer to store the result in. 
 * @return          True on success
 */
bool StateChannel::getMotorProperty(string &name, PROPERTY property, double &result){
    if (errored)
        return false;

    return getMotorProperty(indexLookup(name), property, result);
}

/**
 * The real getMotorProperty that gets the motor property off of the state channel data. This takes
 * a board number instead of a name because hubo-ach doesn't deal with joint names. 
 * 
 * @param  board    The board number of the joint that you want to access
 * @param  property The property you want to get 
 * @param  result   A pointer to store the result in
 * @return          True on success
 */
bool StateChannel::getMotorProperty(int board, PROPERTY property, double &result){
    if (errored || board < 0 || board >= HUBO_JOINT_COUNT)
        return false;

    switch (property){
    case POSITION:
        result = currentReference.joint[board].pos;
        break;
    case GOAL:
        result = currentReference.joint[board].ref;
        break;
    case VELOCITY:
        result = currentReference.joint[board].vel;
        break;
    case TEMPERATURE:
        result = currentReference.joint[board].tmp;
        break;
    case CURRENT:
        result = currentReference.joint[board].cur;
        break;
    case ENABLED:
        result = currentReference.joint[board].active;
        break;
    case HOMED:
        result = currentReference.status[board].homeFlag;
        break;
    case ERRORED:
        result = currentReference.status[board].jam
        || currentReference.status[board].pwmSaturated
        || currentReference.status[board].bigError
        || currentReference.status[board].encError
        || currentReference.status[board].driverFault
        || currentReference.status[board].posMinError
        || currentReference.status[board].posMaxError
        || currentReference.status[board].velError
        || currentReference.status[board].accError
        || currentReference.status[board].tempError;
        break;
    case JAM_ERROR:
        result = currentReference.status[board].jam;
        break;
    case PWM_SATURATED_ERROR:
        result = currentReference.status[board].pwmSaturated;
        break;
    case BIG_ERROR:
        result = currentReference.status[board].bigError;
        break;
    case ENC_ERROR:
        result = currentReference.status[board].encError;
        break;
    case DRIVE_FAULT_ERROR:
        result = currentReference.status[board].driverFault;
        break;
    case POS_MIN_ERROR:
        result = currentReference.status[board].posMinError;
        break;
    case POS_MAX_ERROR:
        result = currentReference.status[board].posMaxError;
        break;
    case VELOCITY_ERROR:
        result = currentReference.status[board].velError;
        break;
    case ACCELERATION_ERROR:
        result = currentReference.status[board].accError;
        break;
    case TEMP_ERROR:
        result = currentReference.status[board].tempError;
        break;
    default:
        return false;
    }
    return true;
}

/**
 * Get the property off of an IMU rather than a joint. This is a wrapper for the one that takes
 * the board number. This takes a name for the IMU. 
 * @param  name     Name of the IMU
 * @param  property The property that you want to get 
 * @param  result   A pointer to store the result in
 * @return          True on success
 */
bool StateChannel::getIMUProperty(string &name, PROPERTY property, double& result){
    if (errored)
        return false;

    int index = -1;
    if (strcmp(name.c_str(), "IMU") == 0)
        index = BODY_IMU;
    else if (strcmp(name.c_str(), "LAI") == 0)
        index = LEFT_IMU;
    else if (strcmp(name.c_str(), "RAI") == 0)
        index = RIGHT_IMU;

    return getIMUProperty(index, property, result);
}

/**
 * Get the property off of an IMU. This one takes the board number and is the function that 
 * reads from the state data. 
 * @param  board    Board number of the IMU 
 * @param  property Property that you want to get
 * @param  result   A pointer to store the result in 
 * @return          True on success
 */
bool StateChannel::getIMUProperty(int board, PROPERTY property, double& result){
    if (errored || board < 0 || board >= HUBO_IMU_COUNT)
        return false;

    switch (property){
    case X_ACCEL:
        result = currentReference.imu[board].a_x;
        break;
    case Y_ACCEL:
        result = currentReference.imu[board].a_y;
        break;
    case Z_ACCEL:
        result = currentReference.imu[board].a_z;
        break;
    case X_ROTAT:
        result = currentReference.imu[board].w_x;
        break;
    case Y_ROTAT:
        result = currentReference.imu[board].w_y;
        break;
    default:
        return false;
    }
    return true;
}

/**
 * Get the property of a Force Torque sensor. This one is a wrapper for the function
 * that takes the board number. 
 * 
 * @param  name     Name of the FT sensor
 * @param  property Property that you want to get
 * @param  result   A pointer to store the result in. 
 * @return          True on success
 */
bool StateChannel::getFTProperty(string &name, PROPERTY property, double& result){
    if (errored)
        return false;

    int index = -1;
    if (strcmp(name.c_str(), "LAT") == 0)
        index = FT_LA;
    else if (strcmp(name.c_str(), "RAT") == 0)
        index = FT_RA;
    else if (strcmp(name.c_str(), "LWT") == 0)
        index = FT_LW;
    else if (strcmp(name.c_str(), "RWT") == 0)
        index = FT_RW;

    return getFTProperty(index, property, result);
}

/**
 * Get the property of a Force Torque sensor. This is the one that actually 
 * accesses the state data. It takes a board number instead of a string name. 
 * 
 * @param  board    Board number of the FT sensor
 * @param  property Property that you want to get
 * @param  result   A pointer to store the result in
 * @return          True on success
 */
bool StateChannel::getFTProperty(int board, PROPERTY property, double& result){
    if (errored || board < 0 || board >= HUBO_FT_COUNT)
        return false;

    switch (property){
    case M_X:
        result = currentReference.ft[board].m_x;
        break;
    case M_Y:
        result = currentReference.ft[board].m_y;
        break;
    case F_Z:
        result = currentReference.ft[board].f_z;
        break;
    default:
        return false;
    }
    return true;
}