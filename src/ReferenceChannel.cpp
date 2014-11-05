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
 * The hubo-ach Reference Ach channel. This is where we set joint references for hubo-ach 
 * to command the joint to. 
 */

#include "../include/ReferenceChannel.h"

/**
 * list of all the joints 
 */
const char *ReferenceChannel::urdf_joint_names[] = {
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
 * Constructor that initializes the ach channel
 */
ReferenceChannel::ReferenceChannel() {
    errored = false;

    int r = ach_open(&huboReferenceChannel, HUBO_CHAN_REF_NAME, NULL);
    if (ACH_OK != r && ACH_MISSED_FRAME != r && ACH_STALE_FRAMES != r){
        cout << "Error! Reference Channel failed with state " << r << endl;
        errored = true;
    }

}

/**
 * Destructor
 */
ReferenceChannel::~ReferenceChannel() {}

/**
 * Look up the index of a joint from the name of the joint
 * @param  joint Name of the joint
 * @return       The index of the joint
 */
int ReferenceChannel::indexLookup(string &joint) {
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
 * Load the reference channel
 */
void ReferenceChannel::load(){
    if (errored) return;
    Reference temp;
    memset( &temp, 0, sizeof(temp));
    size_t fs;

    int r = ach_get(&huboReferenceChannel, &temp, sizeof(temp), &fs, NULL, ACH_O_LAST);

    if(ACH_OK != r && ACH_MISSED_FRAME != r && ACH_STALE_FRAMES != r) {
        cout << "Error! Reference Channel failed with state " << r << endl;
        errored = true;
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
 * Set the reference of a joint to a position in radians
 * 
 * @param joint The joint to set 
 * @param rad   The position in radians
 * @param mode  The mode of joint operation
 */
void ReferenceChannel::setReference(string &joint, double rad, Mode mode){
	if (errored) return;
	int index = indexLookup(joint);
	if (index != -1){
		currentReference.ref[index] = rad;
        currentReference.mode[index] = 1;
    }
}

/**
 * Update the reference channel
 */
void ReferenceChannel::update(){
    if (errored) return;
    ach_put(&huboReferenceChannel, &currentReference, sizeof(currentReference));
}
