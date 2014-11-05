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

/*
 * Trajectory.h
 *
 *  Created on: Sep 23, 2013
 *      Author: maestro
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <sstream>
#include <math.h>

#include "WSVFile.h"

#define BUFFER_SIZE 1
#define DEFAULT_HEADER "RHY RHR RHP RKP RAP RAR LHY LHR LHP LKP LAP LAR RSP RSR RSY REP RWY RWR RWP LSP LSR LSY LEP LWY LWR LWP NKY NK1 NK2 WST RF1 RF2 RF3 RF4 RF5 LF1 LF2 LF3 LF4 LF5"

using std::string;
using std::vector;
using std::set;

class Trajectory {

public:

    typedef WSVFile::Frame Frame;
    typedef WSVFile::Buffer Buffer;
    typedef WSVFile::Header Header;
    typedef WSVFile::HeaderMap HeaderMap;

    Trajectory(const string &baseFile, bool read);
    ~Trajectory();

    /**
     * Attempts to load the Trajectory file in path 'filename' and add it to the current Trajectory.
     *
     * Will check:
     *  - Whether the Trajectory loaded properly
     *  - Whether it has the same number of columns as the current trajectory
     *  - Whether all columns in the current Trajectory are present in the new Trajectory
     *  - Whether the end position of the current Trajectory is consistent with the start position in the new Trajectory (to within .01 rad)
     *  - Whether the buffer in the new Trajectory could be filled.
     */
    bool extendTrajectory(const string &filename);

    /**
     * Returns the first value in the first WSVFile associated with this Trajectory for column 'joint' if available.
     * Will mark the Trajectory as closed and return 0 on any error. (This will not close the file.)
     */
    double startPosition(const string &joint);

    /**
     * Accesses the position at the current frame in this Trajectory.
     * In read-only Trajectories, the value for column 'joint' at the current frame is placed in position if available.
     * In write-enabled Trajectories, the value of 'position' is written to the buffer in column 'joint'.
     *
     * Should the Trajectory be errored in any way, or if the current frame does not exist in the buffer, this method will return false.
     *
     * @param joint: joint to query next position from
     * @param position: location to store the next position. Will not be modified if failed read.
     * @return true if read succeeded.
     */
    bool nextPosition(const string &joint, double& position);

    /**
     * Advances to the next "frame" in the WSVFile buffer.
     * In write-enabled Trajectories, a new frame is added to the buffer for writing.
     *
     * In read-enabled Trajectories, should the buffer be exhausted, a new one is loaded from the file.
     * In write-enabled Trajectories, should the buffer be full, the buffer is written to the file and cleared. A new frame is added to the new buffer.
     */
    bool advanceFrame();

    /**
     * Returns whether this trajectory can be read from (Whether advanceFrame() or nextPosition() will do anything, pretty much)
     * This can be fixed by reset()
     */
    bool is_open();

    /**
     * Returns whether this trajectory has been opened for reading.
     */
    bool read_only();

    /**
     * Queries the Trajectory for whether there exists another file to load once the current file has been exhausted.
     */
    bool hasNext();

    int getFrame();

    /**
     * Associates the start of another trajectory with name 'target' with frame 'frame'
     */
    bool addTrigger(int frame, const string &target);

    /**
     * Fills 'target' with the name of the trajectory associated with the current frame, if it exists.
     * If such an association exists, returns true.
     */
    bool getTrigger (string &target);

    /**
     * Queries the Trajectory for the existence of column 'joint' in its header. Also takes into disabled joints into account.
     * This method will not return true if 'joint' exists, but has been disabled.
     */
    bool contains(const string &joint);

    /**
     * Adds 'joint' to the set of columns which should be ignored.
     * Has no effect if 'joint' is already in the set of columns which are ignored.
     */
    void disableJoint(const string &joint);

    /**
     * Removes 'joint' from the set of columns which should be ignored.
     * Has no effect if 'joint' is not in the set of columns.
     */
    void enableJoint(const string &joint);

    /**
     * Returns this Trajectory to its initial state. Does not clear errors. (If this trajectory had an error for any reason, this will not fix it.)
     * For write-enabled Trajectories, this will write the remaining buffer to the file, close it, and re-open the trajectory as a read-only Trajectory.
     */
    bool reset();

    /**
     * Manually sets the header for the WSVFile. This will determine the number of columns in the file.
     * Only works in write-enabled Trajectories.
     */
    void setHeader(const string &header);

    /**
     * Manually sets the header for the WSVFile. This will determine the number of columns in the file.
     * Only works in write-enabled Trajectories.
     */
    void setHeader(const Header &header);

    /**
     * Returns the header of the current WSVFile. (This is not guaranteed to always be the same, but it is guaranteed to always have the same members.
     */
    const Header& getHeader();

private:

    /**
     * Adds another 'frame' (array of values) to the WSVFile write-buffer
     * Only works in write-enabled Trajectories.
     */
    void prepareFrame();

    vector< WSVFile* > files; // Sequential list of WSVFiles which make up the trajectory
    set< string > disabledJoints; // Set of joints which, for all intents and purposes, are not in this trajectory (even if they are in the header)
    map< int, string > triggers; // Association between frames of a trajectory and the start of another trajectory
    int bufferIndex;
    int frame;
    int currentWSV;

    bool open;
    bool read;

};

#endif /* TRAJECTORY_H_ */
