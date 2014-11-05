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
 * TrajHandler.h
 *
 *  Created on: Feb 4, 2014
 *      Author: solisknight
 */

#include <map>
#include <set>
#include <vector>
#include <queue>
#include <string>
#include <iostream>

#include "Trajectory.h"

using std::map;
using std::set;
using std::vector;
using std::queue;
using std::cout;
using std::endl;
using std::string;

#ifndef TRAJHANDLER_H_
#define TRAJHANDLER_H_

#define WRITE_KEY "WRITE"

class TrajHandler {
private:
    typedef map< string, Trajectory* > TrajectoryMap;
    typedef Trajectory::Header Header;

public:

    TrajHandler();
    virtual ~TrajHandler();

    bool loadTrajectory(const string &name, const string& path, bool read);
    bool ignoreFrom(const string &name, const string &col);
    bool ignoreAllFrom(const string &name);
    bool unignoreFrom(const string &name, const string &col);
    bool unignoreAllFrom(const string& name);
    bool extendTrajectory(const string &name, const string &path);
    bool setTrigger(const string &traj, int frame, const string &target);
    void startTrajectory(const string& name);
    void advanceFrame();

    bool hasRunning();
    Trajectory* get(const string& name);
    Trajectory* inRunning(const string& col);
    const vector< string >& getRunning();
    queue< string >& getCurrentTriggers();

    void stopTrajectory(Trajectory* traj);
    void stopTrajectory(const string& name);

private:

    void addToCache(Trajectory* traj);
    void removeFromCache(Trajectory* traj);

    TrajectoryMap loaded;
    TrajectoryMap cache;
    vector< string > running;
    queue< string > triggers;
};

#endif /* TRAJHANDLER_H_ */
