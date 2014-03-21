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
