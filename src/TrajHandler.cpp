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
 * The class that handles all of the trajectories
 *
 *  Created on: Feb 4, 2014
 *      Author: solisknight
 */

#include "TrajHandler.h"

/**
 * Create the Trajectory Handler object
 */
TrajHandler::TrajHandler() {}

/**
 * Clean up the trajectory handler object
 */
TrajHandler::~TrajHandler() {
    TrajectoryMap::iterator it;
    for (it = loaded.begin(); it != loaded.end(); it++)
        delete it->second;
}

/**
 * Load a trajectory into MAESTOR
 * @param  name The name to give the trajectory that you load
 * @param  path The path to the file that contains the trajectory data
 * @param  read Flag that if is true reads the file, if false it write to it. 
 * @return      True on success
 */
bool TrajHandler::loadTrajectory(const string& name, const string& path, bool read){
    if (!running.empty()){
        cout << "Error. A trajectory is currently running. Please stop it first." << endl;
        return false;
    }

    Trajectory* traj = new Trajectory(path, read);
    if (traj->is_open()){
        if (loaded.count(name) == 1){
            cout << "A previous trajectory named " << name << " was loaded. Removing it from memory." << endl;
            delete loaded[name];
        }

        loaded[name] = traj;
        return true;
    }


    delete traj;
    cout << "Error. Trajectory file nonexistent, or encountered error initializing. Aborting." << endl;
    return false;
}

/**
 * Ignore a joint column from a specified trajectory 
 * @param  name The name of the trajectory to ignore the joint in
 * @param  col  The joint to ignore in the trajectory
 * @return      True on success
 */
bool TrajHandler::ignoreFrom(const string& name, const string& col){
    if (loaded.count(name) != 1){
        cout << "A trajectory with name " << name << " is not loaded." << endl;
        return false;
    }
    loaded[name]->disableJoint(col);
    return true;
}

/**
 * Ignore all of the joints in a trajectory
 * @param  name The name of the trajectory. 
 * @return      True on success
 */
bool TrajHandler::ignoreAllFrom(const string& name){
    if (loaded.count(name) != 1)
        return false;

    Header header = loaded[name]->getHeader();
    for (int j = 0; j < header.size(); j++)
        ignoreFrom(name, header[j]);

    return true;
}

/**
 * Unignore a joint column from a specified trajectory
 * @param  name The name the trajectory to ignore the joint in
 * @param  col  The name of the joint to ignore in the trajectory 
 * @return      True on success
 */
bool TrajHandler::unignoreFrom(const string& name, const string& col){
    if (loaded.count(name) != 1){
        cout << "A trajectory with name " << name << " is not loaded." << endl;
        return false;
    }
    loaded[name]->enableJoint(col);
    return true;
}

bool TrajHandler::unignoreAllFrom(const string& name){
    if (loaded.count(name) != 1)
            return false;

    Header header = loaded[name]->getHeader();
    for (int j = 0; j < header.size(); j++)
        unignoreFrom(name, header[j]);

    return true;
}

/**
 * Set a trigger to start another trajectory at a specified frame in the first 
 * trajectory. 
 * @param  traj   The first trajectory where the trigger will be set
 * @param  frame  The frame to trigger the second trajectory
 * @param  target The second trajectory that will be played at the trigger point
 * @return        True on success
 */
bool TrajHandler::setTrigger(const string &traj, int frame, const string& target){
    if (loaded.count(traj) != 1){
        cout << "No trajectory with name " << traj << " is loaded." << endl;
        return false;
    }

    for (int i = 0; i < running.size(); i++){
        if (running[i].compare(traj) == 0){
            cout << "Error. Trajectory " << traj << " has already been started. Cannot set trigger." << endl;
            return false;
        }
    }

    return loaded[traj]->addTrigger(frame, target);

}

/**
 * Extend a loaded trajectory by the contents of a trajectory file
 * @param  name The loaded trajectory to extend 
 * @param  path The path to the extension file
 * @return      True on success
 */
bool TrajHandler::extendTrajectory(const string& name, const string& path){
    if (!running.empty()){
        cout << "Error. A trajectory has been started. Please stop it first." << endl;
        return false;
    }

    if (loaded.count(name) != 1){
        cout << "No trajectory with name " << name << " is loaded. Loading this trajectory by itself." << endl;
        return loadTrajectory(name, path, true);
    }

    return loaded[name]->extendTrajectory(path);
}

/**
 * Start the trajectory
 * @param name The trajectory to start
 */
void TrajHandler::startTrajectory(const string& name){
    Trajectory* traj = NULL;

    if (loaded.count(name) != 1){
        cout << "Cannot start trajectory " << name << ". Not yet loaded." << endl;
        return;
    }

    traj = loaded[name];

    if (!traj->is_open()){
        cout << "Cannot start non-open trajectory " << name << "." << endl;
        return;
    }

    string key(WRITE_KEY);
    if (inRunning(key) && !traj->read_only()){
        cout << "Cannot start " << name << ". Another write-enabled trajectory is running." << endl;
        return;
    }

    Header header;
    for (vector< string >::iterator it = running.begin(); it != running.end(); it++){
        while (loaded.count(*it) != 1){
            it = running.erase(it);
            if (it == running.end())
                break;
        }

        if (name.compare(*it) == 0){
            cout << "Trajectory with name " << name << " is already running." << endl;
            return;
        }

        if (traj->read_only()){
            header = loaded[*it]->getHeader();
            for (int j = 0; j < header.size(); j++){
                if (loaded[*it]->contains(header[j]) && traj->contains(header[j])){
                    cout << "Cannot start trajectory: references " << header[j] << ", already in use by another trajectory. " << endl;
                    return;
                }
            }
        }
    }
    string trigger;
    if (traj->getTrigger(trigger))
        triggers.push(trigger);
    addToCache(traj);
    running.push_back(name);
}

/**
 * Stop the trajectory
 * @param traj The trajectory to stop as a Trajectory object 
 */
void TrajHandler::stopTrajectory(Trajectory* traj){
    for(TrajectoryMap::iterator it = loaded.begin(); it != loaded.end(); it++){
        if (it->second == traj){
            string key(it->first);
            stopTrajectory(key);
            return;
        }
    }
}

/**
 * Stop a trajectory. This takes the name of the trajectory as a name.
 * @param name The name of the trajectory to stop 
 */
void TrajHandler::stopTrajectory(const string& name){
    if (loaded.count(name) != 1){
        cout << "Trajectory " << name << " does not exist." << endl;
        return;
    }
    for (vector<string>::iterator it = running.begin(); it != running.end(); it++){
        while (loaded.count(*it) != 1){
            it = running.erase(it);
            if (it == running.end())
                break;
        }
        if (name.compare(*it) == 0){
            removeFromCache(loaded[*it]);
            loaded[*it]->reset();
            it = running.erase(it);
            return;
        }
    }
}

/**
 * Advance a frame on the current running trajectory 
 */
void TrajHandler::advanceFrame(){
    for (int i = 0; i < running.size(); i++){
        if (!loaded[running[i]]->advanceFrame()){
            string trigger;
            if (loaded[running[i]]->getTrigger(trigger))
                triggers.push(trigger);

            stopTrajectory(running[i]);
            i--;
        } else {
            string trigger;
            if (loaded[running[i]]->getTrigger(trigger))
                triggers.push(trigger);
        }
    }
}

/**
 * Returns true if there is a running trajectory
 * @return True if there is a trajectory running
 */
bool TrajHandler::hasRunning(){
    return !running.empty();
}

/**
 * Get a trajectory object from the name of the trajectory that is loaded
 * @param  name Name of the loaded trajectory
 * @return      The trajectory object 
 */
Trajectory* TrajHandler::get(const string& name){
    if (loaded.count(name) == 1)
        return loaded[name];
    return NULL;
}

/**
 * Return a trajectory that is currently operating a specific joint
 * @param  col The name of the joint that is being operated 
 * @return     Trajectory object that was running
 */
Trajectory* TrajHandler::inRunning(const string& col){
    if (cache.count(col) == 1)
        return cache[col];
    return NULL;
}

/**
 * Return all of the running trajectoies
 */
const vector< string >& TrajHandler::getRunning(){
    return running;
}

/**
 * Return all of the triggers
 */
queue< string >& TrajHandler::getCurrentTriggers(){
    return triggers;
}

/**
 * Add a trajectory to the cache
 * @param traj The trajectory to add to the cache
 */
void TrajHandler::addToCache(Trajectory* traj){
    if (traj->read_only()){
        Header header = traj->getHeader();
        for (Header::iterator it = header.begin(); it != header.end(); it++)
            if (traj->contains(*it))
                cache[*it] = traj;
    } else
        cache[WRITE_KEY] = traj;
}

/**
 * Remove a trajectory from the cache
 * @param traj The trajectory object to remove to the cache
 */
void TrajHandler::removeFromCache(Trajectory* traj){
    for (TrajectoryMap::iterator it = cache.begin(); it != cache.end(); it++){
        if (it->second == traj)
            cache.erase(it);
    }
}
