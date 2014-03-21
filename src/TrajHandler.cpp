/*
 * TrajHandler.cpp
 *
 *  Created on: Feb 4, 2014
 *      Author: solisknight
 */

#include "TrajHandler.h"

TrajHandler::TrajHandler() {}

TrajHandler::~TrajHandler() {
    TrajectoryMap::iterator it;
    for (it = loaded.begin(); it != loaded.end(); it++)
        delete it->second;
}

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

bool TrajHandler::ignoreFrom(const string& name, const string& col){
    if (loaded.count(name) != 1){
        cout << "A trajectory with name " << name << " is not loaded." << endl;
        return false;
    }
    loaded[name]->disableJoint(col);
    return true;
}

bool TrajHandler::ignoreAllFrom(const string& name){
    if (loaded.count(name) != 1)
        return false;

    Header header = loaded[name]->getHeader();
    for (int j = 0; j < header.size(); j++)
        ignoreFrom(name, header[j]);

    return true;
}

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

void TrajHandler::stopTrajectory(Trajectory* traj){
    for(TrajectoryMap::iterator it = loaded.begin(); it != loaded.end(); it++){
        if (it->second == traj){
            string key(it->first);
            stopTrajectory(key);
            return;
        }
    }
}

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

bool TrajHandler::hasRunning(){
    return !running.empty();
}

Trajectory* TrajHandler::get(const string& name){
    if (loaded.count(name) == 1)
        return loaded[name];
    return NULL;
}

Trajectory* TrajHandler::inRunning(const string& col){
    if (cache.count(col) == 1)
        return cache[col];
    return NULL;
}

const vector< string >& TrajHandler::getRunning(){
    return running;
}

queue< string >& TrajHandler::getCurrentTriggers(){
    return triggers;
}

void TrajHandler::addToCache(Trajectory* traj){
    if (traj->read_only()){
        Header header = traj->getHeader();
        for (Header::iterator it = header.begin(); it != header.end(); it++)
            if (traj->contains(*it))
                cache[*it] = traj;
    } else
        cache[WRITE_KEY] = traj;
}

void TrajHandler::removeFromCache(Trajectory* traj){
    for (TrajectoryMap::iterator it = cache.begin(); it != cache.end(); it++){
        if (it->second == traj)
            cache.erase(it);
    }
}
