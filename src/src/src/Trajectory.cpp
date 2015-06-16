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
 * A trajectory object. This stores all of the frames of a trajectory that is loaded
 * and allows you to do opertations on them.
 */

#include "Trajectory.h"

/**
 * Create a trajectory object from a file or open a file to be written to if read is false. 
 * @param   baseFile    The file to open and either read or write to
 * @param   read        Flag to specify if the file it to be written to or read from
 */
Trajectory::Trajectory(const string &baseFile, bool read){
    bufferIndex = 0;
    frame = 0;
    currentWSV = 0;
    open = true;
    this->read = read;

    WSVFile* file = new WSVFile(baseFile, read, BUFFER_SIZE);
    if (file->errored()){
        cout << "Error initializing trajectory file " << baseFile << endl;
        cout << file->getError();
        delete file; // Current method of handling error in constructor
        file = NULL;
        open = false;
        return;
    }

    if (read) {
        if (!file->headerSupplied() && file->numCols() == 40){
            file->setHeader(DEFAULT_HEADER);
        }

        if (file->loadBuffer())
            files.push_back(file);
        else {
            cout << "Error loading buffer for trajectory file " << baseFile << endl;
            if (file->errored())
                cout << file->getError() << endl;
            delete file; // current method of handling error in constructor
            file = NULL;
            open = false;
            return;
        }
    } else {
        files.push_back(file);
        prepareFrame();
    }
}

/**
 * Clean up after the trajectory object is destructed 
 */
Trajectory::~Trajectory(){
    for (int i = 0; i < files.size(); i++){
        if (files[i]){
            delete files[i];
            files[i] = NULL;
        }
    }
}

/**
 * Extend an already loaded trajectory by the contents of a trajectory file at the specified location. 
 * 
 * @param  filename The file that has the trajectory information you want to have at the end of the current trajectory
 * @return          True on success
 */
bool Trajectory::extendTrajectory(const string &filename){
    if (!open || !read)
        return false;

    WSVFile* file = new WSVFile(filename, true, BUFFER_SIZE);
    if (file->errored()){
        cout << "Error initializing trajectory file " << filename << endl;
        delete file; // Current method of handling error in constructor
        file = NULL;
        return false;
    }

    // If the new file doesn't supply a header, and has 40 columns, assume the default header, and move along.
    if (!file->headerSupplied() && file->numCols() == 40)
        file->setHeader(DEFAULT_HEADER);

    // Nab the file at the end of the current trajectory.
    WSVFile* last = files[files.size() - 1];

    if (!last)
        return false;

    if (file->numCols() != last->numCols()){
        cout << "Column size for " << filename << " does not match existing trajectory." << endl;
        return false;
    }

    Frame lastEnd = last->end();
    Frame fileStart = file->start();

    string col;
    int fileIndex;
    double lastPos;
    double startPos;
    for (int i = 0; i < last->orderedHeader().size(); i++){
        col = last->orderedHeader()[i];
        if (file->header().count(col) != 1){
            cout << "Column " << col << " not present in " << filename << endl;
            return false;
        }

        fileIndex = file->header()[col];

        lastPos = lastEnd[i];
        startPos = fileStart[fileIndex];

        if (fabs( lastPos - startPos ) > .01){
            cout << "Start position of " << filename << " inconsistent with previous end position." << endl;
            return false;
        }
    }

    if (file->loadBuffer())
        files.push_back(file);
    else {
        cout << "Error loading buffer for trajectory file " << filename << endl;
        delete file; // current method of handling error in constructor
        file = NULL;
        return false;
    }

    return true;
}

/**
 * Go to the starting position and load it for a joint
 * @param  joint The joint you want the starting position of
 * @return       The starting position of the joint
 */
double Trajectory::startPosition(const string &joint){
    if (files.size() == 0)
        return 0;
    WSVFile* file = files[0];
    if (!file || file->errored()){
        open = false;
        return 0;
    }

    if (file && !file->errored() && contains(joint))
        return file->start()[ file->header()[joint] ];

    return 0;
}

/**
 * Get the next position for a joint
 * @param  joint    The joint you want the position of
 * @param  position A pointer to store the position of the joint
 * @return          True on success
 */
bool Trajectory::nextPosition(const string &joint, double &position){
    if (!open || files.size() <= currentWSV)
        return false;

    WSVFile* file = files[currentWSV];
    if (!file || file->errored() || !contains(joint) || bufferIndex >= file->bufferSize()){
        if (file && file->errored())
            cout << file->getError() << endl;
        return false;
    }

    if (read)
        position = file->buffer()[bufferIndex][file->header()[joint]];
    else
        file->buffer()[bufferIndex][file->header()[joint]] = position;
    return true;
}

/**
 * Advance a frame in the trajectory 
 * @return True on success
 */
bool Trajectory::advanceFrame(){
    if (!open || files.size() <= currentWSV)
        return false;
    WSVFile* file = files[currentWSV];
    if (!file || file->errored()){
        if (file && file->errored())
            cout << "Error on advance frame: " << file->getError() << endl;
        return false;
    }

    bufferIndex++;
    frame++;

    // Prepares a frame for writing.
    if (!read)
        prepareFrame();

    if (bufferIndex < file->bufferSize())
        return true;

    // The frame has passed the end of the buffer. We need to load a new one (or write it)
    bufferIndex = 0;
    if (read){
        if (!file->loadBuffer()){
            if (file->errored() || currentWSV + 1 == files.size()){
                if (file->errored())
                    cout << "Error on loading buffer " << file->getError() << endl;
                open = false;
                frame = -1;
                return false;
            }
            currentWSV++;
        }

    } else {
        if (!file->storeBuffer(file->bufferSize())){
            open = false;
            return false;
        }
        file->buffer().clear();
        prepareFrame();
    }
    return true;
}

/**
 * See if the trajectory is open
 * @return If the the trajectory is open
 */
bool Trajectory::is_open(){
    return open;
}

/**
 * See if the trajectory is in read mode
 * @return If the trajectory is in read mode
 */
bool Trajectory::read_only(){
    return read;
}

/**
 * Get the current frame
 * @return The current frame
 */
int Trajectory::getFrame(){
    return frame;
}

/**
 * Add a trigger to this trajectory to begin another trajectory at the frame
 * number. 
 * @param  frame  The frame to cause the trigger at
 * @param  target The target to play at the frame
 * @return        True on success. 
 */
bool Trajectory::addTrigger(int frame, const string &target){
    if (triggers.count(frame) != 0)
        return false;
    triggers[frame] = target;
    return true;
}

/**
 * Get the triggered trajectory if there is a trigger set
 * @param  target A pointer to store the name of the next trigger trajectory
 * @return        True if there is a trigger or false if there isn't 
 */
bool Trajectory::getTrigger(string& target){
    if (triggers.count(frame) == 0)
        return false;
    target = triggers[frame];
    return true;
}

/**
 * Return true of the trajectory has a next line
 * @return True if there is a next line
 */
bool Trajectory::hasNext(){
    return files.size() > currentWSV && currentWSV != files.size() - 1;
}

/**
 * Check to see if a joint exists in this trajectory
 * @param  joint The joint to check for existance of
 * @return       True if the joint is in the trajectory
 */
bool Trajectory::contains(const string& joint){
    if (files.size() <= currentWSV)
        return false;
    WSVFile* file = files[currentWSV];
    if (!file || file->errored())
        return false;

    return file->header().count(joint) == 1 && !disabledJoints.count(joint) == 1;
}

/**
 * Disable a joint in the trajectory
 * @param joint The joint to disable
 */
void Trajectory::disableJoint(const string& joint){
    disabledJoints.insert(joint);
}

/**
 * Enable a joint in the trajectory
 * @param joint The joint to enable
 */
void Trajectory::enableJoint(const string& joint){
    set<string>::iterator it = disabledJoints.find(joint);

    if (it != disabledJoints.end())
        disabledJoints.erase(it);
}

/**
 * Reset the trajectory
 * @return True on success
 */
bool Trajectory::reset(){
    if (files.size() <= currentWSV){
        cout << "Error! Resetting trajectory past its list of trajectories. Aborting." << endl;
        open = false;
        return false;
    }

    WSVFile* file = files[currentWSV];
    if (!file || file->errored()){
        cout << "Last file has errored. Cannot reset." << endl;
        cout << file->getError() << endl;
        return false;
    }

    if (!read && bufferIndex < file->buffer().size() && bufferIndex > 0)
        file->storeBuffer(bufferIndex); // Writes what is remaining to the file

    for (int i = 0; i < files.size(); i++){
        if (files[i] && files[i]->errored()){
            cout << "Error: " << file->getError() << endl;
            open = false;
            return false;
        }
        files[i]->reset();
    }

    read = true;
    currentWSV = 0;
    bufferIndex = 0;
    frame = 0;

    open = files[currentWSV]->loadBuffer();
    return open;
}

/**
 * Set the header of the trajectory
 * @param header The header as a string
 */
void Trajectory::setHeader(const string& header){
    if (!files[currentWSV] || files[currentWSV]->readOnly())
        return;

    files[currentWSV]->setHeader(header);
    files[currentWSV]->storeHeader();
}

/**
 * Set the header of the trajectory
 * @param header The header as a struct
 */
void Trajectory::setHeader(const Header& header){
    if (!files[currentWSV] || files[currentWSV]->readOnly())
        return;

    files[currentWSV]->setHeader(header);
    files[currentWSV]->storeHeader();
}

/**
 * Get the joint header from the trajectory
 * @return The joint header as a structure
 */
const Trajectory::Header& Trajectory::getHeader(){
    if (files[currentWSV])
        return files[currentWSV]->orderedHeader();
    vector< string > temp;
    return temp;
}

/**
 * Prepare the frame to be written to the file
 */
void Trajectory::prepareFrame(){
    WSVFile* file = files[currentWSV];
    if (!open || read || !file || file->errored() )
        return;

    if (bufferIndex < file->bufferSize()){
        file->buffer().push_back(Frame());
        for (int i = 0; i < file->orderedHeader().size(); i++)
            file->buffer()[bufferIndex].push_back(0);
    }
}
