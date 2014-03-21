/*
 * Trajectory.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: maestro
 */

#include "Trajectory.h"

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

Trajectory::~Trajectory(){
    for (int i = 0; i < files.size(); i++){
        if (files[i]){
            delete files[i];
            files[i] = NULL;
        }
    }
}

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
    //TODO: Find a better way.
}

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

bool Trajectory::is_open(){
    return open;
}

bool Trajectory::read_only(){
    return read;
}

int Trajectory::getFrame(){
    return frame;
}

bool Trajectory::addTrigger(int frame, const string &target){
    if (triggers.count(frame) != 0)
        return false;
    triggers[frame] = target;
    return true;
}

bool Trajectory::getTrigger(string& target){
    if (triggers.count(frame) == 0)
        return false;
    target = triggers[frame];
    return true;
}

bool Trajectory::hasNext(){
    return files.size() > currentWSV && currentWSV != files.size() - 1;
}

bool Trajectory::contains(const string& joint){
    if (files.size() <= currentWSV)
        return false;
    WSVFile* file = files[currentWSV];
    if (!file || file->errored())
        return false;

    return file->header().count(joint) == 1 && !disabledJoints.count(joint) == 1;
}

void Trajectory::disableJoint(const string& joint){
    disabledJoints.insert(joint);
}

void Trajectory::enableJoint(const string& joint){
    set<string>::iterator it = disabledJoints.find(joint);

    if (it != disabledJoints.end())
        disabledJoints.erase(it);
}

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


void Trajectory::setHeader(const string& header){
    if (!files[currentWSV] || files[currentWSV]->readOnly())
        return;

    files[currentWSV]->setHeader(header);
    files[currentWSV]->storeHeader();
}

void Trajectory::setHeader(const Header& header){
    if (!files[currentWSV] || files[currentWSV]->readOnly())
        return;

    files[currentWSV]->setHeader(header);
    files[currentWSV]->storeHeader();
}

const Trajectory::Header& Trajectory::getHeader(){
    if (files[currentWSV])
        return files[currentWSV]->orderedHeader();
    vector< string > temp;
    return temp;
}

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
