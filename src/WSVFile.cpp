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
 * WSVFile.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: William Hilton
 *      Author: Solis Knight
 */

#include "WSVFile.h"

WSVFile::WSVFile(const string& fname, bool read, int bufferSize) {
    filename = fname;
    _read = read;
    _bufferSize = bufferSize;

    _lines = 0;
    _cols = 0;
    _error = false;
    _hasHeader = false;

    // Opens the file for either reading or writing.
    _file.open(filename.c_str(), read ? fstream::in : (fstream::out | fstream::trunc) );

    if (!_file.is_open()) {
        _errorMessageStream << "Trajectory Error: Unable to open _file '" << filename << "'";
        errorMessage = _errorMessageStream.str();
        _error = true;
        return;
    }

    if (read)
        loadStatistics();
}

WSVFile::~WSVFile(){
    if (!_error && _file.is_open())
        _file.close();
}


void WSVFile::setHeader(const string& line) {
    static stringstream sline;
    static string field;

    _orderedHeader.clear();
    _header.clear();
    sline.clear();

    sline << line;
    for (int i = 0; sline >> field; i++) {
        //cout << '(' << field << ')';
        _orderedHeader.push_back(field);
        _header[field] = i;
    }
    _hasHeader = true;
}

void WSVFile::setHeader(const vector<string>& header){
    _header.clear();

    _orderedHeader = header;
    for (int i = 0; i < header.size(); i++)
        _header[header[i]] = i;

    _hasHeader = true;
}

bool WSVFile::storeHeader(){
    if (_read || !_hasHeader)
        return false;

    for (int i = 0; i < _orderedHeader.size(); i++)
        _file << _orderedHeader[i] << WRITE_WHITESPACE;
    _file << endl;

    return true;
}

bool WSVFile::loadBuffer() {
    if (_file.eof() || _error || !_read){
        if (_file.eof()){
            errorMessage = "Attempting to load buffer at end of file.";
            return false;
        } else if (!_read){
            errorMessage = "Attempting to load buffer in a non-read trajectory.";
        }
        _error = true;
        return false;
    }

    static vector<string> fields;
    static Frame vals;
    /*static*/ stringstream fieldToVal;
    static string line;
    static TrajLineType type;

    int line_num = 0;
    int col_num = 0;
    double val;

    /*fieldToVal.clear();
    fieldToVal.seekp(0);
    fieldToVal.seekg(0);*/
    _buffer.clear();

    while ((line_num < _bufferSize) && getline(_file, line)) {
        // Clear old values
        fields.clear();
        vals.clear();

        // If the line was empty, skip it.
        type = line_type(line);
        if (type == BLANK || type == COMMENT)
            continue;

        // Split the line into vector of words
        split(line, fields);

        if (!is_numeric(fields[0]))
            continue;

        // Valid line found. Update the count.
        line_num++;
        if (DEBUG) cout << line << endl;
        if (DEBUG) cout << "buffer: ";
        // Parse lines.
        for (col_num = 0; col_num < fields.size(); col_num++) {
            fieldToVal << fields[col_num];
            // Streaming to a double had better work.
            if (fieldToVal >> val) {
                if (DEBUG) cout << '{' << val << '}';
                vals.push_back(val);
                fieldToVal.clear();
            } else {
                // If it doesn't, throw an error.
                _error = true;
                _errorMessageStream << "Trajectory Error: " << "Cannot convert to float. In '"
                    << filename << "' [line " << line_num << " column " << col_num << "] " << fields[col_num];
                errorMessage = _errorMessageStream.str();
                _file.close();
                return false;
            }
        }
        if (DEBUG) cout << endl;

        // Check that this line had the right number of columns.
        if (col_num != _cols) {
            _error = true;
            _errorMessageStream << "Trajectory Error: " << "Wrong number of columns. Expected "
                << _cols << " but saw " << vals.size() << ". In '" << filename << "' [line " << line_num << "]";
            errorMessage = _errorMessageStream.str();
            _file.close();
            return false;
        }

        // Add this row of values to our data table.
        _buffer.push_back(vals);
    }


    // If we read any valid data, return 1
    return line_num != 0;
}

bool WSVFile::storeBuffer(int frame){
    if (_read || frame >= _buffer.size())
        return false;

    for (int r = 0; r < frame; r++){
        for (int c = 0; c < _buffer[r].size(); c++)
            _file << _buffer[r][c] << WRITE_WHITESPACE;

        _file << endl;
    }
    return true;
}

void WSVFile::reset(){
    if (_error)
        return;
    if (!_read)
        _file.close();

    _file.clear();
    _file.seekg (_dataStart, _file.beg);
    //cout << "Resetting " << filename << " Data start: " << _dataStart << endl;

    if (!_read) {
        // Open file as input
        _file.open(filename.c_str(), fstream::in);

        if (!_file.is_open()) {
            _errorMessageStream << "Trajectory Error: Unable to open _file '" << filename << "'";
            errorMessage = _errorMessageStream.str();
            _error = true;
            return;
        }

        _read = true;
        loadStatistics();
    }
}

bool WSVFile::errored(){
    return _error;
}

int WSVFile::numLines(){
    return _lines;
}

int WSVFile::numCols(){
    return _cols;
}

bool WSVFile::headerSupplied(){
    return _hasHeader;
}

int WSVFile::bufferSize(){
    return _bufferSize;
}

bool WSVFile::readOnly(){
    return _read;
}

string WSVFile::getError(){
    return errorMessage;
}

WSVFile::HeaderMap& WSVFile::header(){
    return _header;
}

vector< string >& WSVFile::orderedHeader(){
    return _orderedHeader;
}

WSVFile::Buffer& WSVFile::buffer(){
    return _buffer;
}

WSVFile::Frame& WSVFile::start(){
    return _start;
}

WSVFile::Frame& WSVFile::end(){
    return _end;
}

// http://stackoverflow.com/a/5577987/2168416
bool WSVFile::is_numeric(const string& str) {
    /*static*/ stringstream conv;
    static double tmp;
    conv.clear();
    /*conv.seekg(0);
    conv.seekp(0);*/

    conv << str;
    conv >> tmp;
    return conv.eof();
}

void WSVFile::split(const string& line, vector<string>& fields) {
    /*static*/ stringstream sline;
    static string field;
    sline.clear();
    /*sline.seekp(0);
    sline.seekg(0);*/
    fields.clear();

    // Split the line into vector of words
    sline << line;

    while(sline >> field)
        fields.push_back(field);
}

WSVFile::TrajLineType WSVFile::line_type(const string& line) {
    // If the line was empty, skip it.
    if (line.length() == 0) {
        //cout << "Skipping empty line." << endl;
        return BLANK;
    } else if (line[0] == '#' || (line.length() > 1 && line[0] == '/' && line[1] == '/') ) {
        //cout << "Skipping commented line. " << line << endl;
        return COMMENT;
    } else {
        return DATA; // Could be a header too.
    }
}

void WSVFile::loadStatistics(){
    if (!_read)
        return;

    if (!_file.is_open()){
        _errorMessageStream << "Trajectory Error: Loading statistics for non-open file '" << filename << "'";
        errorMessage = _errorMessageStream.str();
        _error = true;
        return;
    }

    static string line;
    static string lastLine;
    /*static*/ stringstream sline;
    static vector<string> fields;
    static Frame vals;

    int line_num = 0;
    bool found_header = false;
    bool found_start = false;
    bool isNumeric = false;

    while (getline(_file, line)) {
        // Clear old values
        fields.clear();
        vals.clear();

        // Load line
        line_num++;

        // If the line was empty, skip it.
        TrajLineType t = line_type(line);
        if (t == BLANK || t == COMMENT)
            continue;

        lastLine = line;

        if (found_header && found_start)
            continue;

        // Split the line into vector of words
        split(line, fields);

        isNumeric = is_numeric(fields[0]);

        // Act on the first row that is not blank or a comment.
        if (!found_header) {
            found_header = true;

            // Set # of columns for this file.
            _cols = fields.size();

            // Figure out if the row is a header or not, and treat specially. If not header, store start.
            if (isNumeric) {
                _hasHeader = false;
                found_start = true;
                _start.resize(_cols);

                for (int i = 0; i < _cols; i++){
                    sline.clear();
                    /*sline.seekg(0);
                    sline.seekp(0);*/
                    sline << fields[i];
                    sline >> _start[i];
                }

                _dataStart = 0;

            } else {
                _dataStart = _file.tellg();
                setHeader(fields);

                continue;
            }
        }

        // Act on the first data member
        if (!found_start && isNumeric){
            found_start = true;
            if (!fields.size() == _cols){
                _errorMessageStream << "Trajectory Error: Inconsistent column size " << fields.size() << " for file '" << filename << "'";
                errorMessage = _errorMessageStream.str();
                _error = true;
                return;
            }
            _start.resize(_cols);
            for (int i = 0; i < _cols; i++){
                sline.clear();
                /*sline.seekg(0);
                sline.seekp(0);*/
                sline << fields[i];
                sline >> _start[i];
            }
        }

        // Count the number of data lines in the file.
        _lines++;
    }

    split(lastLine, fields);
    if (is_numeric(fields[0])) {
        if (fields.size() != _cols){
            _errorMessageStream << "Trajectory Error: Inconsistent column size " << fields.size() << " for file '" << filename << "'";
            errorMessage = _errorMessageStream.str();
            _error = true;
            return;
        }
        _end.resize(_cols);
        for (int i = 0; i < _cols; i++){
            sline.clear();
            /*sline.seekp(0);
            sline.seekg(0);*/
            sline << fields[i];
            sline >> _end[i];
        }
    }

    // Rewind file to beginning of data.
    reset();
}
