/*
 * WSVFile.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: William Hilton
 *      Author: Solis Knight
 */

#ifndef WSVFILE_H_
#define WSVFILE_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#define DEBUG 0

using std::map;
using std::vector;
using std::string;
using std::cout;
using std::endl;
using std::fstream;
using std::stringstream;
using std::ostringstream;

#define WRITE_WHITESPACE '\t'

// Whitespace Separated Value file reader class.
// Whitespace is defined by the C++ isspace() function: ' \t\n\r\f\v'
class WSVFile {
private:
    enum TrajLineType { DATA, COMMENT, BLANK };

public:
    typedef vector< double > Frame;
    typedef vector< Frame > Buffer;
    typedef vector< string > Header;
    typedef map < string, int > HeaderMap;

    /* Constructor. Notably, it will initialize:
        .hasHeader
        .header (if it exists)
        .lines
        .cols
        BUT NOT
        .data
    After calling, PLEASE check
        .error
        .errorMessage (if .error)
    */
    WSVFile(const string& filename, bool read, int bufferSize = 10);

    ~WSVFile();

    // Use to override the header set by the constructor.
    void setHeader(const string& line);

    void setHeader(const vector<string>& header);

    /**
     * Writes the current header to a file.
     * Only works if marked as a writable file.
     */
    bool storeHeader();

    /**
     * Fills the internal buffer with data vectors from the file.
     * Will return true if valid data was read. (i.e. if the read hits a non-full column, or invalid data, this method will return false.)
     * Note: This method will return true even if the buffer was not fully filled.
     */
    bool loadBuffer();

    /**
     * Writes the current buffer (up to frame 'frame') to the file opened for write.
     */
    bool storeBuffer(int frame);

    /**
     * Resets this file to its initial state.
     * In the case of writable files, the file is closed, cleared, and reopened for reading. Statistics are compiled.
     * Readable files are simply reset to the beginning.
     */
    void reset();


    bool errored();

    int numLines();

    int numCols();

    bool headerSupplied();

    int bufferSize();

    bool readOnly();

    string getError();



    HeaderMap& header();

    vector< string >& orderedHeader();



    Buffer& buffer();

    Frame& start();

    Frame& end();


private:

    bool is_numeric(const string& str);

    /**
     * Determines, from the content of the current line, what type it is.
     * Classifies lines as either blank, a comment, or data
     */
    TrajLineType line_type(const string& line);

    // Split a string into a vector of strings by using whitespace as a delimiter.
    void split(const string& line, vector< string >& fields);

    /**
     * Iterates through each line of a file opened for read searching for statistics such as:
     *  - number of lines
     *  - existence of a header
     *  - number of columns
     *  - start vector
     *  - end vector
     */
    void loadStatistics();


    /**  Configuration  */
    ostringstream _errorMessageStream;
    fstream _file;
    string filename;
    string errorMessage;

    bool _hasHeader;
    int _dataStart;
    int _lines;
    int _cols;
    int _bufferSize;
    bool _error;
    bool _read;

    /**  Readable Data  */
    HeaderMap _header;
    vector< string > _orderedHeader;
    Buffer _buffer;                 // Note the assumption that all values in the WSV file will be floating point numbers.
    Frame _start;                   // Note the assumption that all values in the WSV file will be floating point numbers.
    Frame _end;                     // Note the assumption that all values in the WSV file will be floating point numbers.

};

#endif
