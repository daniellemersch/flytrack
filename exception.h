/*
 *  The exeception class allow to manage errors in a standardized way with error codes.
 *  It also permits to access error mesage from a graphic interface through a get_error method.
 *
 *  Created by Danielle Mersch on 15/06/2015.
 *  Copyright 2015 __LMB__. All rights reserved.
 *
 */

#ifndef __exception__
#define __exception__


#include <cstring>
#include <iostream>
#include <string>


using namespace std;


const int USE = 1;
const int PARAMETER_ERROR = 2;
const int UNKNOWN_OPTION = 3;
const int OPTION_MISSING = 4;

const int OUTPUT_EXISTS = 5;
const int NO_INPUT = 6;
const int CANNOT_OPEN_FILE = 7;
const int CANNOT_READ_FILE = 8;
const int CANNOT_WRITE_FILE = 9;

const int ARGUMENT_MISSING = 10;

const int DATA_ERROR = 11;
const int NOT_ENOUGH_BYTES = 12;
const int BUFFER = 13;
const int INTERNAL_ERROR = 14;

const int CAMERA_ERROR = 15;

class Exception{

public:

Exception(int error, string info ="");

void display_error(int e, string info);
string get_error();

private:
string error_message;

};

#endif //__exception__

