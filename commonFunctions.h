#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include "4Vector.h"

#ifndef piValue
    #define piValue 3.14159265359f
#endif

void stringToArray(float array[],const std::string xyz);
float getFloat(const std::string& text);
int getInt(const std::string& text);
bool isArrayEmpty(float arr[]);
float* minus(float arr[]);
std::string printMatrix(const float arr[]);
void printToConsole(int verbosity,const char* txt);
void setSimObjectName(int objectHandle,const char* desiredName);
C4Vector getQuaternionFromRpy(float rpy[3]);
