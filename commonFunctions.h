#pragma once

#include <string>
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <simMath/4Vector.h>

#ifndef piValue
    #define piValue 3.14159265359
#endif

void stringToArray(double array[],const std::string xyz);
void stringToFArray(float array[],const std::string xyz);
void stringToSizeArray(double array[],const std::string xyz);
double getFloat(const std::string& text);
int getInt(const std::string& text);
bool isArrayEmpty(double arr[]);
double* minus(double arr[]);
std::string printMatrix(const double arr[]);
void printToConsole(int verbosity,const char* txt);
void setSimObjectName(int objectHandle,const char* desiredName);
C4Vector getQuaternionFromRpy(double rpy[3]);
