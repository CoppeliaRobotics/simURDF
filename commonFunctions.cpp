#pragma once

#include "commonFunctions.h"
#include <string>
#include <simLib.h>

void printToConsole(int verbosity,const char* txt)
{
    simAddLog("URDF",verbosity,txt);
}

void stringToArray(double array[],const std::string xyz)
{
    std::string buf;
    std::stringstream ss(xyz); 

    int i = 0;
    while (ss >> buf){ array[i++]=getFloat(buf);}
}

void stringToFArray(float array[],const std::string xyz)
{
    std::string buf;
    std::stringstream ss(xyz);

    int i = 0;
    while (ss >> buf){ array[i++]=(float)getFloat(buf);}
}

void stringToSizeArray(double array[],const std::string xyz)
{
    std::string buf;
    std::stringstream ss(xyz);

    for (size_t i=0;i<3;i++)
    {
        if (ss >> buf)
            array[i]=getFloat(buf);
        if (array[i]<0.001)
            array[i]=0.001;
    }
}

double getFloat(const std::string& text)
{
    try
    {
        return(boost::lexical_cast<double>(text));
    }
    catch (boost::bad_lexical_cast &)
    {
        return(0.0);
    }
}

int getInt(const std::string& text)
{
    try
    {
        return(boost::lexical_cast<int>(text));
    }
    catch (boost::bad_lexical_cast &)
    {
        return(0);
    }
}


bool isArrayEmpty(double array[])
{
    if(array[0] == 0.0 && array[1] ==0.0 && array[2] == 0.0){ return true;}
    else {return false;}
}
double* minus(double arr[])
{
    arr[0] = -arr[0];
    arr[1] = -arr[1];
    arr[2] = -arr[2];

    return arr;
}
std::string printMatrix(const double arr[])
{
    return (" value0 = "+boost::lexical_cast<std::string>(arr[0])+" value1 = "+boost::lexical_cast<std::string>(arr[1])+ " value2 = "+boost::lexical_cast<std::string>(arr[2])).c_str();
}

void setSimObjectName(int objectHandle,const char* desiredName)
{
#if SIM_PROGRAM_VERSION_NB>=40300
    simSetObjectAlias(objectHandle,desiredName,0);
#else
    std::string baseName(desiredName);
    for (int i=0;i<int(baseName.size());i++)
    { // Objects in CoppeliaSim can only contain a-z, A-Z, 0-9, '_' or exaclty one '#' optionally followed by a number
        char n=baseName[i];
        if ( ((n<'a')||(n>'z')) && ((n<'A')||(n>'Z')) && ((n<'0')||(n>'9')) )
            baseName[i]='_';
    }
    std::string objName(baseName);
    int suffix=2;
    while (simSetObjectName(objectHandle,objName.c_str())==-1)
        objName=baseName+boost::lexical_cast<std::string>(suffix++);
#endif
}

C4Vector getQuaternionFromRpy(double rpy[3])
{
    C4Vector q1,q2,q3;
    q1.setEulerAngles(C3Vector(rpy[0],0.0,0.0));
    q2.setEulerAngles(C3Vector(0.0,rpy[1],0.0));
    q3.setEulerAngles(C3Vector(0.0,0.0,rpy[2]));
    return(q3*q2*q1);
}

