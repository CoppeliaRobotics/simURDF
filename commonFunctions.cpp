#pragma once

#include "commonFunctions.h"
#include <string>
#include "simLib.h"

void printToConsole(int verbosity,const char* txt)
{
    static int handle=-1;
    simAddLog("URDF",verbosity,txt);
    std::string text(txt);
    text+="\n";
    if (simAuxiliaryConsolePrint(handle,text.c_str())<=0)
    {
        handle=simAuxiliaryConsoleOpen("URDF import",5000,2+4,nullptr,nullptr,nullptr,nullptr);
        simAuxiliaryConsolePrint(handle,text.c_str());
    }
}

void stringToArray(float array[],const std::string xyz)
{
    std::string buf;
    std::stringstream ss(xyz); 

    int i = 0;
    while (ss >> buf){ array[i++]=getFloat(buf);}
}

void stringToSizeArray(float array[],const std::string xyz)
{
    std::string buf;
    std::stringstream ss(xyz);

    for (size_t i=0;i<3;i++)
    {
        if (ss >> buf)
            array[i]=getFloat(buf);
        if (array[i]<0.001f)
            array[i]=0.001f;
    }
}

float getFloat(const std::string& text)
{
    try
    {
        return(boost::lexical_cast<float>(text));
    }
    catch (boost::bad_lexical_cast &)
    {
        return(0.0f);
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


bool isArrayEmpty(float array[])
{
    if(array[0] == 0.0f && array[1] ==0.0f && array[2] == 0.0f){ return true;}
    else {return false;}
}
float* minus(float arr[])
{
    arr[0] = -arr[0];
    arr[1] = -arr[1];
    arr[2] = -arr[2];

    return arr;
}
std::string printMatrix(const float arr[])
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

C4Vector getQuaternionFromRpy(float rpy[3])
{
    C4Vector q1,q2,q3;
    q1.setEulerAngles(C3Vector(rpy[0],0.0f,0.0f));
    q2.setEulerAngles(C3Vector(0.0f,rpy[1],0.0f));
    q3.setEulerAngles(C3Vector(0.0f,0.0f,rpy[2]));
    return(q3*q2*q1);
}

