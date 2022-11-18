#include "simExtURDF.h"
#include "simLib.h"
#include <iostream>
#include <vector>
#include <bitset>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include "robot.h"
#include "scriptFunctionData.h"

#ifdef _WIN32
    #include <direct.h>
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
    #include <string.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define PLUGIN_VERSION 14   // 1 until 20/1/2013 (1 was a very early beta)
                            // 2 until 10/1/2014 (CoppeliaSim3.0.5)
                            // 3: new lock
                            // 4: since CoppeliaSim 3.1.2
                            // 5: since after CoppeliaSim 3.1.3
                            // 6: since CoppeliaSim 3.2.2
                            // 7: since CoppeliaSim 3.2.2 rev2
                            // 8: since CoppeliaSim 3.3.0 (headless mode detect)
                            // 9: since CoppeliaSim 3.3.1 (Using stacks to exchange data with scripts)
                            // 10: since CoppeliaSim 3.4.1 (new API notation)
                            // 11: using simSetShapeInertia and simSetShapeMass instead of simSetShapeMassAndInertia (since CoppeliaSim 4.1.0 rev2)
                            // 12: Urdf --> URDF
                            // 13: removed C++ UI, now provided via add-on
                            // 14: various rework and bug fix

static LIBRARY simLib;

const int inArgs_IMPORT[]={
    3,
    sim_script_arg_string,0, // filenameAndPath or URDF contents
    sim_script_arg_int32,0, // options
    sim_script_arg_string,0, // op. replacement string for 'package://'
};

void simImportUrdfCallback(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_IMPORT,inArgs_IMPORT[0]-2,"simURDF.import"))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int options=0;
        if ( (inData->size()>=2)&&(inData->at(1).int32Data.size()==1) )
            options=inData->at(1).int32Data[0];
        std::string packageStr;
        if ( (inData->size()>=3)&&(inData->at(2).stringData.size()==1) )
            packageStr=inData->at(2).stringData[0];
        robot Robot(inData->at(0).stringData[0].c_str(), (options&1)==0, (options&2)==0,options&4,options&8,options&512,(options&32)==0,(options&64)==0,(options&128)==0,(options&256)==0,packageStr.c_str());
        char* name = (char*)simCreateBuffer(Robot.name.length()+1);
        memcpy((void*) name, (void*) Robot.name.c_str(), Robot.name.length()+1);
        D.pushOutData(CScriptFunctionDataItem(name));
        simReleaseBuffer(name);
        D.writeDataToStack(p->stackID);
    }
}

SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
     char curDirAndFile[1024];
 #ifdef _WIN32
     _getcwd(curDirAndFile, sizeof(curDirAndFile));
 #elif defined (__linux) || defined (__APPLE__)
     getcwd(curDirAndFile, sizeof(curDirAndFile));
 #endif
     std::string currentDirAndPath(curDirAndFile);

     std::string temp(currentDirAndPath);
 #ifdef _WIN32
     temp+="/coppeliaSim.dll";
 #elif defined (__linux)
     temp+="/libcoppeliaSim.so";
 #elif defined (__APPLE__)
     temp+="/libcoppeliaSim.dylib";
 #endif /* __linux || __APPLE__ */

     simLib=loadSimLibrary(temp.c_str());
    if (simLib==nullptr)
    {
        printf("simExtURDF: error: could not find or correctly load the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
        return(0); // Means error, CoppeliaSim will unload this plugin
    }
    if (getSimProcAddresses(simLib)==0)
    {
        printf("simExtURDF: error: could not find all required functions in the CoppeliaSim library. Cannot start the plugin.\n"); // cannot use simAddLog here.
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppeliaSim will unload this plugin
    }

    simRegisterScriptVariable("simURDF","require('simURDF')",0);

    simRegisterScriptCallbackFunction("simURDF.import@URDF","string robot_name=simURDF.import(string urdf,int options=0,string packageStrReplace=nil)",simImportUrdfCallback);
    simRegisterScriptCallbackFunction("simURDF.importFile@URDF","string robot_name=simURDF.importFile(string fileAndPath,int options=0)",simImportUrdfCallback);

    // Following for backward compatibility:
    simRegisterScriptVariable("simExtImportUrdf","simURDF.import",-1);
    simRegisterScriptVariable("simExtImportUrdfFile","simURDF.importFile",-1);
    simRegisterScriptCallbackFunction("simExtImportUrdf@URDF","Please use the simURDF.import notation instead",0);
    simRegisterScriptCallbackFunction("simExtImportUrdfFile@URDF","Please use the simURDF.importFile notation instead",0);

    return(PLUGIN_VERSION);
}

SIM_DLLEXPORT void simEnd()
{
    unloadSimLibrary(simLib); // release the library
}

SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    void* retVal=nullptr;

    simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
    return(retVal);
}
