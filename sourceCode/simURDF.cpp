#include "simURDF.h"
#include <string>
#include <simLib/simLib.h>
#include <iostream>
#include <vector>
#include <bitset>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include "robot.h"
#include <simLib/scriptFunctionData.h>

#define PLUGIN_VERSION 16   // 1 until 20/1/2013 (1 was a very early beta)
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
                            // 15: doubles + new shapes
                            // 16: since CoppeliaSim 4.6 (new plugins)

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
    if (D.readDataFromStack(p->stackID,inArgs_IMPORT,inArgs_IMPORT[0]-2,nullptr))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int options=0;
        if ( (inData->size()>=2)&&(inData->at(1).int32Data.size()==1) )
            options=inData->at(1).int32Data[0];
        std::string packageStr;
        if ( (inData->size()>=3)&&(inData->at(2).stringData.size()==1) )
            packageStr=inData->at(2).stringData[0];
        robot Robot(inData->at(0).stringData[0].c_str(),options,packageStr.c_str());
        D.pushOutData(CScriptFunctionDataItem(Robot.name));
        D.pushOutData(CScriptFunctionDataItem(Robot.parentlessObjects));
        D.writeDataToStack(p->stackID);
    }
}

SIM_DLLEXPORT int simInit(SSimInit* info)
{
    simLib=loadSimLibrary(info->coppeliaSimLibPath);
    if (simLib==nullptr)
    {
        simAddLog(info->pluginName,sim_verbosity_errors,"could not find or correctly load the CoppeliaSim library. Cannot start the plugin.");
        return(0); // Means error, CoppeliaSim will unload this plugin
    }
    if (getSimProcAddresses(simLib)==0)
    {
        simAddLog(info->pluginName,sim_verbosity_errors,"could not find all required functions in the CoppeliaSim library. Cannot start the plugin.");
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppeliaSim will unload this plugin
    }

    simRegisterScriptCallbackFunction("import",nullptr,simImportUrdfCallback);

    return(PLUGIN_VERSION);
}

SIM_DLLEXPORT void simCleanup()
{
    unloadSimLibrary(simLib); // release the library
}

SIM_DLLEXPORT void simMsg(SSimMsg*)
{
}
