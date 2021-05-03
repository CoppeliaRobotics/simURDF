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

#define PLUGIN_VERSION 13   // 1 until 20/1/2013 (1 was a very early beta)
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

static LIBRARY simLib;

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

    simRegisterScriptCallbackFunction("simURDF.import@URDF","string robot_name=simURDF.import(string urdf,int options=0)",simImportUrdfCallback);
    simRegisterScriptCallbackFunction("simURDF.importFile@URDF","string robot_name=simURDF.importFile(string fileAndPath,bool hideCollisionLinks=true,bool hideJoints=true,bool convexDecomposeNonConvexCollidables=true,bool createVisualIfNone=true,bool showConvexDecompositionDlg=false,bool centerAboveGround=true,bool makeModel=true,bool noSelfCollision=false,bool positionCtrl=true)",simImportUrdfCallback);

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

SIM_DLLEXPORT simChar* simImportUrdf(const simChar* filenameOrUrdf, simBool hideCollisionLinks, simBool hideJoints, simBool convexDecomposeNonConvexCollidables, simBool createVisualIfNone, simBool showConvexDecompositionDlg, simBool centerAboveGround, simBool makeModel, simBool noSelfCollision, simBool positionCtrl) {

    robot Robot(filenameOrUrdf, hideCollisionLinks, hideJoints, convexDecomposeNonConvexCollidables, createVisualIfNone, showConvexDecompositionDlg, centerAboveGround, makeModel, noSelfCollision, positionCtrl);

    simChar* result = (simChar*)simCreateBuffer(Robot.name.length()+1);
    memcpy((void*) result, (void*) Robot.name.c_str(), Robot.name.length()+1);

    return result;
}

const int inArgs_IMPORT[]={
    10,
    sim_script_arg_string,0, // filenameAndPath or URDF contents
    sim_script_arg_int32,0, // assign collision links to layer 9
    sim_script_arg_bool,0, // assign joints to layer 10
    sim_script_arg_bool,0, // convex decompose
    sim_script_arg_bool,0, // create visual links if none
    sim_script_arg_bool,0, // show convex decomposition dialog
    sim_script_arg_bool,0, // center model above ground
    sim_script_arg_bool,0, // prepare model definition if feasible
    sim_script_arg_bool,0, // alternate local respondable masks
    sim_script_arg_bool,0  // enable position control for revolute and prismatic joints
};

void simImportUrdfCallback(SScriptCallBack* p)
{
    CScriptFunctionData D;
    if (D.readDataFromStack(p->stackID,inArgs_IMPORT,inArgs_IMPORT[0]-9,"simURDF.import"))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        int options=0;
        bool arg2=true;
        bool arg3=true;
        bool arg4=true;
        bool arg5=true;
        bool arg6=false;
        bool arg7=true;
        bool arg8=true;
        bool arg9=false;
        bool arg10=true;
        if ( (inData->size()>=2)&&(inData->at(1).int32Data.size()==1) )
        {
            options=inData->at(1).int32Data[0];
            arg2=options;
        }
        if ( (inData->size()>=3)&&(inData->at(2).boolData.size()==1) )
            arg3=inData->at(2).boolData[0];
        if ( (inData->size()>=4)&&(inData->at(3).boolData.size()==1) )
            arg4=inData->at(3).boolData[0];
        if ( (inData->size()>=5)&&(inData->at(4).boolData.size()==1) )
            arg5=inData->at(4).boolData[0];
        if ( (inData->size()>=6)&&(inData->at(5).boolData.size()==1) )
            arg6=inData->at(5).boolData[0];
        if ( (inData->size()>=7)&&(inData->at(6).boolData.size()==1) )
            arg7=inData->at(6).boolData[0];
        if ( (inData->size()>=8)&&(inData->at(7).boolData.size()==1) )
            arg8=inData->at(7).boolData[0];
        if ( (inData->size()>=9)&&(inData->at(8).boolData.size()==1) )
            arg9=inData->at(8).boolData[0];
        if ( (inData->size()>=10)&&(inData->at(9).boolData.size()==1) )
            arg10=inData->at(9).boolData[0];
        if (options>1)
        {
            arg3=((options&2)==0);
            arg4=((options&4)==0);
            arg5=((options&8)==0);
            arg6=((options&16)!=0);
            arg7=((options&32)==0);
            arg8=((options&64)==0);
            arg9=((options&128)!=0);
            arg10=((options&256)==0);
        }
        simChar* name = simImportUrdf(inData->at(0).stringData[0].c_str(),arg2,arg3,arg4,arg5,arg6,arg7,arg8,arg9,arg10);

        D.pushOutData(CScriptFunctionDataItem(name));
    }
    D.writeDataToStack(p->stackID);
}
