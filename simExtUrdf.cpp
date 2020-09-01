#include "simExtUrdf.h"
#include "simLib.h"
#include <iostream>
#include <vector>
#include <bitset>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include "urdfdialog.h"
#include "robot.h"
#include "scriptFunctionData.h"

#ifdef _WIN32
    #include <direct.h>
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
    #include <string.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define PLUGIN_VERSION 11   // 1 until 20/1/2013 (1 was a very early beta)
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

static LIBRARY simLib;
static CUrdfDialog* urdfDialog=nullptr;

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

    if (simGetBooleanParameter(sim_boolparam_headless)==0)
    { // if CoppeliaSim doesn't run in headless mode
        QWidget* pMainWindow = (QWidget*)simGetMainWindow(1);
        urdfDialog=new CUrdfDialog(pMainWindow); // The plugin dialog
        simAddModuleMenuEntry("",1,&urdfDialog->dialogMenuItemHandle);
        simSetModuleMenuItemState(urdfDialog->dialogMenuItemHandle,1,"URDF import...");
    }

    simRegisterScriptVariable("simURDF","require('simExtUrdf')",0);

    simRegisterScriptCallbackFunction("simURDF.import@Urdf","string robot_name=simURDF.import(string urdf,bool hideCollisionLinks=true,bool hideJoints=true,bool convexDecomposeNonConvexCollidables=true,bool createVisualIfNone=true,bool showConvexDecompositionDlg=false,bool centerAboveGround=true,bool makeModel=true,bool noSelfCollision=false,bool positionCtrl=true)",simImportUrdfCallback);
    simRegisterScriptCallbackFunction("simURDF.importFile@Urdf","string robot_name=simURDF.importFile(string fileAndPath,bool hideCollisionLinks=true,bool hideJoints=true,bool convexDecomposeNonConvexCollidables=true,bool createVisualIfNone=true,bool showConvexDecompositionDlg=false,bool centerAboveGround=true,bool makeModel=true,bool noSelfCollision=false,bool positionCtrl=true)",simImportUrdfCallback);

    // Following for backward compatibility:
    simRegisterScriptVariable("simExtImportUrdf","simURDF.import",-1);
    simRegisterScriptVariable("simExtImportUrdfFile","simURDF.importFile",-1);
    simRegisterScriptCallbackFunction("simExtImportUrdf@Urdf","Please use the simURDF.import notation instead",0);
    simRegisterScriptCallbackFunction("simExtImportUrdfFile@Urdf","Please use the simURDF.importFile notation instead",0);

    return(PLUGIN_VERSION);
}

SIM_DLLEXPORT void simEnd()
{
    delete urdfDialog;
    unloadSimLibrary(simLib); // release the library
}

SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
    static bool refreshDlgFlag=true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    void* retVal=nullptr;

    if (urdfDialog!=nullptr)
    {
        if (message==sim_message_eventcallback_refreshdialogs)
            refreshDlgFlag=true; // CoppeliaSim dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

        if (message==sim_message_eventcallback_menuitemselected)
        { // A custom menu bar entry was selected..
            if (auxiliaryData[0]==urdfDialog->dialogMenuItemHandle)
                urdfDialog->makeVisible(!urdfDialog->getVisible()); // Toggle visibility of the dialog
        }

        if (message==sim_message_eventcallback_instancepass)
        { // It is important to always correctly react to events in CoppeliaSim. This message is the most convenient way to do so:
            urdfDialog->handleCommands();
            urdfDialog->setSimulationStopped(simGetSimulationState()==sim_simulation_stopped);

            int flags=auxiliaryData[0];
            bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
            bool instanceSwitched=((flags&64)!=0);

            if (sceneContentChanged)
                refreshDlgFlag=true;
        }
        if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
        { // handle refresh of the plugin's dialog:
            urdfDialog->refresh(); // Refresh the dialog
            refreshDlgFlag=false;
        }
    }

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
    sim_script_arg_bool,0, // assign collision links to layer 9
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
        bool arg2=true;
        bool arg3=true;
        bool arg4=true;
        bool arg5=true;
        bool arg6=false;
        bool arg7=true;
        bool arg8=true;
        bool arg9=false;
        bool arg10=true;
        if ( (inData->size()>=2)&&(inData->at(1).boolData.size()==1) )
            arg2=inData->at(1).boolData[0];
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
        simChar* name = simImportUrdf(inData->at(0).stringData[0].c_str(),arg2,arg3,arg4,arg5,arg6,arg7,arg8,arg9,arg10);

        D.pushOutData(CScriptFunctionDataItem(name));
    }
    D.writeDataToStack(p->stackID);
}
