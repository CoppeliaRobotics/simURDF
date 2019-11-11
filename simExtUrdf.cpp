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

#define PLUGIN_VERSION 10   // 1 until 20/1/2013 (1 was a very early beta)
                            // 2 until 10/1/2014 (CoppeliaSim3.0.5)
                            // 3: new lock
                            // 4: since CoppeliaSim 3.1.2
                            // 5: since after CoppeliaSim 3.1.3
                            // 6: since CoppeliaSim 3.2.2
                            // 7: since CoppeliaSim 3.2.2 rev2
                            // 8: since CoppeliaSim 3.3.0 (headless mode detect)
                            // 9: since CoppeliaSim 3.3.1 (Using stacks to exchange data with scripts)
                            // 10: since CoppeliaSim 3.4.1 (new API notation)

LIBRARY simLib;
CUrdfDialog* urdfDialog=NULL;

// This is the plugin start routine (called just once, just after the plugin was loaded):
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt)
{
    // Dynamically load and bind CoppeliaSim functions:
     // ******************************************
     // 1. Figure out this plugin's directory:
     char curDirAndFile[1024];
 #ifdef _WIN32
     _getcwd(curDirAndFile, sizeof(curDirAndFile));
 #elif defined (__linux) || defined (__APPLE__)
     getcwd(curDirAndFile, sizeof(curDirAndFile));
 #endif
     std::string currentDirAndPath(curDirAndFile);
     // 2. Append the CoppeliaSim library's name:
     std::string temp(currentDirAndPath);
 #ifdef _WIN32
     temp+="/coppeliaSim.dll";
 #elif defined (__linux)
     temp+="/libcoppeliaSim.so";
 #elif defined (__APPLE__)
     temp+="/libcoppeliaSim.dylib";
 #endif /* __linux || __APPLE__ */
    // 3. Load the CoppeliaSim library:
    simLib=loadSimLibrary(temp.c_str());
    if (simLib==NULL)
    {
        std::cout << "Error, could not find or correctly load the CoppeliaSim library. Cannot start 'Urdf' plugin.\n";
        return(0); // Means error, CoppeliaSim will unload this plugin
    }
    if (getSimProcAddresses(simLib)==0)
    {
        std::cout << "Error, could not find all required functions in the CoppeliaSim library. Cannot start 'Urdf' plugin.\n";
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppeliaSim will unload this plugin
    }
    // ******************************************

    // Check the version of CoppeliaSim:
    // ******************************************
    int simVer,simRev;
    simGetIntegerParameter(sim_intparam_program_version,&simVer);
    simGetIntegerParameter(sim_intparam_program_revision,&simRev);
    if( (simVer<30400) || ((simVer==30400)&&(simRev<9)) )
    {
        std::cout << "Sorry, your CoppeliaSim copy is somewhat old, CoppeliaSim 3.4.0 rev9 or higher is required. Cannot start 'Urdf' plugin.\n";
        unloadSimLibrary(simLib);
        return(0);
    }
    // ******************************************

    // Check if CoppeliaSim runs in headless mode:
    // ******************************************
    if (simGetBooleanParameter(sim_boolparam_headless)>0)
    {
        std::cout << "CoppeliaSim runs in headless mode. Cannot start 'Urdf' plugin.\n";
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppeliaSim will unload this plugin
    }
    // ******************************************

    QWidget* pMainWindow = (QWidget*)simGetMainWindow(1);
    urdfDialog=new CUrdfDialog(pMainWindow); // The plugin dialog
    simAddModuleMenuEntry("",1,&urdfDialog->dialogMenuItemHandle);
    simSetModuleMenuItemState(urdfDialog->dialogMenuItemHandle,1,"URDF import...");

    simRegisterScriptVariable("simURDF","require('simExtUrdf')",0);

    simRegisterScriptCallbackFunction("simURDF.import@Urdf","string robot_name=simURDF.import(string urdf,bool hideCollisionLinks,bool hideJoints,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool showConvexDecompositionDlg,bool centerAboveGround,bool makeModel,bool noSelfCollision,bool positionCtrl)",simImportUrdfCallback);
    simRegisterScriptCallbackFunction("simURDF.importFile@Urdf","string robot_name=simURDF.importFile(string fileAndPath,bool hideCollisionLinks,bool hideJoints,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool showConvexDecompositionDlg,bool centerAboveGround,bool makeModel,bool noSelfCollision,bool positionCtrl)",simImportUrdfCallback);

    // Following for backward compatibility:
    simRegisterScriptVariable("simExtImportUrdf","simURDF.import",-1);
    simRegisterScriptVariable("simExtImportUrdfFile","simURDF.importFile",-1);
    simRegisterScriptCallbackFunction("simExtImportUrdf@Urdf","Please use the simURDF.import notation instead",0);
    simRegisterScriptCallbackFunction("simExtImportUrdfFile@Urdf","Please use the simURDF.importFile notation instead",0);

    return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when CoppeliaSim is ending, i.e. releasing this plugin):
SIM_DLLEXPORT void simEnd()
{
    // Here you could handle various clean-up tasks
    delete urdfDialog;
    unloadSimLibrary(simLib); // release the library
}

// This is the plugin messaging routine (i.e. CoppeliaSim calls this function very often, with various messages):
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 6 lines at the beginning and unchanged:
    static bool refreshDlgFlag=true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
    void* retVal=NULL;

    // Here we can intercept many messages from CoppeliaSim (actually callbacks).
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the CoppeliaSim user manual.

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

        if (instanceSwitched)
        {
            // React to an instance switch here!!
        }

        if (sceneContentChanged)
        {
            refreshDlgFlag=true;
        }
    }
    if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
    { // handle refresh of the plugin's dialog:
        urdfDialog->refresh(); // Refresh the dialog
        refreshDlgFlag=false;
    }

    // Keep following unchanged:
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
    if (D.readDataFromStack(p->stackID,inArgs_IMPORT,inArgs_IMPORT[0],"simURDF.import"))
    {
        std::vector<CScriptFunctionDataItem>* inData=D.getInDataPtr();
        simChar* name = simImportUrdf(inData->at(0).stringData[0].c_str(),inData->at(1).boolData[0],inData->at(2).boolData[0],inData->at(3).boolData[0],inData->at(4).boolData[0],inData->at(5).boolData[0],inData->at(6).boolData[0],inData->at(7).boolData[0],inData->at(8).boolData[0],inData->at(9).boolData[0]);

        D.pushOutData(CScriptFunctionDataItem(name));
    }
    D.writeDataToStack(p->stackID);
}
