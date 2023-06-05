#pragma once

#include <simLib/simLib.h>
#include <simLib/simExp.h>
#include <string>

// The 3 required entry points of the CoppeliaSim plugin:
SIM_DLLEXPORT int simInit(const char* pluginName);
SIM_DLLEXPORT void simCleanup();
SIM_DLLEXPORT void simMsg(int message,int* auxData,void* pointerData);
