#pragma once

#include <simLib/simLib.h>
#include <simLib/simExp.h>
#include <string>

// The 3 required entry points of the CoppeliaSim plugin:
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt);
SIM_DLLEXPORT void simEnd();
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData);
