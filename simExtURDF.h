#pragma once

#include <simLib/simLib.h>
#include <string>

#ifdef _WIN32
    #define SIM_DLLEXPORT extern "C" __declspec(dllexport)
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #define SIM_DLLEXPORT extern "C"
#endif /* __linux || __APPLE__ */

// The 3 required entry points of the CoppeliaSim plugin:
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt);
SIM_DLLEXPORT void simEnd();
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData);
