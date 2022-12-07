#include "sensor.h"
#include "commonFunctions.h"
#include "4X4Matrix.h"

sensor::sensor()
{
    //Initialize arrays
     origin_xyz[0] = 0;   origin_xyz[1] = 0;   origin_xyz[2] = 0;
     origin_rpy[0] = 0;   origin_rpy[1] = 0;   origin_rpy[2] = 0;

     resolution[0]=256;
     resolution[1]=256;

     clippingPlanes[0]=0.01;
     clippingPlanes[1]=2.0;

     cameraSensorPresent=false;
     proximitySensorPresent=false;
     gazeboSpec=false;

     nSensor = -1;
     nSensorAux = -1;
}


sensor::~sensor()
{
}

