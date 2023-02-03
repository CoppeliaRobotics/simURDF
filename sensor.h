#pragma once

#include "simLib.h"
#include <iostream>
#include <vector>
#include <bitset>
#include <simMath/4X4Matrix.h>


class sensor
{
public:
    //Variables
    double origin_xyz[3];
    double origin_rpy[3];

    double resolution[2];
    double clippingPlanes[2];

    bool cameraSensorPresent;
    bool proximitySensorPresent;

    bool gazeboSpec;

    int nSensor;
    int nSensorAux;
    std::string name;

    std::string parentLink;

    //Functions
    sensor(void);
    ~sensor(void);
};

