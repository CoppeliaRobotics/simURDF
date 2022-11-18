#pragma once

#include "simLib.h"
#include <iostream>
#include <vector>
#include <bitset>
#include "4X4Matrix.h"


class sensor
{
public:
    //Variables
    float origin_xyz[3];
    float origin_rpy[3];

    float resolution[2];
    float clippingPlanes[2];

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

