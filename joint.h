#pragma once

#include "simLib.h"
#include <iostream>
#include <vector>
#include <bitset>
#include "4X4Matrix.h"


class joint
{
public:
    //Variables
    float origin_xyz[3];
    float origin_rpy[3];
    float axis[3];
    float lowerLimit;
    float upperLimit;
    bool jointLimitsSpecified;
    float effortLimitAngular;
    float effortLimitLinear;
    float velocityLimitAngular;
    float velocityLimitLinear;

    C7Vector jointBaseFrame;

    simInt nJoint;
    std::string name;
    simInt jointType;

    std::string parentJoint;
    std::string parentLink;
    std::string childLink;

    //Functions
    joint(void);
    ~joint(void);
    
    //Read
    void setJointType(std::string gazeboJointType);
    void setOrigin(std::string gazebo_origin_xyz,std::string gazebo_origin_rpy);
    void setAxis(std::string gazebo_axis_xyz);
    void setParent(std::string gazebo_parentName);
    void setChild(std::string gazebo_childName);
};

