#pragma once

#include <simLib.h>
#include <iostream>
#include <vector>
#include <bitset>
#include <simMath/4X4Matrix.h>


class joint
{
public:
    //Variables
    double origin_xyz[3];
    double origin_rpy[3];
    double axis[3];
    double lowerLimit;
    double upperLimit;
    bool jointLimitsSpecified;
    double effortLimitAngular;
    double effortLimitLinear;
    double velocityLimitAngular;
    double velocityLimitLinear;

    C7Vector jointBaseFrame;

    int nJoint;
    std::string name;
    int jointType;

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

