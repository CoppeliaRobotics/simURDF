#include "joint.h"
#include "commonFunctions.h"
#include <simMath/4X4Matrix.h>

joint::joint() 
{
    //Initialize arrays
     origin_xyz[0] = 0;   origin_xyz[1] = 0;   origin_xyz[2] = 0;
     origin_rpy[0] = 0;   origin_rpy[1] = 0;   origin_rpy[2] = 0;
     axis[0] = 1;         axis[1] = 0;         axis[2] = 0;

     jointBaseFrame.setIdentity();

     nJoint = -1;
     jointType=-1;
     lowerLimit=0.0;
     upperLimit=0.0;
     jointLimitsSpecified=false;

     effortLimitAngular=2.0;
     effortLimitLinear=200.0;
     velocityLimitAngular=20.0;
     velocityLimitLinear=20.0;
}


joint::~joint()
{
}

void joint::setJointType(std::string gazeboJointType)
{
        if (gazeboJointType == "revolute")
            jointType=0;
        if (gazeboJointType == "prismatic")
            jointType=1;
        if (gazeboJointType == "spherical")
            jointType=2;
        if (gazeboJointType == "continuous")
            jointType=3;
        if (gazeboJointType == "fixed")
            jointType=4;
}

void joint::setOrigin(std::string gazebo_origin_xyz,std::string gazebo_origin_rpy)
{
    stringToArray(origin_xyz,gazebo_origin_xyz);
    stringToArray(origin_rpy,gazebo_origin_rpy);
}

void joint::setParent(std::string gazebo_parentName)
{
    parentLink = gazebo_parentName;
}

void joint::setChild(std::string gazebo_childName)
{
    childLink = gazebo_childName;
}

void joint::setAxis(std::string gazebo_axis_xyz)
{
    stringToArray(axis,gazebo_axis_xyz.c_str());
}


