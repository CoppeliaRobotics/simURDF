#pragma once

#include <simLib/simLib.h>
#include <iostream>
#include <vector>
#include <bitset>
#include <sstream>
#include <boost/lexical_cast.hpp>

#include <simMath/4X4Matrix.h>
#include <simMath/3Vector.h>
#include <simMath/7Vector.h>
#include <simMath/3X3Matrix.h>
#include <simMath/mathFuncs.h>

#include "commonFunctions.h"

/**
 * An element is a <visual>, <intertial> or <collision> element
 */
class urdfVisualOrCollision
{
    public:
        urdfVisualOrCollision();

        // Position
        double xyz[3], rpy[3];

        // For geometry
        double box_size[3];
        double sphere_size[3];
        double cylinder_size[3];
        float rgba[4];                  // a is ignored
        bool hasColor;
        double mesh_scaling[3];

        std::string meshFilename;
        std::string meshFilename_alt;
        int meshExtension;

        // The simulator identifier
        int n;
};

class urdfLink
{
public:
    std::vector<urdfVisualOrCollision> visuals;
    std::vector<urdfVisualOrCollision> collisions;
    void addVisual();
    urdfVisualOrCollision &currentVisual();
    void addCollision();
    urdfVisualOrCollision &currentCollision();

    //Variables Inertial
    double inertial_xyz[3];
    double inertial_rpy[3];
    bool inertiaPresent;

    //Common variables
    double inertia[9];
    double mass;

    std::string name;

    //This variables does not came from the urdf.
    std::string parent;
    int nParent;

    std::string child;
    int nChild;

    int nLinkVisual;
    int nLinkCollision;
    
    //Functions
    urdfLink();
    ~urdfLink();
    
    void setPosition(std::string gazebo_xyz,std::string choose);
    void setRotation(std::string gazebo_rpy,std::string choose);
    void setBox(std::string gazebo_size,std::string choose);
    void setSphere(std::string gazebo_radius,std::string choose);
    void setCylinder(std::string gazebo_radius,std::string gazebo_length,std::string choose);
    void setMass(std::string gazebo_mass);
    void setInertia(int position, std::string gazebo_inertia_number);
    void setMeshFilename(std::string packagePath,std::string meshFilename,std::string choose,const char* packageReplaceStr,const char* urdfFile);
    void setColor(std::string color);
    void verifyInertia();
    int scaleShapeIfRequired(int shapeHandle,double scalingFactors[3]);
    void createLink(int options);
};

