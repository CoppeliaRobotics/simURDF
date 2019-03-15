#pragma once

#include "v_repLib.h"
#include <iostream>
#include <vector>
#include <bitset>
#include <sstream>
#include <boost/lexical_cast.hpp>

#include "4X4Matrix.h"
#include "3Vector.h"
#include "7Vector.h"
#include "3X3Matrix.h"
#include "MyMath.h"

#include "commonFunctions.h"

/**
 * An element is a <visual>, <intertial> or <collision> element
 */
class urdfVisualOrCollision
{
    public:
        urdfVisualOrCollision();

        // Position
        float xyz[3], rpy[3];

        // For geometry
        float box_size[3];              //If all the coordinates are zero do not create the object
        float sphere_size[3];           //If all the coordinates are zero do not create the object
        float cylinder_size[3];         //If all the coordinates are zero do not create the object
        float rgba[4];                  // a is ignored
        bool hasColor;
        float mesh_scaling[3];

        std::string meshFilename;
        std::string meshFilename_alt;
        simInt meshExtension;

        // The simulator identifier
        simInt n;
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
    float inertial_xyz[3];
    float inertial_rpy[3];
    bool inertiaPresent;

    //Common variables
    float inertia[9];
    float mass;

    std::string name;

    //This variables does not came from the urdf.
    std::string parent;
    simInt nParent;

    std::string child;
    simInt nChild;

    simInt nLinkVisual;
    simInt nLinkCollision;
    
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
    void setMeshFilename(std::string packagePath,std::string meshFilename,std::string choose);
    void setColor(std::string color);
    void verifyInertia();
    int scaleShapeIfRequired(int shapeHandle,float scalingFactors[3]);
    void createLink(bool hideCollisionLinks,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool& showConvexDecompositionDlg);
};

