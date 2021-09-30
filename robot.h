#pragma once

#include "simLib.h"
#include <iostream>
#include <vector>
#include "tinyxml2/tinyxml2.h"
#include "link.h"
#include "joint.h"
#include "sensor.h"
#include "4X4Matrix.h"



class robot
{
private:
    std::vector<joint*> vJoints;
    std::vector<urdfLink*> vLinks;
    std::vector<sensor*> vSensors;

public:
    //Data
    simExtUrdf::tinyxml2::XMLDocument doc;
    simExtUrdf::tinyxml2::XMLElement *robotElement;

    std::string packagePath;
    std::string name;
    
    //Functions
    robot(std::string filenameOrUrdf,bool hideCollisionLinks,bool hideJoints,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool convexHull,bool centerAboveGround,bool makeModel,bool noSelfCollision,bool positionCtrl,const char* packageReplaceStr);
    ~robot();

    void openString(std::string urdf); //opens a URDF from string
    void openFile(std::string filenameAndPath); //opens the filenamAndPath which is given by a file dialog and sets packagePath

    void initRobotFromDoc(bool hideCollisionLinks,bool hideJoints,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool convexHull,bool centerAboveGround,bool makeModel,bool noSelfCollision,bool positionCtrl,const char* packageReplaceStr);
    
    void readJoints();
    void readLinks(const char* packageReplaceStr);
    void readSensors();

    void createJoints(bool hideJoints,bool positionCtrl);
    void createLinks(bool hideCollisionLinks,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool convexHull);
    void createSensors();

    int getJointPosition(std::string jointName);
    int getLinkPosition(std::string linkName);

    void setLocalRespondableMaskCummulative_alternate(int objHandle,bool bitSet);
};

