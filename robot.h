#pragma once

#include "v_repLib.h"
#include <iostream>
#include <vector>
#include "tinyxml2.h"
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
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *robotElement;

    std::string packagePath;
    std::string name;
    
    //Functions
    robot(std::string filenameOrUrdf,bool hideCollisionLinks,bool hideJoints,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool showConvexDecompositionDlg,bool centerAboveGround,bool makeModel,bool noSelfCollision,bool positionCtrl);
    ~robot();

    void openString(std::string urdf); //opens a URDF from string
    void openFile(std::string filenameAndPath); //opens the filenamAndPath which is given by a file dialog and sets packagePath

    void initRobotFromDoc(bool hideCollisionLinks,bool hideJoints,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool showConvexDecompositionDlg,bool centerAboveGround,bool makeModel,bool noSelfCollision,bool positionCtrl);
    
    void readJoints();
    void readLinks();
    void readSensors();

    void createJoints(bool hideJoints,bool positionCtrl);
    void createLinks(bool hideCollisionLinks,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool showConvexDecompositionDlg);
    void createSensors();

    int getJointPosition(std::string jointName);
    int getLinkPosition(std::string linkName);

    void setLocalRespondableMaskCummulative_alternate(int objHandle,bool bitSet);
};

