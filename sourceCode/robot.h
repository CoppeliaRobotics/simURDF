#pragma once

#include <simLib/simLib.h>
#include <iostream>
#include <vector>
#include "tinyxml2/tinyxml2.h"
#include "link.h"
#include "joint.h"
#include "sensor.h"
#include <simMath/4X4Matrix.h>



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
    std::vector<int> parentlessObjects;
    
    //Functions
    robot(std::string filenameOrUrdf,int options,const char* packageReplaceStr);
    ~robot();

    void openString(std::string urdf); //opens a URDF from string
    void openFile(std::string filenameAndPath); //opens the filenamAndPath which is given by a file dialog and sets packagePath

    void initRobotFromDoc(int options,const char* packageReplaceStr,const char* urdfFile);
    
    void readJoints();
    void readLinks(const char* packageReplaceStr, const char* urdfFile);
    void readSensors();

    void createJoints(bool hideJoints,bool positionCtrl);
    void createLinks(int options);
    void createSensors();

    int getJointPosition(std::string jointName);
    int getLinkPosition(std::string linkName);

    void setLocalRespondableMaskCummulative_alternate(int objHandle,bool bitSet);
};

