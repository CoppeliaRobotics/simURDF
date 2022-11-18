#include "robot.h"
#include <bitset>
#include <string>


robot::robot(std::string filenameOrUrdf,bool hideCollisionLinks,bool hideJoints,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool convexHull,bool centerAboveGround,bool makeModel,bool noSelfCollision,bool positionCtrl,const char* packageReplaceStr)
{
    if (filenameOrUrdf.compare(0, 5, "<?xml") == 0) {
        openString(filenameOrUrdf);
    } else {
        openFile(filenameOrUrdf);
    }

    this->initRobotFromDoc(hideCollisionLinks, hideJoints, convexDecomposeNonConvexCollidables, createVisualIfNone, convexHull, centerAboveGround, makeModel, noSelfCollision, positionCtrl,packageReplaceStr);
}

void robot::initRobotFromDoc(bool hideCollisionLinks,bool hideJoints,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool convexHull,bool centerAboveGround,bool makeModel,bool noSelfCollision,bool positionCtrl,const char* packageReplaceStr)
{
    robotElement = doc.FirstChildElement("robot");
    if(robotElement==nullptr)
    {
        printToConsole(sim_verbosity_scripterrors,"there is no robot in the file.");
        return;
    }

    //Save the robot's name
    name=robotElement->Attribute("name");

    readJoints();
    readLinks(packageReplaceStr);
    readSensors();

    std::string txt("there are "+boost::lexical_cast<std::string>(vJoints.size())+" joints.");
    printToConsole(sim_verbosity_scriptinfos,txt.c_str());

    txt="there are "+boost::lexical_cast<std::string>(vLinks.size())+" links.";
    printToConsole(sim_verbosity_scriptinfos,txt.c_str());

    txt="there are "+boost::lexical_cast<std::string>(vSensors.size())+" sensors.";
    printToConsole(sim_verbosity_scriptinfos,txt.c_str());

    createJoints(hideJoints,positionCtrl);
    createLinks(hideCollisionLinks,convexDecomposeNonConvexCollidables,createVisualIfNone,convexHull);
    createSensors();

    std::vector<int> parentlessObjects;
    std::vector<int> allShapes;
    std::vector<int> allObjects;
    std::vector<int> allSensors;
    for (int i=0;i<int(vLinks.size());i++)
    {
        if (simGetObjectParent(vLinks[i]->nLinkVisual)==-1)
            parentlessObjects.push_back(vLinks[i]->nLinkVisual);
        allObjects.push_back(vLinks[i]->nLinkVisual);
        allShapes.push_back(vLinks[i]->nLinkVisual);

        if (simGetObjectParent(vLinks[i]->nLinkCollision)==-1)
            parentlessObjects.push_back(vLinks[i]->nLinkCollision);
        allObjects.push_back(vLinks[i]->nLinkCollision);
        allShapes.push_back(vLinks[i]->nLinkCollision);
    }
    for (int i=0;i<int(vJoints.size());i++)
    {
        if (vJoints[i]->nJoint!=-1)
        {
            if (simGetObjectParent(vJoints[i]->nJoint)==-1)
                parentlessObjects.push_back(vJoints[i]->nJoint);
            allObjects.push_back(vJoints[i]->nJoint);
        }
    }
    for (int i=0;i<int(vSensors.size());i++)
    {
        if (vSensors[i]->nSensor!=-1)
        {
            if (simGetObjectParent(vSensors[i]->nSensor)==-1)
                parentlessObjects.push_back(vSensors[i]->nSensor);
            allObjects.push_back(vSensors[i]->nSensor);
            allSensors.push_back(vSensors[i]->nSensor);
        }
        if (vSensors[i]->nSensorAux!=-1)
        {
            allObjects.push_back(vSensors[i]->nSensorAux);
            allSensors.push_back(vSensors[i]->nSensorAux);
        }
    }

    // If we want to alternate respondable mask:
    if (!noSelfCollision)
    {
        for (int i=0;i<int(parentlessObjects.size());i++)
            setLocalRespondableMaskCummulative_alternate(parentlessObjects[i],true);
    }

    // Now center the model:
    if (centerAboveGround)
    {
        bool firstValSet=false;
        C3Vector minV,maxV;
        for (int shNb=0;shNb<int(allShapes.size());shNb++)
        {
            float* vertices;
            int verticesSize;
            int* indices;
            int indicesSize;
            if (simGetShapeMesh(allShapes[shNb],&vertices,&verticesSize,&indices,&indicesSize,nullptr)!=-1)
            {
                C7Vector tr;
                simGetObjectPosition(allShapes[shNb],-1,tr.X.data);
                C3Vector euler;
                simGetObjectOrientation(allShapes[shNb],-1,euler.data);
                tr.Q.setEulerAngles(euler);
                for (int i=0;i<verticesSize/3;i++)
                {
                    C3Vector v(vertices+3*i);
                    v*=tr;
                    if (!firstValSet)
                    {
                        minV=v;
                        maxV=v;
                        firstValSet=true;
                    }
                    else
                    {
                        minV.keepMin(v);
                        maxV.keepMax(v);
                    }
                }
                simReleaseBuffer((char*)vertices);
                simReleaseBuffer((char*)indices);
            }
        }

        C3Vector shiftAmount((minV+maxV)*-0.5f);
        shiftAmount(2)+=(maxV(2)-minV(2))*0.5f;
        for (int i=0;i<int(parentlessObjects.size());i++)
        {
            C3Vector p;
            simGetObjectPosition(parentlessObjects[i],-1,p.data);
            p+=shiftAmount;
            simSetObjectPosition(parentlessObjects[i],-1,p.data);
        }
    }

    // Now create a model bounding box if that makes sense:
    if ((makeModel)&&(parentlessObjects.size()==1))
    {
        int p=simGetModelProperty(parentlessObjects[0]);
        p|=sim_modelproperty_not_model;
        simSetModelProperty(parentlessObjects[0],p-sim_modelproperty_not_model);

        for (int i=0;i<int(allObjects.size());i++)
        {
            if (allObjects[i]!=parentlessObjects[0])
            {
                int p=simGetObjectProperty(allObjects[i]);
                simSetObjectProperty(allObjects[i],p|sim_objectproperty_selectmodelbaseinstead);
            }
        }

        for (int i=0;i<int(allSensors.size());i++)
        {
            if (allSensors[i]!=parentlessObjects[0])
            {
                int p=simGetObjectProperty(allSensors[i]);
                simSetObjectProperty(allSensors[i],p|sim_objectproperty_dontshowasinsidemodel); // sensors are usually large and it is ok if they do not appear as inside of the model bounding box!
            }
        }

    }

    // Now select all new objects:
    simRemoveObjectFromSelection(sim_handle_all,-1);
    for (int i=0;i<int(allObjects.size());i++)
        simAddObjectToSelection(sim_handle_single,allObjects[i]);
}


robot::~robot()
{
    for (int i=0;i<int(vLinks.size());i++)
        delete vLinks[i];
    for (int i=0;i<int(vJoints.size());i++)
        delete vJoints[i];
    for (int i=0;i<int(vSensors.size());i++)
        delete vSensors[i];
}

void robot::openString(std::string urdf)
{
    //The URDF is directly passed as a string
    if(doc.Parse(urdf.c_str())!=simExtUrdf::tinyxml2::XML_NO_ERROR)
    {
        //something went wrong
        printToConsole(sim_verbosity_scripterrors,"the given string is not a valid URDF document.");
        return;
    }
}

void robot::openFile(std::string filenameAndPath)
{ 
    //Open the file
    if(doc.LoadFile((char*)filenameAndPath.c_str())!=simExtUrdf::tinyxml2::XML_NO_ERROR)
    {
        //something went wrong
        printToConsole(sim_verbosity_scripterrors,"file couldn't be opened.");
        return;
    }
#ifdef WIN_SIM
    //Set the path to the package
    int cutPackagePath = filenameAndPath.find_last_of("/");
    packagePath=filenameAndPath.substr(0,cutPackagePath);
    cutPackagePath = packagePath.find_last_of("/");
    packagePath=packagePath.substr(0,cutPackagePath);  //I do it twice to get  N:/drcsim-1.3/models/ so I can replace it for package://
#endif
}

void robot::readJoints()
{
    if (robotElement->FirstChildElement("joint") == nullptr)
        return;
    simExtUrdf::tinyxml2::XMLElement* jointElement;
    jointElement = robotElement->FirstChildElement("joint");

    while (jointElement)
    {
        joint* Joint=new joint();
        if (jointElement->Attribute("type") != nullptr)
            Joint->setJointType(jointElement->Attribute("type"));
        if (jointElement->Attribute("name") != nullptr)
            Joint->name = jointElement->Attribute("name");

        simExtUrdf::tinyxml2::XMLElement* joint_originElement = jointElement->FirstChildElement("origin");
        if (joint_originElement!=nullptr)
        {
            if ( (joint_originElement->Attribute("xyz") != nullptr) && (joint_originElement->Attribute("rpy") != nullptr) )
                Joint->setOrigin(joint_originElement->Attribute("xyz"),joint_originElement->Attribute("rpy"));
            if ( (joint_originElement->Attribute("xyz") != nullptr) && (joint_originElement->Attribute("rpy") == nullptr) )
                Joint->setOrigin(joint_originElement->Attribute("xyz"),"0 0 0");
            if ( (joint_originElement->Attribute("xyz") == nullptr) && (joint_originElement->Attribute("rpy") != nullptr) )
                Joint->setOrigin("0 0 0",joint_originElement->Attribute("rpy"));
        }

    
        simExtUrdf::tinyxml2::XMLElement* joint_axisElement = jointElement->FirstChildElement("axis");
        if (joint_axisElement!=nullptr)
        {
            if (joint_axisElement->Attribute("xyz") != nullptr)
                Joint->setAxis(joint_axisElement->Attribute("xyz"));
        }
                
        simExtUrdf::tinyxml2::XMLElement* joint_parentElement = jointElement->FirstChildElement("parent");
        if (joint_parentElement!=nullptr)
        {  
            if (joint_parentElement->Attribute("link") != nullptr)
                Joint->setParent(joint_parentElement->Attribute("link"));
        }

        simExtUrdf::tinyxml2::XMLElement* joint_childElement = jointElement->FirstChildElement("child");
        if (joint_childElement!=nullptr)
        {
            if (joint_childElement->Attribute("link") != nullptr)
                Joint->setChild(joint_childElement->Attribute("link"));
        }

        simExtUrdf::tinyxml2::XMLElement* joint_limitElement = jointElement->FirstChildElement("limit");
        if(joint_limitElement!=nullptr)
        {
            if (joint_limitElement->Attribute("lower")!=nullptr)
            {
                Joint->lowerLimit=getFloat(joint_limitElement->Attribute("lower"));
                Joint->jointLimitsSpecified=true;
            }
            if (joint_limitElement->Attribute("upper")!=nullptr)
            {
                Joint->upperLimit=getFloat(joint_limitElement->Attribute("upper"));
                Joint->jointLimitsSpecified=true;
            }
            if (joint_limitElement->Attribute("effort")!=nullptr)
            {
                Joint->effortLimitAngular=getFloat(joint_limitElement->Attribute("effort"));
                Joint->effortLimitLinear=getFloat(joint_limitElement->Attribute("effort"));
            }
            if (joint_limitElement->Attribute("velocity")!=nullptr)
            {
                Joint->velocityLimitAngular=getFloat(joint_limitElement->Attribute("velocity"));
                Joint->velocityLimitLinear=getFloat(joint_limitElement->Attribute("velocity"));
            }
        }

        vJoints.push_back(Joint);
        jointElement = jointElement->NextSiblingElement("joint"); //moves to the next joint
    }
}

void robot::readLinks(const char* packageReplaceStr)
{
    simExtUrdf::tinyxml2::XMLElement* linkElement =  robotElement->FirstChildElement("link");
    while (linkElement)
    {
        urdfLink* Link=new urdfLink();
        Link->name = linkElement->Attribute("name");
        
        //INERTIAL
        simExtUrdf::tinyxml2::XMLElement* inertialElement = linkElement->FirstChildElement("inertial");
        if(inertialElement!=nullptr)
        {
            Link->inertiaPresent=true;
            simExtUrdf::tinyxml2::XMLElement* inertial_originElement = inertialElement->FirstChildElement("origin");
            if(inertial_originElement != nullptr)
            {
                if (inertial_originElement->Attribute("xyz")!= nullptr)
                    Link->setPosition(inertial_originElement->Attribute("xyz"),"inertial");
                if (inertial_originElement->Attribute("rpy")!= nullptr)
                    Link->setRotation(inertial_originElement->Attribute("rpy"),"inertial");
            }

            simExtUrdf::tinyxml2::XMLElement* inertial_massElement = inertialElement->FirstChildElement("mass");
            if(inertial_massElement!= nullptr)
            {
                if (inertial_massElement->Attribute("value") != nullptr)
                    Link->setMass(inertial_massElement->Attribute("value"));
            }
            
            simExtUrdf::tinyxml2::XMLElement* inertial_inertiaElement = inertialElement->FirstChildElement("inertia");
            if(inertial_inertiaElement != nullptr)
            {
                if (inertial_inertiaElement->Attribute("ixx") != nullptr){ Link->setInertia(0,inertial_inertiaElement->Attribute("ixx")); }
                if (inertial_inertiaElement->Attribute("ixy") != nullptr){ Link->setInertia(1,inertial_inertiaElement->Attribute("ixy")); Link->setInertia(3,inertial_inertiaElement->Attribute("ixy"));}
                if (inertial_inertiaElement->Attribute("ixz") != nullptr){ Link->setInertia(2,inertial_inertiaElement->Attribute("ixz")); Link->setInertia(6,inertial_inertiaElement->Attribute("ixz")); }
                if (inertial_inertiaElement->Attribute("iyy") != nullptr){ Link->setInertia(4,inertial_inertiaElement->Attribute("iyy")); }
                if (inertial_inertiaElement->Attribute("iyz") != nullptr){ Link->setInertia(5,inertial_inertiaElement->Attribute("iyz")); Link->setInertia(7,inertial_inertiaElement->Attribute("iyz")); }
                if (inertial_inertiaElement->Attribute("izz") != nullptr){ Link->setInertia(8,inertial_inertiaElement->Attribute("izz")); }
            }
            Link->verifyInertia(); // required because some of the URDF files have inertia matrices that are 0!!!
        }

        //VISUAL
        simExtUrdf::tinyxml2::XMLNode* visualElement = linkElement->FirstChildElement("visual");
        while(visualElement != nullptr)
        {
            Link->addVisual();

            simExtUrdf::tinyxml2::XMLElement* visual_originElement = visualElement->FirstChildElement("origin");
            if(visual_originElement != nullptr)
            {
                if (visual_originElement->Attribute("xyz")!= nullptr)
                    Link->setPosition(visual_originElement->Attribute("xyz"),"visual");
                if (visual_originElement->Attribute("rpy")!= nullptr)
                    Link->setRotation(visual_originElement->Attribute("rpy"),"visual");
            }

            simExtUrdf::tinyxml2::XMLElement* visual_geometryElement = visualElement->FirstChildElement("geometry");
            if(visual_geometryElement!=nullptr)
            {

                simExtUrdf::tinyxml2::XMLElement* visual_geometry_meshElement = visual_geometryElement->FirstChildElement("mesh");
                if(visual_geometry_meshElement!=nullptr)
                {
                    if (visual_geometry_meshElement->Attribute("filename") != nullptr)
                        Link->setMeshFilename(packagePath,visual_geometry_meshElement->Attribute("filename"),"visual",packageReplaceStr);
                    if (visual_geometry_meshElement->Attribute("scale") != nullptr)
                        stringToArray(Link->currentVisual().mesh_scaling,visual_geometry_meshElement->Attribute("scale"));
                }
                simExtUrdf::tinyxml2::XMLElement* visual_geometry_boxElement = visual_geometryElement->FirstChildElement("box");
                if(visual_geometry_boxElement!=nullptr)
                {
                    if(visual_geometry_boxElement->Attribute("size") != nullptr){Link->setBox(visual_geometry_boxElement->Attribute("size"),"visual");}
                } 
                simExtUrdf::tinyxml2::XMLElement* visual_geometry_sphereElement = visual_geometryElement->FirstChildElement("sphere");
                if(visual_geometry_sphereElement!=nullptr)
                {
                    if(visual_geometry_sphereElement->Attribute("radius") != nullptr){Link->setSphere(visual_geometry_sphereElement->Attribute("radius"),"visual");}
                } 
                simExtUrdf::tinyxml2::XMLElement* visual_geometry_cylinderElement = visual_geometryElement->FirstChildElement("cylinder");
                if(visual_geometry_cylinderElement!=nullptr)
                {
                    if( visual_geometry_cylinderElement->Attribute("radius") != nullptr && visual_geometry_cylinderElement->Attribute("length") != nullptr)
                    {Link->setCylinder( visual_geometry_cylinderElement->Attribute("radius"),visual_geometry_cylinderElement->Attribute("length"),"visual");}
                } 

            }
            simExtUrdf::tinyxml2::XMLElement* visual_materialElement = visualElement->FirstChildElement("material");
            if(visual_materialElement!=nullptr)
            {
                simExtUrdf::tinyxml2::XMLElement* visual_geometry_colorElement = visual_materialElement->FirstChildElement("color");
                if(visual_geometry_colorElement!=nullptr)
                {
                    if (visual_geometry_colorElement->Attribute("rgba") != nullptr)
                        Link->setColor(visual_geometry_colorElement->Attribute("rgba"));
                }
            }
            visualElement = visualElement->NextSiblingElement("visual");
        }
        //COLLISION
        simExtUrdf::tinyxml2::XMLNode* collisionElement = linkElement->FirstChildElement("collision");
        while(collisionElement != nullptr)
        {
            Link->addCollision();

            simExtUrdf::tinyxml2::XMLElement* collision_originElement = collisionElement->FirstChildElement("origin");
            if(collision_originElement != nullptr)
            {
                if (collision_originElement->Attribute("xyz")!= nullptr)
                    Link->setPosition(collision_originElement->Attribute("xyz"),"collision");
                if (collision_originElement->Attribute("rpy")!= nullptr)
                    Link->setRotation(collision_originElement->Attribute("rpy"),"collision");
            }

            simExtUrdf::tinyxml2::XMLElement* collision_geometryElement = collisionElement->FirstChildElement("geometry");
            if(collision_geometryElement!=nullptr)
            {

                simExtUrdf::tinyxml2::XMLElement* collision_geometry_meshElement = collision_geometryElement->FirstChildElement("mesh");
                if(collision_geometry_meshElement!=nullptr)
                {
                    if (collision_geometry_meshElement->Attribute("filename") != nullptr)
                        Link->setMeshFilename(packagePath,collision_geometry_meshElement->Attribute("filename"),"collision",packageReplaceStr);
                    if (collision_geometry_meshElement->Attribute("scale") != nullptr)
                        stringToArray(Link->currentCollision().mesh_scaling,collision_geometry_meshElement->Attribute("scale"));
                }
                simExtUrdf::tinyxml2::XMLElement* collision_geometry_boxElement = collision_geometryElement->FirstChildElement("box");
                if(collision_geometry_boxElement!=nullptr)
                {
                    if(collision_geometry_boxElement->Attribute("size") != nullptr)
                    {Link->setBox(collision_geometry_boxElement->Attribute("size"),"collision");}
                } 
                simExtUrdf::tinyxml2::XMLElement* collision_geometry_sphereElement = collision_geometryElement->FirstChildElement("sphere");
                if(collision_geometry_sphereElement!=nullptr)
                {
                    if(collision_geometry_sphereElement->Attribute("radius") != nullptr)
                    {Link->setSphere(collision_geometry_sphereElement->Attribute("radius"),"collision");}
                } 
                simExtUrdf::tinyxml2::XMLElement* collision_geometry_cylinderElement = collision_geometryElement->FirstChildElement("cylinder");
                if(collision_geometry_cylinderElement!=nullptr)
                {
                    if(collision_geometry_cylinderElement->Attribute("radius") != nullptr && collision_geometry_cylinderElement->Attribute("length") != nullptr)
                    {Link->setCylinder( collision_geometry_cylinderElement->Attribute("radius"),collision_geometry_cylinderElement->Attribute("length"),"collision");}
                } 

            }
            collisionElement = collisionElement->NextSiblingElement("collision");
        }
        
        vLinks.push_back(Link);
        linkElement = linkElement->NextSiblingElement("link"); //moves to the next link
    }
}

void robot::readSensors()
{
    { // URDF sensors:
    simExtUrdf::tinyxml2::XMLElement* sensorElement;
    sensorElement = robotElement->FirstChildElement("sensor");
    while (sensorElement)
    {
        sensor* Sensor=new sensor();
        Sensor->name = sensorElement->Attribute("name");
        Sensor->gazeboSpec=true;


        simExtUrdf::tinyxml2::XMLElement* sensor_parentElement = sensorElement->FirstChildElement("parent");
        if (sensor_parentElement!=nullptr)
        {
            if (sensor_parentElement->Attribute("link") != nullptr)
                Sensor->parentLink=sensor_parentElement->Attribute("link");
        }

        simExtUrdf::tinyxml2::XMLElement* sensor_originElement = sensorElement->FirstChildElement("origin");
        if(sensor_originElement != nullptr)
        {
            if (sensor_originElement->Attribute("xyz")!= nullptr)
                stringToArray(Sensor->origin_xyz,sensor_originElement->Attribute("xyz"));
            if (sensor_originElement->Attribute("rpy")!= nullptr)
                stringToArray(Sensor->origin_rpy,sensor_originElement->Attribute("rpy"));
        }

        simExtUrdf::tinyxml2::XMLElement* sensor_cameraElement = sensorElement->FirstChildElement("camera");
        Sensor->cameraSensorPresent=(sensor_cameraElement != nullptr);
        if(sensor_cameraElement != nullptr)
        {
            simExtUrdf::tinyxml2::XMLElement* sensor_imageElement = sensor_cameraElement->FirstChildElement("image");
            if(sensor_imageElement != nullptr)
            {
                if (sensor_imageElement->Attribute("width")!= nullptr)
                    Sensor->resolution[0]=getInt(sensor_imageElement->Attribute("width"));
                if (sensor_imageElement->Attribute("height")!= nullptr)
                    Sensor->resolution[1]=getInt(sensor_imageElement->Attribute("height"));
                if (sensor_imageElement->Attribute("near")!= nullptr)
                    Sensor->clippingPlanes[0]=getFloat(sensor_imageElement->Attribute("near"));
                if (sensor_imageElement->Attribute("far")!= nullptr)
                    Sensor->clippingPlanes[1]=getFloat(sensor_imageElement->Attribute("far"));
            }
        }
        simExtUrdf::tinyxml2::XMLElement* sensor_rayElement = sensorElement->FirstChildElement("ray");
        Sensor->proximitySensorPresent=(sensor_rayElement != nullptr);

        vSensors.push_back(Sensor);
        sensorElement = sensorElement->NextSiblingElement("sensor"); //moves to the next sensor
    }
    }



    { // GAZEBO SENSORS:
    simExtUrdf::tinyxml2::XMLElement* gazeboElement;
    gazeboElement = robotElement->FirstChildElement("gazebo");
    while (gazeboElement)
    {
        simExtUrdf::tinyxml2::XMLElement* sensorElement;
        sensorElement = gazeboElement->FirstChildElement("sensor");
        while (sensorElement)
        {
            sensor* Sensor=new sensor();
            Sensor->name = sensorElement->Attribute("name");
            Sensor->gazeboSpec=true;


            simExtUrdf::tinyxml2::XMLElement* sensor_parentElement = sensorElement->FirstChildElement("parent");
            if (sensor_parentElement!=nullptr)
            {
                if (sensor_parentElement->Attribute("link") != nullptr)
                    Sensor->parentLink=sensor_parentElement->Attribute("link");
            }

            simExtUrdf::tinyxml2::XMLElement* sensor_originElement = sensorElement->FirstChildElement("origin");
            if(sensor_originElement != nullptr)
            {
                if (sensor_originElement->Attribute("xyz")!= nullptr)
                    stringToArray(Sensor->origin_xyz,sensor_originElement->Attribute("xyz"));
                if (sensor_originElement->Attribute("rpy")!= nullptr)
                    stringToArray(Sensor->origin_rpy,sensor_originElement->Attribute("rpy"));
            }

            simExtUrdf::tinyxml2::XMLElement* sensor_cameraElement = sensorElement->FirstChildElement("camera");
            Sensor->cameraSensorPresent=(sensor_cameraElement != nullptr);
            if(sensor_cameraElement != nullptr)
            {
                simExtUrdf::tinyxml2::XMLElement* sensor_imageElement = sensor_cameraElement->FirstChildElement("image");
                if(sensor_imageElement != nullptr)
                {
                    if (sensor_imageElement->Attribute("width")!= nullptr)
                        Sensor->resolution[0]=getInt(sensor_imageElement->Attribute("width"));
                    if (sensor_imageElement->Attribute("height")!= nullptr)
                        Sensor->resolution[1]=getInt(sensor_imageElement->Attribute("height"));
                    if (sensor_imageElement->Attribute("near")!= nullptr)
                        Sensor->clippingPlanes[0]=getFloat(sensor_imageElement->Attribute("near"));
                    if (sensor_imageElement->Attribute("far")!= nullptr)
                        Sensor->clippingPlanes[1]=getFloat(sensor_imageElement->Attribute("far"));
                }
            }
            simExtUrdf::tinyxml2::XMLElement* sensor_rayElement = sensorElement->FirstChildElement("ray");
            Sensor->proximitySensorPresent=(sensor_rayElement != nullptr);

            vSensors.push_back(Sensor);
            sensorElement = sensorElement->NextSiblingElement("sensor"); //moves to the next sensor
        }
        gazeboElement = gazeboElement->NextSiblingElement("gazebo"); //moves to the next gazebo thing
    }
    }
}

void robot::createJoints(bool hideJoints,bool positionCtrl)
{
    //Set parents and childs for all the links
    for(size_t i = 0; i < vJoints.size() ; i++)
    {
        vLinks.at(getLinkPosition(vJoints.at(i)->parentLink))->child = vJoints.at(i)->name;
        vLinks.at(getLinkPosition(vJoints.at(i)->childLink))->parent = vJoints.at(i)->name;
    }

    //Create the joints
    for(size_t i = 0; i < vJoints.size() ; i++)
    {
        //Move the joints to the positions specifieds by the urdf file
        C7Vector tmp;
        tmp.setIdentity();
        tmp.X.setData(vJoints.at(i)->origin_xyz);
        tmp.Q=getQuaternionFromRpy(vJoints.at(i)->origin_rpy);
        vJoints.at(i)->jointBaseFrame=vJoints.at(i)->jointBaseFrame*tmp;

        //Set name jointParent to each joint
        int nParentLink = getLinkPosition(vJoints.at(i)->parentLink);
        vJoints.at(i)->parentJoint = vLinks.at(nParentLink)->parent;

        //Create the joint/forceSensor/dummy:
        if (vJoints.at(i)->jointType==-1)
            vJoints.at(i)->nJoint = simCreateDummy(0.02f,nullptr); // when joint type was not recognized
        if (vJoints.at(i)->jointType==0)
            vJoints.at(i)->nJoint = simCreateJoint(sim_joint_revolute_subtype,sim_jointmode_force,2,nullptr,nullptr,nullptr);
        if (vJoints.at(i)->jointType==1)
            vJoints.at(i)->nJoint = simCreateJoint(sim_joint_prismatic_subtype,sim_jointmode_force,2,nullptr,nullptr,nullptr);
        if (vJoints.at(i)->jointType==2)
            vJoints.at(i)->nJoint = simCreateJoint(sim_joint_spherical_subtype,sim_jointmode_force,2,nullptr,nullptr,nullptr);
        if (vJoints.at(i)->jointType==3)
            vJoints.at(i)->nJoint = simCreateJoint(sim_joint_revolute_subtype,sim_jointmode_force,2,nullptr,nullptr,nullptr);
        if (vJoints.at(i)->jointType==4)
        { // when joint type is "fixed"
            int intParams[5]={1,4,4,0,0};
            float floatParams[5]={0.02f,1.0f,1.0f,0.0f,0.0f};
            vJoints.at(i)->nJoint = simCreateForceSensor(0,intParams,floatParams,nullptr);
        }

        if ( (vJoints.at(i)->jointType==0)||(vJoints.at(i)->jointType==1)||(vJoints.at(i)->jointType==3) )
        {
            if ( (vJoints.at(i)->jointType!=3)&&vJoints.at(i)->jointLimitsSpecified )
            {
                float interval[2]={vJoints.at(i)->lowerLimit,vJoints.at(i)->upperLimit-vJoints.at(i)->lowerLimit};
                simSetJointInterval(vJoints.at(i)->nJoint,0,interval);
                simSetJointPosition(vJoints.at(i)->nJoint,0.0f); // shouldn't be needed anymore from CoppeliaSim 3.2.2 rev2 upwards
            }
            if (vJoints.at(i)->jointType==0)
            { // revolute
                simSetJointForce(vJoints.at(i)->nJoint,vJoints.at(i)->effortLimitAngular);
                simSetObjectFloatParameter(vJoints.at(i)->nJoint,sim_jointfloatparam_upper_limit,vJoints.at(i)->velocityLimitAngular);
            }
            else
            { // prismatic
                simSetJointForce(vJoints.at(i)->nJoint,vJoints.at(i)->effortLimitLinear);
                simSetObjectFloatParameter(vJoints.at(i)->nJoint,sim_jointfloatparam_upper_limit,vJoints.at(i)->velocityLimitLinear);
            }
            // We turn the position control on:
            if (positionCtrl)
            {
                simSetObjectIntParameter(vJoints.at(i)->nJoint,sim_jointintparam_motor_enabled,1);
                simSetObjectIntParameter(vJoints.at(i)->nJoint,sim_jointintparam_motor_enabled,1);
            }
        }

        //Set the name:
        setSimObjectName(vJoints.at(i)->nJoint,vJoints.at(i)->name.c_str());
        if (hideJoints)
            simSetObjectIntParameter(vJoints.at(i)->nJoint,sim_objintparam_visibility_layer,512); // layer 10
    }

    //Set positions to joints from the 4x4matrix
    for(size_t i = 0; i < vJoints.size() ; i++)
    {
        simSetObjectPosition(vJoints.at(i)->nJoint,-1,vJoints.at(i)->jointBaseFrame.X.data);
        simSetObjectOrientation(vJoints.at(i)->nJoint,-1 ,vJoints.at(i)->jointBaseFrame.Q.getEulerAngles().data);
    }

    //Set joint parentship between them (thes parentship will be remove before adding the joints)
    for(size_t i = 0; i < vJoints.size() ; i++)
    {
        if (vJoints.at(i)->parentJoint.size()>0)
        {
            int parentJointIndex=getJointPosition(vJoints.at(i)->parentJoint);
            if ( parentJointIndex!= -1)
            {
                int nParentJoint = vJoints.at(parentJointIndex)->nJoint;
                int nJoint = vJoints.at(i)->nJoint;
                simSetObjectParent(nJoint,nParentJoint,false);
            }
        }
    }

    //Delete all the partnership without moving the joints but after doing that update the transform matrix
    for(size_t i = 0; i < vJoints.size() ; i++)
    { 
        C4X4Matrix tmp;  
        simGetObjectPosition(vJoints.at(i)->nJoint,-1,tmp.X.data);
        C3Vector euler;
        simGetObjectOrientation(vJoints.at(i)->nJoint,-1,euler.data);
        tmp.M.setEulerAngles(euler);
        vJoints.at(i)->jointBaseFrame = tmp;

        int nJoint = vJoints.at(i)->nJoint;
        simSetObjectParent(nJoint,-1,true);
    }

    for(size_t i = 0; i < vJoints.size() ; i++)
    {
        C4X4Matrix jointAxisMatrix;
        jointAxisMatrix.setIdentity();
        C3Vector axis(vJoints.at(i)->axis);
        C3Vector rotAxis;
        float rotAngle=0.0f;
        if (axis(2)<1.0f)
        {
            if (axis(2)<=-1.0f)
                rotAngle=3.14159265359f;
            else
                rotAngle=acosf(axis(2));
            rotAxis(0)=-axis(1);
            rotAxis(1)=axis(0);
            rotAxis(2)=0.0f;
            rotAxis.normalize();
            C7Vector m(jointAxisMatrix);
            float alpha=-atan2(rotAxis(1),rotAxis(0));
            float beta=atan2(-sqrt(rotAxis(0)*rotAxis(0)+rotAxis(1)*rotAxis(1)),rotAxis(2));
            C7Vector r;
            r.X.clear();
            r.Q.setEulerAngles(0.0f,0.0f,alpha);
            m=r*m;
            r.Q.setEulerAngles(0.0f,beta,0.0f);
            m=r*m;
            r.Q.setEulerAngles(0.0f,0.0f,rotAngle);
            m=r*m;
            r.Q.setEulerAngles(0.0f,-beta,0.0f);
            m=r*m;
            r.Q.setEulerAngles(0.0f,0.0f,-alpha);
            m=r*m;
            jointAxisMatrix=m.getMatrix();
        }
        C4Vector q((vJoints.at(i)->jointBaseFrame*jointAxisMatrix).Q);
        simSetObjectOrientation(vJoints.at(i)->nJoint,-1,q.getEulerAngles().data);
    }
}

void robot::createLinks(bool hideCollisionLinks,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool convexHull)
{
    for(size_t i = 0; i < vLinks.size() ; i++)
    {
        urdfLink *Link = vLinks.at(i);
        Link->createLink(hideCollisionLinks,convexDecomposeNonConvexCollidables,createVisualIfNone,convexHull);

        // We attach the collision link to a joint. If the collision link isn't there, we use the visual link instead:

        C7Vector linkInitialConf;
        C7Vector linkDesiredConf;
        int effectiveLinkHandle=-1;
        if (Link->nLinkCollision!=-1)
        {
            effectiveLinkHandle=Link->nLinkCollision;
            // Collision object position and orientation is already set in the Link
            float xyz[3] = {0,0,0};
            float rpy[3] = {0,0,0};
            linkDesiredConf.X.setData(xyz);
            linkDesiredConf.Q=getQuaternionFromRpy(rpy);
        }
        else
        {
            effectiveLinkHandle = Link->nLinkVisual;
            if (effectiveLinkHandle != -1)
            {
                // Visual object position and orientation is already set in the Link
                float xyz[3] = {0,0,0};
                float rpy[3] = {0,0,0};
                linkDesiredConf.X.setData(xyz);
                linkDesiredConf.Q=getQuaternionFromRpy(rpy);
            }
        }
        simGetObjectPosition(effectiveLinkHandle,-1,linkInitialConf.X.data);
        C3Vector euler;
        simGetObjectOrientation(effectiveLinkHandle,-1,euler.data);
        linkInitialConf.Q.setEulerAngles(euler);

        C7Vector trAbs(linkDesiredConf*linkInitialConf); // still local

        if (Link->parent.size()>0)
        {
            int parentJointIndex=getJointPosition(Link->parent);
            if( parentJointIndex!= -1)
            {
                joint* Joint = vJoints.at(parentJointIndex);
                trAbs=Joint->jointBaseFrame*trAbs; // now absolute
            }
        }



        //set the transfrom matrix to the object
        simSetObjectPosition(effectiveLinkHandle,-1,trAbs.X.data);
        simSetObjectOrientation(effectiveLinkHandle,-1,trAbs.Q.getEulerAngles().data);
    }

    // Finally the real parenting:
    for(size_t i = 0; i < vJoints.size() ; i++)
    {
        int parentLinkIndex=getLinkPosition(vJoints.at(i)->parentLink);
        if (parentLinkIndex!=-1)
        {
            urdfLink* parentLink=vLinks[parentLinkIndex];
            if (parentLink->nLinkCollision!=-1)
                simSetObjectParent(vJoints.at(i)->nJoint,parentLink->nLinkCollision,true);
            else
                simSetObjectParent(vJoints.at(i)->nJoint,parentLink->nLinkVisual,true);
        }

        int childLinkIndex=getLinkPosition(vJoints.at(i)->childLink);
        if (childLinkIndex!=-1)
        {
            urdfLink* childLink=vLinks[childLinkIndex];
            if (childLink->nLinkCollision!=-1)
                simSetObjectParent(childLink->nLinkCollision,vJoints.at(i)->nJoint,true);
            else
                simSetObjectParent(childLink->nLinkVisual,vJoints.at(i)->nJoint,true);
        }
    }
}


void robot::createSensors()
{
        for(size_t i = 0; i < vSensors.size() ; i++)
        {
            sensor *Sensor = vSensors.at(i);
            if (Sensor->gazeboSpec)
                printToConsole(sim_verbosity_scripterrors,"sensor will not be created: the URDF specification is supported, but this is a Gazebo tag which is not documented as it seems.");
            else
            {
                if (Sensor->cameraSensorPresent)
                {
                    int intParams[4]={(int)Sensor->resolution[0],(int)Sensor->resolution[1],0,0};
                    float floatParams[11]={Sensor->clippingPlanes[0],Sensor->clippingPlanes[1],60.0f*piValue/180.0f,0.2f,0.2f,0.4f,0.0f,0.0f,0.0f,0.0f,0.0f};
                    Sensor->nSensor=simCreateVisionSensor(1,intParams,floatParams,nullptr);
                    //Set the name:
                    setSimObjectName(Sensor->nSensor,std::string(name+"_camera").c_str());
                }
                int proxSensHandle=-1;
                if (Sensor->proximitySensorPresent)
                { // Proximity sensors seem to be very very specific and not general / generic at all. How come?! I.e. a succession of ray description (with min/max distances) would do
                    int intParams[8]={16,16,1,4,16,1,0,0};
                    float floatParams[15]={0.0f,0.48f,0.1f,0.1f,0.1f,0.1f,0.0f,0.02f,0.02f,30.0f*piValue/180.0f,piValue/2.0f,0.0f,0.02f,0.0f,0.0f};
                    proxSensHandle=simCreateProximitySensor(sim_proximitysensor_cone_subtype,sim_objectspecialproperty_detectable_all,0,intParams,floatParams,nullptr);
                    //Set the name:
                    setSimObjectName(proxSensHandle,std::string(Sensor->name+"_proximity").c_str());
                }
                // the doc doesn't state if a vision and proximity sensor can be declared at the same time...
                if (proxSensHandle!=-1)
                {
                    if (Sensor->nSensor!=-1)
                    {
                        Sensor->nSensorAux=proxSensHandle;
                        simSetObjectParent(Sensor->nSensorAux,Sensor->nSensor,true);
                    }
                    else
                        Sensor->nSensor=proxSensHandle;
                }

                // Find the local configuration:
                C7Vector sensorLocal;
                sensorLocal.X.setData(Sensor->origin_xyz);
                sensorLocal.Q=getQuaternionFromRpy(Sensor->origin_rpy);
                C4Vector rot(0.0f,0.0f,piValue); // the CoppeliaSim sensors are rotated by 180deg around the Z-axis
                sensorLocal.Q=sensorLocal.Q*rot;


                // We attach the sensor to a link:
                C7Vector x;
                x.setIdentity();
                int parentLinkIndex=getLinkPosition(Sensor->parentLink);
                if (parentLinkIndex!=-1)
                {
                    if (vLinks.at(parentLinkIndex)->parent.size()>0)
                    {
                        int parentJointLinkIndex=getJointPosition(vLinks.at(parentLinkIndex)->parent);
                        if (parentJointLinkIndex!=-1)
                            x=vJoints.at(parentJointLinkIndex)->jointBaseFrame;
                    }
                }
                C7Vector sensorGlobal(x*sensorLocal);
                if (Sensor->nSensor!=-1)
                {
                    simSetObjectPosition(Sensor->nSensor,-1,sensorGlobal.X.data);
                    simSetObjectOrientation(Sensor->nSensor,-1,sensorGlobal.Q.getEulerAngles().data);
                }
                if ((parentLinkIndex!=-1)&&(Sensor->nSensor!=-1))
                {
                    if (vLinks.at(parentLinkIndex)->visuals.size()!=0)
                        simSetObjectParent(Sensor->nSensor,vLinks.at(parentLinkIndex)->nLinkVisual,true);
                    if (vLinks.at(parentLinkIndex)->collisions.size()!=0)
                        simSetObjectParent(Sensor->nSensor,vLinks.at(parentLinkIndex)->nLinkCollision,true);
                }
            }
        }
}



int robot::getJointPosition(std::string jointName)
{
    for(size_t i = 0; i < vJoints.size() ; i++)
    {
        if(vJoints.at(i)->name == jointName){return i;}
    }
    std::string txt("there is no joint with name '"+ jointName+"'");
    printToConsole(sim_verbosity_scripterrors,txt.c_str());
    return -1;
}
int robot::getLinkPosition(std::string linkName)
{
    for(size_t i = 0; i < vLinks.size() ; i++)
    {
        if(vLinks.at(i)->name == linkName){return i;}
    }
    std::string txt("there is no link with name '"+ linkName+"'");
    printToConsole(sim_verbosity_scripterrors,txt.c_str());
    return -1;
}

void robot::setLocalRespondableMaskCummulative_alternate(int objHandle,bool bitSet)
{
    if (simGetObjectType(objHandle)==sim_object_shape_type)
    {
        int p;
        simGetObjectIntParameter(objHandle,sim_shapeintparam_respondable,&p);
        if (p!=0)
        {
            if (bitSet)
                simSetObjectIntParameter(objHandle,sim_shapeintparam_respondable_mask,0xff01);
            else
                simSetObjectIntParameter(objHandle,sim_shapeintparam_respondable_mask,0xff02);
            bitSet=!bitSet;
        }
    }
    int index=0;
    while (true)
    {
        int childHandle=simGetObjectChild(objHandle,index++);
        if (childHandle==-1)
            break;
        setLocalRespondableMaskCummulative_alternate(childHandle,bitSet);
    }
}
