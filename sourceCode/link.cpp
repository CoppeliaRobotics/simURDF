#include "link.h"
#include "rospackagehelper.h"
#include <filesystem>

bool isUriPath(const std::string& path)
{
    const std::string filePrefix = "file://";
    if (path.size() >= filePrefix.size() && std::equal(filePrefix.begin(), filePrefix.end(), path.begin(), [](char a, char b) { return tolower(a) == tolower(b); }))
        return true;
    return false;
}

std::string getAbsPath(const char* urdfFile, const char* assetFile)
{
    std::string assetF(assetFile);
    if ((strlen(urdfFile) > 0) && (assetF.find("package://") == std::string::npos))
    {
        bool isAssetUri = isUriPath(assetF);
        if (std::filesystem::path(assetF).is_absolute() || isAssetUri)
        {
            if (isAssetUri)
            {
                size_t prefix_end = assetF.find("://") + 3;
                std::string prefix = assetF.substr(0, prefix_end);
                std::string path_part = assetF.substr(prefix_end);
                std::filesystem::path norm_path = std::filesystem::path(path_part).lexically_normal();
                assetF = prefix + norm_path.string();
            }
        }
        else
        {
            std::filesystem::path basePath = std::filesystem::path(urdfFile);
            std::filesystem::path targetPath(assetF);
            std::filesystem::path baseDirectory = basePath.parent_path();
            std::filesystem::path absolutePath = baseDirectory / targetPath;
            absolutePath = absolutePath.lexically_normal();
            assetF = absolutePath.string();
        }
    }
    return assetF;
}

urdfVisualOrCollision::urdfVisualOrCollision()
{
        //Variables Visual              
         xyz[0]= 0;             xyz[1]= 0;              xyz[2]= 0;
         rpy[0]= 0;             rpy[1]= 0;              rpy[2]= 0;
         box_size[0]= 0;            box_size[1]= 0;         box_size[2]= 0; 
         sphere_size[0]= 0;     sphere_size[1]= 0;      sphere_size[2]= 0;
         cylinder_size[0]= 0;    cylinder_size[1]= 0;       cylinder_size[2]= 0;
         rgba[0]=0.4f;          rgba[1]=0.4f;           rgba[2]=0.4f;   rgba[3]=0.0f;
         hasColor=false;
         mesh_scaling[0]=1.0;
         mesh_scaling[1]=1.0;
         mesh_scaling[2]=1.0;

         n = -1;
}

std::string strip(const char* filename)
{
    std::string f(filename);
    std::string ff(f);
    for (size_t i=0;i<ff.size();i++)
        ff[i]=std::tolower(ff[i]);
    size_t p=ff.find("file://");
    if (p!=std::string::npos)
        f=f.substr(7);
    return f;
}

urdfLink::urdfLink()
{
    //Initialize arrays
        //Variables Inertial
         inertial_xyz[0]= 0;            inertial_xyz[1]= 0;             inertial_xyz[2]= 0;
         inertial_rpy[0]= 0;            inertial_rpy[1]= 0;             inertial_rpy[2]= 0;

         mass = 1.0;
         inertia[8] = 0.0; inertia[7] = 0; inertia[6] = 0; inertia[5] = 0; inertia[4] = 0.0; inertia[3] = 0; inertia[2] = 0; inertia[1] = 0; inertia[0] = 0.0;
        inertiaPresent=false;

        nParent=-1;
        nChild=-1;
        nLinkVisual=-1;
        nLinkCollision=-1;
}

void urdfLink::addVisual()
{
    visuals.push_back(urdfVisualOrCollision());
}
    
urdfVisualOrCollision &urdfLink::currentVisual()
{
    return visuals[visuals.size()-1];
}

void urdfLink::addCollision()
{
    collisions.push_back(urdfVisualOrCollision());
}

urdfVisualOrCollision &urdfLink::currentCollision()
{
    return collisions[collisions.size()-1];
}

urdfLink::~urdfLink()
{
}

void urdfLink::setPosition(std::string gazebo_xyz,std::string choose)
{
    if(choose == "inertial")
        stringToArray(inertial_xyz,gazebo_xyz.c_str());
    if(choose == "visual")
        stringToArray(currentVisual().xyz,gazebo_xyz.c_str());
    if(choose == "collision")
        stringToArray(currentCollision().xyz,gazebo_xyz.c_str());
}

void urdfLink::setRotation(std::string gazebo_rpy,std::string choose)
{
    if(choose == "inertial")
        stringToArray(inertial_rpy,gazebo_rpy.c_str());
    if(choose == "visual")
        stringToArray(currentVisual().rpy,gazebo_rpy.c_str());
    if(choose == "collision")
        stringToArray(currentCollision().rpy,gazebo_rpy.c_str());
}


void urdfLink::setBox(std::string gazebo_size,std::string choose)
{
    if(choose == "visual")
    {
        stringToSizeArray(currentVisual().box_size, gazebo_size);
    }
    if(choose == "collision")
    {
        stringToSizeArray(currentCollision().box_size, gazebo_size);
    }
}
void urdfLink::setSphere(std::string gazebo_radius,std::string choose)
{
    if(choose == "visual")
    {
        stringToSizeArray(currentVisual().sphere_size,gazebo_radius+" "+gazebo_radius+" "+gazebo_radius);

        urdfVisualOrCollision &visual = currentVisual();
        visual.sphere_size[0] = visual.sphere_size[0] * 2; //Radius to bounding box conversion
        visual.sphere_size[1] = visual.sphere_size[1] * 2; //Radius to bounding box conversion
        visual.sphere_size[2] = visual.sphere_size[2] * 2; //Radius to bounding box conversion

    }
    if(choose == "collision")
    {
        stringToSizeArray(currentCollision().sphere_size,gazebo_radius+" "+gazebo_radius+" "+gazebo_radius);
        
        urdfVisualOrCollision &collision = currentCollision();
        collision.sphere_size[0] = collision.sphere_size[0] * 2; //Radius to bounding box conversion
        collision.sphere_size[1] = collision.sphere_size[1] * 2; //Radius to bounding box conversion
        collision.sphere_size[2] = collision.sphere_size[2] * 2; //Radius to bounding box conversion
    }
}
void urdfLink::setCylinder(std::string gazebo_radius,std::string gazebo_length,std::string choose)
{
    if(choose == "visual")
    {
        stringToSizeArray(currentVisual().cylinder_size,gazebo_radius+" "+gazebo_radius+" "+gazebo_length);
        urdfVisualOrCollision &visual = currentVisual();

        visual.cylinder_size[0] = visual.cylinder_size[0] * 2; //Radius to bounding box conversion
        visual.cylinder_size[1] = visual.cylinder_size[1] * 2; //Radius to bounding box conversion
    
    }
    if(choose == "collision")
    {
        stringToSizeArray(currentCollision().cylinder_size,gazebo_radius+" "+gazebo_radius+" "+gazebo_length);
        urdfVisualOrCollision &collision = currentCollision();

        collision.cylinder_size[0] = collision.cylinder_size[0] * 2; //Radius to bounding box conversion
        collision.cylinder_size[1] = collision.cylinder_size[1] * 2; //Radius to bounding box conversion
        
    }
}

void urdfLink::setColor(std::string color)
{
    stringToFArray(currentVisual().rgba,color);
    currentVisual().hasColor=true;
}

void urdfLink::setMass(std::string gazebo_mass)
{
    double m=getFloat(gazebo_mass);
    if (m>0.0)
        mass=m;
}
void urdfLink::setInertia(int position, std::string gazebo_inertia_number)
{
    inertia[position] = getFloat(gazebo_inertia_number);
}
void urdfLink::verifyInertia()
{
    double c=0.0;
    for (int i=0;i<9;i++)
        c+=fabs(inertia[i]);
    if (c==0.0)
    {
        std::string txt("found an invalid inertia entry for link '"+ name+"'");
        printToConsole(sim_verbosity_scripterrors,txt.c_str());

        inertia[0]=0.001;
        inertia[4]=0.001;
        inertia[8]=0.001;
    }
}

void urdfLink::setMeshFilename(std::string packagePath,std::string meshFilename,std::string choose,const char* packageReplaceStr,const char* urdfFile)
{
    if (strlen(packageReplaceStr)>0)
        meshFilename.replace(meshFilename.find("package://"),strlen("package://"),packageReplaceStr);
    meshFilename = getAbsPath(urdfFile, meshFilename.c_str());
    std::string meshFilename_alt; // we use an alternative filename... the package location is somewhat strangely defined sometimes!!
#ifndef WIN_SIM
    if (meshFilename.compare(0,10,"package://")==0) // condition added by Marc on 17/1/2014
    {
        meshFilename = meshFilename.substr(10,meshFilename.size()); //to delete de package:// part
        size_t packageNameEndPos = meshFilename.find("/");
        if (packageNameEndPos != std::string::npos && packageNameEndPos > 0)
        {
            // extract the package name
            std::string packageName = meshFilename.substr(0, packageNameEndPos);

            // get the root dir the packageName is relative to (can be different for different packages,
            // e.g. in multi-workspace setups)
            std::string packageRootDir = rosPackageHelper::getPackageRootDir(packageName);

            if (packageRootDir.length() > 0)
            {
                meshFilename_alt = packageRootDir + std::string("/../") + meshFilename;
                meshFilename = packageRootDir + std::string("/") + meshFilename;
            }
        }
        else
        {
            printToConsole(sim_verbosity_scripterrors,"could not decode package name from the mesh file specification.");
        }
    }
#else
    if (meshFilename.compare(0,10,"package://")==0)
    {
        meshFilename = meshFilename.substr(9,meshFilename.size()); //to delete de package:/ part
        meshFilename_alt=meshFilename;
        meshFilename = packagePath + meshFilename;
        packagePath = packagePath.substr(0, packagePath.find_last_of("/"));
        meshFilename_alt = packagePath + meshFilename_alt;
    }
#endif

    std::string extension = meshFilename.substr(meshFilename.size()-3,meshFilename.size());
    int nExtension;
    if(extension == "obj" || extension =="OBJ"){ nExtension = 0;}
    else if(extension == "dxf" || extension == "DXF"){ nExtension = 1;}
    else if(extension == "3ds" || extension == "3DS"){ nExtension = 2;}
    else if(extension == "stl" || extension == "STL"){ nExtension = 4;}
    else if(extension == "dae" || extension == "DAE"){ nExtension = 5;}
    else
    {
        nExtension = -1;
        std::string txt("the extension '"+ extension +"' is not currently a supported.");
        printToConsole(sim_verbosity_scripterrors,txt.c_str());
    }

    if(choose == "visual")
    {
        urdfVisualOrCollision &visual = currentVisual();
        visual.meshFilename = meshFilename;
        visual.meshFilename_alt = meshFilename_alt;
        visual.meshExtension = nExtension;
    }
    if(choose == "collision")
    {
        urdfVisualOrCollision &collision = currentCollision();
        collision.meshFilename = meshFilename;
        collision.meshFilename_alt = meshFilename_alt;
        collision.meshExtension = nExtension;
    }
}

//Write
void urdfLink::createLink(int options)
{
    bool hideCollisionLinks=(options&1)==0;
    bool convexDecomposeNonConvexCollidables=(options&4)!=0;
    bool createVisualIfNone=(options&8)!=0;
    bool convexHull=(options&512)!=0;
    bool shapeAtJointLoc=(options&1024)!=0;

    std::string txt("creating link '"+name+"'...");
    printToConsole(sim_verbosity_scripterrors,txt.c_str());

    // Visuals
    std::vector<urdfVisualOrCollision>::iterator it;
    for (it=visuals.begin(); it!=visuals.end(); it++)
    {
        urdfVisualOrCollision &visual = *it;
        
        if(!visual.meshFilename.empty())
        {
            std::string fname(strip(visual.meshFilename.c_str()));
            bool exists=true;
            bool useAlt=false;
            if (!simDoesFileExist(fname.c_str()))
            {
                fname=strip(visual.meshFilename_alt.c_str());
                exists=simDoesFileExist(fname.c_str());
                useAlt=true;
            }

            if (!exists)
            {
                if (!useAlt)
                    printToConsole(sim_verbosity_scripterrors,("mesh file '"+visual.meshFilename+"' does not exist.").c_str());
                else
                    printToConsole(sim_verbosity_scripterrors,("neither mesh file '"+visual.meshFilename+"' nor '"+visual.meshFilename_alt+"' do exist.").c_str());
            }
            else {
                printToConsole(sim_verbosity_scripterrors,("importing "+fname).c_str());
                try {
                    visual.n = simImportShape(visual.meshExtension,fname.c_str(),16+128,0.0001,1.0);
                } catch (std::exception& e) {
                    printToConsole(sim_verbosity_scripterrors,e.what());
                } catch (...) {
                    printToConsole(sim_verbosity_scripterrors,"exception caught while importing the mesh file.");
                }
            }

            if (!visual.n)
            {
                if (!useAlt)
                    txt="failed to create the mesh '"+visual.meshFilename+"' with extension type "+boost::lexical_cast<std::string>(visual.meshExtension);
                else
                    txt="failed to create the mesh '"+visual.meshFilename+"' or '"+visual.meshFilename_alt+"' with extension type "+boost::lexical_cast<std::string>(visual.meshExtension);
                printToConsole(sim_verbosity_scripterrors,txt.c_str());
            }
            else
                visual.n = scaleShapeIfRequired(visual.n,visual.mesh_scaling);
        }
        else if (!isArrayEmpty(visual.sphere_size))
        {
            visual.n = simCreatePrimitiveShape(sim_primitiveshape_spheroid,visual.sphere_size,1);
            simSetShapeMass(visual.n,mass);
        }
        else if (!isArrayEmpty(visual.cylinder_size))
        {
            visual.n = simCreatePrimitiveShape(sim_primitiveshape_cylinder,visual.cylinder_size,1);
            simSetShapeMass(visual.n,mass);
        }
        else if (!isArrayEmpty(visual.box_size))
        {
            visual.n = simCreatePrimitiveShape(sim_primitiveshape_cuboid,visual.box_size,1);
            simSetShapeMass(visual.n,mass);
        }
    }

    //collisions
    for (it=collisions.begin(); it!=collisions.end(); it++)
    {
        urdfVisualOrCollision &collision = *it;

        if(!collision.meshFilename.empty())
        {
            std::string fname(strip(collision.meshFilename.c_str()));
            bool exists=true;
            bool useAlt=false;
            if (!simDoesFileExist(fname.c_str()))
            {
                fname=strip(collision.meshFilename_alt.c_str());
                exists=simDoesFileExist(fname.c_str());
                useAlt=true;
            }

            if (!exists)
                if (!useAlt)
                    printToConsole(sim_verbosity_scripterrors,("mesh file '"+collision.meshFilename+"' does not exist.").c_str());
                else
                    printToConsole(sim_verbosity_scripterrors,("neither mesh file '"+collision.meshFilename+"' nor '"+collision.meshFilename_alt+"' do exist.").c_str());
            else
                collision.n = simImportShape(collision.meshExtension,fname.c_str(),16+128,0.0001,1.0);

            if (collision.n == -1)
            {
                if (!useAlt)
                    txt="failed to create the mesh '"+collision.meshFilename+"' with extension type "+boost::lexical_cast<std::string>(collision.meshExtension);
                else
                    txt="failed to create the mesh '"+collision.meshFilename+"' or '"+collision.meshFilename_alt+"' with extension type "+boost::lexical_cast<std::string>(collision.meshExtension);
                printToConsole(sim_verbosity_scripterrors,txt.c_str());
            }
            else
            {
                collision.n=scaleShapeIfRequired(collision.n,collision.mesh_scaling);

                int p;
                simGetObjectInt32Param(collision.n,sim_shapeintparam_convex,&p);
                if (p==0)
                { // not convex
                    if (convexHull)
                    { // has precedence
                        double* vertices;
                        int verticesL;
                        int* indices;
                        int indicesL;
                        simGetShapeMesh(collision.n,&vertices,&verticesL,&indices,&indicesL,nullptr);
                        double tr[12];
                        simGetObjectMatrix(collision.n,-1,tr);
                        for (int i=0;i<verticesL/3;i++)
                            simTransformVector(tr,vertices+3*i);
                        double* vertices2;
                        int vertices2L;
                        int* indices2;
                        int indices2L;
                        simGetQHull(vertices,verticesL,&vertices2,&vertices2L,&indices2,&indices2L,0,nullptr);
                        simReleaseBuffer((char*)vertices);
                        simReleaseBuffer((char*)indices);
                        int h=simCreateMeshShape(0,0.0,vertices2,vertices2L,indices2,indices2L,nullptr);
                        simReleaseBuffer((char*)vertices2);
                        simReleaseBuffer((char*)indices2);
                        simRemoveObjects(&collision.n,1);
                        collision.n=h;
                    }
                    else
                    {
                        if (convexDecomposeNonConvexCollidables)
                        {
                            int convInts[5]={1,500,200,0,0}; // 3rd value from 100 to 500 on 5/2/2014
                            double convFloats[5]={100.0,30.0,0.25,0.0,0.0};
                            simConvexDecompose(collision.n,1+4+8+16+64,convInts,convFloats); // we generate convex shapes!
                        }
                    }
                }
                simSetObjectInt32Param(collision.n,3003,!inertiaPresent); // we make it non-static if there is an inertia
                simSetObjectInt32Param(collision.n,3004,1); // we make it respondable since it is a collision object
            }

        }
        else if (!isArrayEmpty(collision.sphere_size))
        {
            collision.n = simCreatePrimitiveShape(sim_primitiveshape_spheroid,collision.sphere_size,1);
            simSetObjectInt32Param(collision.n,sim_shapeintparam_respondable,1);
            if (inertiaPresent)
                simSetObjectInt32Param(collision.n,sim_shapeintparam_static,0);
            simSetShapeMass(collision.n,mass);
        }
        else if (!isArrayEmpty(collision.cylinder_size))
        {
            collision.n = simCreatePrimitiveShape(sim_primitiveshape_cylinder,collision.cylinder_size,1);
            simSetObjectInt32Param(collision.n,sim_shapeintparam_respondable,1);
            if (inertiaPresent)
                simSetObjectInt32Param(collision.n,sim_shapeintparam_static,0);
            simSetShapeMass(collision.n,mass);
        }
        else if (!isArrayEmpty(collision.box_size))
        {
            collision.n = simCreatePrimitiveShape(sim_primitiveshape_cuboid,collision.box_size,1);
            simSetObjectInt32Param(collision.n,sim_shapeintparam_respondable,1);
            if (inertiaPresent)
                simSetObjectInt32Param(collision.n,sim_shapeintparam_static,0);
            simSetShapeMass(collision.n,mass);
        }

        // Set the respondable mask:
        simSetObjectInt32Param(collision.n,3019,0xff00); // colliding with everything except with other objects in that tree hierarchy
    }

    if (createVisualIfNone&&(visuals.size()==0))
    {
        if (collisions.size() > 0)
        { // We create a visual from the collision shapes
            for (it=collisions.begin(); it!=collisions.end(); it++) {
                urdfVisualOrCollision &collision = *it;
                int newShape=collision.n;
                simCopyPasteObjects(&newShape,1,2+4+8+32);
                simSetObjectInt32Param(newShape,sim_shapeintparam_respondable,0);
                simSetObjectInt32Param(newShape,sim_shapeintparam_static,0);
                addVisual();
                currentVisual().n = newShape;
            }
        }
        else
        { // This is an empty link (no visual and no collision); create a dummy visual
            double dummySize[3]={0.0005,0.0005,0.0005};
            addVisual();
            currentVisual().n = simCreatePrimitiveShape(sim_primitiveshape_cuboid,dummySize,1);
            simSetShapeMass(currentVisual().n,0.00001);
        }
    }

    if (inertiaPresent && (collisions.size()==0))
    {
        // we do not have a collision object. Let's create a dummy collision object, since inertias can't exist on their own in CoppeliaSim:
        double dummySize[3]={0.05,0.05,0.05};
        addCollision();
        currentCollision().n = simCreatePrimitiveShape(sim_primitiveshape_spheroid,dummySize,1);
        simSetShapeMass(currentCollision().n,mass);
    }

    // Grouping visuals
    const float specular[3]={0.2f,0.2f,0.2f};
    int *shapes = new int[visuals.size()];
    int validShapes = 0;
    C7Vector desiredShapeFramePose;
    for (unsigned int i=0; i<visuals.size(); i++)
    {
        urdfVisualOrCollision &visual = visuals[i];
        if (visual.n!=-1)
        {
            if (visual.hasColor)
            {
                simSetShapeColor(visual.n,nullptr,sim_colorcomponent_ambient_diffuse,visual.rgba);
                simSetShapeColor(visual.n,nullptr,sim_colorcomponent_specular,specular);
            }

            C7Vector frame;
            frame.X.setData(visual.xyz);
            frame.Q=getQuaternionFromRpy(visual.rpy);

            if (validShapes==0)
                desiredShapeFramePose=frame; // pick the first visual item

            C7Vector initVisualFrame;
            simGetObjectPosition(visual.n,-1,initVisualFrame.X.data);
            C3Vector euler;
            simGetObjectOrientation(visual.n,-1,euler.data);
            initVisualFrame.Q.setEulerAngles(euler);

            C7Vector x(frame*initVisualFrame);

            simSetObjectPosition(visual.n,-1,x.X.data);
            simSetObjectOrientation(visual.n,-1,x.Q.getEulerAngles().data);

            shapes[validShapes++] = visual.n;
        }
    }
    //std::cout << std::flush;
    if (validShapes > 1)
        nLinkVisual = simGroupShapes(shapes,validShapes);
    else if (validShapes == 1)
        nLinkVisual = shapes[0];
    if (validShapes>0)
    {
        if (shapeAtJointLoc)
        { // try to place the origin of the shape at the joint's location
            double identity[7]={0.0,0.0,0.0,0.0,0.0,0.0,1.0};
            simRelocateShapeFrame(nLinkVisual,identity);
        }
        else
        {
            double p[7];
            desiredShapeFramePose.getData(p,true);
            simRelocateShapeFrame(nLinkVisual,p);
        }
        simAlignShapeBB(nLinkVisual,nullptr);
    }

    // Grouping collisions
    shapes = new int[collisions.size()];
    validShapes = 0;
    for (unsigned int i=0; i<collisions.size(); i++) {
        urdfVisualOrCollision &collision = collisions[i];
        if (collision.n!=-1) {
            C7Vector frame;
            frame.X.setData(collision.xyz);
            frame.Q=getQuaternionFromRpy(collision.rpy);

            if (validShapes==0)
                desiredShapeFramePose=frame; // pick the first collision item

            C7Vector initCollisionFrame;
            simGetObjectPosition(collision.n,-1,initCollisionFrame.X.data);
            C3Vector euler;
            simGetObjectOrientation(collision.n,-1,euler.data);
            initCollisionFrame.Q.setEulerAngles(euler);

            C7Vector x(frame*initCollisionFrame);

            simSetObjectPosition(collision.n,-1,x.X.data);
            simSetObjectOrientation(collision.n,-1,x.Q.getEulerAngles().data);

            shapes[validShapes++] = collision.n;
        }
    }
    //std::cout << std::flush;
    if (validShapes > 1)
        nLinkCollision = simGroupShapes(shapes, validShapes);
    else if (validShapes == 1)
        nLinkCollision = shapes[0];
    if (validShapes>0)
    {
        if (shapeAtJointLoc)
        { // try to place the origin of the shape at the joint's location
            double identity[7]={0.0,0.0,0.0,0.0,0.0,0.0,1.0};
            simRelocateShapeFrame(nLinkCollision,identity);
        }
        else
        {
            double p[7];
            desiredShapeFramePose.getData(p,true);
            simRelocateShapeFrame(nLinkCollision,p);
        }
        simAlignShapeBB(nLinkCollision,nullptr);
    }


    // Inertia
    if (inertiaPresent)
    {
        C7Vector inertiaFrame;
        inertiaFrame.X.setData(inertial_xyz);
        inertiaFrame.Q=getQuaternionFromRpy(inertial_rpy);

        double _m[12];
        simGetObjectMatrix(nLinkCollision,-1,_m); // nLinkCollision has not yet any parent/children
        C4X4Matrix m;
        m.setData(_m);
        C4X4Matrix x(m.getInverse()*inertiaFrame.getMatrix());
        double i[12]={x.M(0,0),x.M(0,1),x.M(0,2),x.X(0),x.M(1,0),x.M(1,1),x.M(1,2),x.X(1),x.M(2,0),x.M(2,1),x.M(2,2),x.X(2)};
        simSetShapeMass(nLinkCollision,mass);
        simSetShapeInertia(nLinkCollision,inertia,i);
    }
    else
    {
        if (collisions.size() > 0)
        {
            std::string txt("found a collision object without inertia data for link '"+ name+"'. Is that link meant to be static?");
            printToConsole(sim_verbosity_scripterrors,txt.c_str());
        }
    }
    
    // Set the names, visibility, etc.:
    if (nLinkVisual!=-1)
    {
        setSimObjectName(nLinkVisual,std::string(name+"_visual").c_str());
        //const double specularDiffuse[3]={0.3,0.3,0.3};
        if (nLinkCollision!=-1)
        { // if we have a collision object, we attach the visual object to it, then forget the visual object

            C3Vector euler;
            C7Vector collisionFrame;
            simGetObjectPosition(nLinkCollision,-1,collisionFrame.X.data);
            simGetObjectOrientation(nLinkCollision,-1,euler.data);
            collisionFrame.Q.setEulerAngles(euler);

            C7Vector visualFrame;
            simGetObjectPosition(nLinkVisual,-1,visualFrame.X.data);
            simGetObjectOrientation(nLinkVisual,-1,euler.data);
            visualFrame.Q.setEulerAngles(euler);

            C7Vector x(collisionFrame.getInverse()*visualFrame);

            simSetObjectPosition(nLinkVisual,-1,x.X.data);
            simSetObjectOrientation(nLinkVisual,-1,x.Q.getEulerAngles().data);
            simSetObjectParent(nLinkVisual,nLinkCollision,0);
        }
    }
    if (nLinkCollision!=-1)
    {
        setSimObjectName(nLinkCollision,std::string(name+"_respondable").c_str());
        if (hideCollisionLinks)
            simSetObjectInt32Param(nLinkCollision,10,256); // we "hide" that object in layer 9
    }
}

int urdfLink::scaleShapeIfRequired(int shapeHandle,double scalingFactors[3])
{ // in future there will be a non-iso scaling function for objects in CoppeliaSim, but until then...
    if ( (scalingFactors[0]*scalingFactors[1]*scalingFactors[2]>0.99999)&&(scalingFactors[0]>0.0)&&(scalingFactors[1]>0.0) )
        return(shapeHandle); // no scaling required!
    if (fabs(scalingFactors[0])<0.00001)
        scalingFactors[0]=0.00001*scalingFactors[0]/fabs(scalingFactors[0]);
    if (fabs(scalingFactors[1])<0.00001)
        scalingFactors[1]=0.00001*scalingFactors[1]/fabs(scalingFactors[1]);
    if (fabs(scalingFactors[2])<0.00001)
        scalingFactors[2]=0.00001*scalingFactors[2]/fabs(scalingFactors[2]);
    int newShapeHandle=shapeHandle;
    double* vertices;
    int verticesSize;
    int* indices;
    int indicesSize;
    if (simGetShapeMesh(shapeHandle,&vertices,&verticesSize,&indices,&indicesSize,nullptr)!=-1)
    {
        // Scale the vertices:
        C7Vector tr;
        simGetObjectPosition(shapeHandle,-1,tr.X.data);
        C3Vector euler;
        simGetObjectOrientation(shapeHandle,-1,euler.data);
        tr.Q.setEulerAngles(euler);
        for (int i=0;i<verticesSize/3;i++)
        {
            C3Vector v(vertices+3*i);
            v*=tr;
            v(0)*=scalingFactors[0];
            v(1)*=scalingFactors[1];
            v(2)*=scalingFactors[2];
            vertices[3*i+0]=v(0);
            vertices[3*i+1]=v(1);
            vertices[3*i+2]=v(2);
        }
        // Flip the triangles (if needed)
        if (scalingFactors[0]*scalingFactors[1]*scalingFactors[2]<0.0)
        {
            for (int i=0;i<indicesSize/3;i++)
            {
                int tmp=indices[3*i+0];
                indices[3*i+0]=indices[3*i+1];
                indices[3*i+1]=tmp;
            }
        }
        // Remove the old shape and create a new one with the scaled data:
        simRemoveObjects(&shapeHandle,1);
        newShapeHandle=simCreateMeshShape(0,20.0*piValue/180.0,vertices,verticesSize,indices,indicesSize,nullptr);
        simReleaseBuffer((char*)vertices);
        simReleaseBuffer((char*)indices);
    }
    return(newShapeHandle);
}


