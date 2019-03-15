#include "link.h"
#include "rospackagehelper.h"
    
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
         mesh_scaling[0]=1.0f;
         mesh_scaling[1]=1.0f;
         mesh_scaling[2]=1.0f;

         n = -1;
}

urdfLink::urdfLink()
{
    //Initialize arrays
        //Variables Inertial
         inertial_xyz[0]= 0;            inertial_xyz[1]= 0;             inertial_xyz[2]= 0;
         inertial_rpy[0]= 0;            inertial_rpy[1]= 0;             inertial_rpy[2]= 0;

         mass = 1.0f;
         inertia[8] = 0.0f; inertia[7] = 0; inertia[6] = 0; inertia[5] = 0; inertia[4] = 0.0f; inertia[3] = 0; inertia[2] = 0; inertia[1] = 0; inertia[0] = 0.0f;
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
        stringToArray(currentVisual().box_size, gazebo_size);
    }
    if(choose == "collision")
    {
        stringToArray(currentCollision().box_size, gazebo_size);
    }
}
void urdfLink::setSphere(std::string gazebo_radius,std::string choose)
{
    if(choose == "visual")
    {
        stringToArray(currentVisual().sphere_size,gazebo_radius+" "+gazebo_radius+" "+gazebo_radius);

        urdfVisualOrCollision &visual = currentVisual();
        visual.sphere_size[0] = visual.sphere_size[0] * 2; //Radius to bounding box conversion
        visual.sphere_size[1] = visual.sphere_size[1] * 2; //Radius to bounding box conversion
        visual.sphere_size[2] = visual.sphere_size[2] * 2; //Radius to bounding box conversion

    }
    if(choose == "collision")
    {
        stringToArray(currentCollision().sphere_size,gazebo_radius+" "+gazebo_radius+" "+gazebo_radius);
        
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
        stringToArray(currentVisual().cylinder_size,gazebo_radius+" "+gazebo_radius+" "+gazebo_length);
        urdfVisualOrCollision &visual = currentVisual();

        visual.cylinder_size[0] = visual.cylinder_size[0] * 2; //Radius to bounding box conversion
        visual.cylinder_size[1] = visual.cylinder_size[1] * 2; //Radius to bounding box conversion
    
    }
    if(choose == "collision")
    {
        stringToArray(currentCollision().cylinder_size,gazebo_radius+" "+gazebo_radius+" "+gazebo_length);
        urdfVisualOrCollision &collision = currentCollision();

        collision.cylinder_size[0] = collision.cylinder_size[0] * 2; //Radius to bounding box conversion
        collision.cylinder_size[1] = collision.cylinder_size[1] * 2; //Radius to bounding box conversion
        
    }
}

void urdfLink::setColor(std::string color)
{
    stringToArray(currentVisual().rgba,color);
    currentVisual().hasColor=true;
}

void urdfLink::setMass(std::string gazebo_mass)
{
    float m=getFloat(gazebo_mass);
    if (m>0.0f)
        mass=m;
}
void urdfLink::setInertia(int position, std::string gazebo_inertia_number)
{
    inertia[position] = getFloat(gazebo_inertia_number);
}
void urdfLink::verifyInertia()
{
    float c=0.0f;
    for (int i=0;i<9;i++)
        c+=fabs(inertia[i]);
    if (c==0.0f)
    {
        std::string txt("ERROR: found an invalid inertia entry for link '"+ name+"'");
        printToConsole(txt.c_str());

        inertia[0]=0.001f;
        inertia[4]=0.001f;
        inertia[8]=0.001f;
    }
}

void urdfLink::setMeshFilename(std::string packagePath,std::string meshFilename,std::string choose)
{
    std::string meshFilename_alt; // we use an alternative filename... the package location is somewhat strangely defined sometimes!!
#ifndef WIN_VREP
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
            printToConsole("Could not decode package name from the mesh file specification.");
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
        std::string txt("ERROR: the extension '"+ extension +"' is not currently a supported.");
        printToConsole(txt.c_str());
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
void urdfLink::createLink(bool hideCollisionLinks,bool convexDecomposeNonConvexCollidables,bool createVisualIfNone,bool& showConvexDecompositionDlg)
{
    std::string txt("Creating link '"+name+"'...");
    printToConsole(txt.c_str());

    // Visuals
    std::vector<urdfVisualOrCollision>::iterator it;
    for (it=visuals.begin(); it!=visuals.end(); it++) {
        urdfVisualOrCollision &visual = *it;
        
        if(!visual.meshFilename.empty())
        {
            std::string fname(visual.meshFilename);
            bool exists=true;
            bool useAlt=false;
            if (!simDoesFileExist(fname.c_str()))
            {
                fname=visual.meshFilename_alt;
                exists=simDoesFileExist(fname.c_str());
                useAlt=true;
            }

            if (!exists)
                if (!useAlt)
                    printToConsole(("ERROR: mesh file '"+visual.meshFilename+"' does not exist.").c_str());
                else
                    printToConsole(("ERROR: neither mesh file '"+visual.meshFilename+"' nor '"+visual.meshFilename_alt+"' do exist.").c_str());
            else {
                printToConsole("Importing");
                printToConsole(fname.c_str());
                try {
                    visual.n = simImportShape(visual.meshExtension,fname.c_str(),16+128,0.0001f,1.0f);
                } catch (std::exception& e) {
                    printToConsole(e.what());
                } catch (...) {
                    printToConsole("Exception caught while importing the mesh file.");
                }
            }

            if (!visual.n)
            {
                if (!useAlt)
                    txt="ERROR: failed to create the mesh '"+visual.meshFilename+"' with extension type "+boost::lexical_cast<std::string>(visual.meshExtension);
                else
                    txt="ERROR: failed to create the mesh '"+visual.meshFilename+"' or '"+visual.meshFilename_alt+"' with extension type "+boost::lexical_cast<std::string>(visual.meshExtension);
                printToConsole(txt.c_str());
            }
            else
                visual.n = scaleShapeIfRequired(visual.n,visual.mesh_scaling);
        }
        else if (!isArrayEmpty(visual.sphere_size))
            visual.n = simCreatePureShape( 1,1+2+16, visual.sphere_size, mass, NULL);
        else if (!isArrayEmpty(visual.cylinder_size))
            visual.n = simCreatePureShape( 2,1+2+16, visual.cylinder_size, mass, NULL);
        else if (!isArrayEmpty(visual.box_size))
            visual.n = simCreatePureShape( 0,1+2+16, visual.box_size, mass, NULL);
    }

    //collisions
    for (it=collisions.begin(); it!=collisions.end(); it++) {
        urdfVisualOrCollision &collision = *it;

        if(!collision.meshFilename.empty())
        {
            std::string fname(collision.meshFilename);
            bool exists=true;
            bool useAlt=false;
            if (!simDoesFileExist(fname.c_str()))
            {
                fname=collision.meshFilename_alt;
                exists=simDoesFileExist(fname.c_str());
                useAlt=true;
            }

            if (!exists)
                if (!useAlt)
                    printToConsole(("ERROR: mesh file '"+collision.meshFilename+"' does not exist.").c_str());
                else
                    printToConsole(("ERROR: neither mesh file '"+collision.meshFilename+"' nor '"+collision.meshFilename_alt+"' do exist.").c_str());
            else
                collision.n = simImportShape(collision.meshExtension,fname.c_str(),16+128,0.0001f,1.0);

            if (collision.n == -1)
            {
                if (!useAlt)
                    txt="ERROR: failed to create the mesh '"+collision.meshFilename+"' with extension type "+boost::lexical_cast<std::string>(collision.meshExtension);
                else
                    txt="ERROR: failed to create the mesh '"+collision.meshFilename+"' or '"+collision.meshFilename_alt+"' with extension type "+boost::lexical_cast<std::string>(collision.meshExtension);
                printToConsole(txt.c_str());
            }
            else
            {
                collision.n=scaleShapeIfRequired(collision.n,collision.mesh_scaling);
                int p;
                int convInts[5]={1,500,200,0,0}; // 3rd value from 100 to 500 on 5/2/2014
                float convFloats[5]={100.0f,30.0f,0.25f,0.0f,0.0f};
                if ( convexDecomposeNonConvexCollidables&&(simGetObjectIntParameter(collision.n,3017,&p)>0)&&(p==0) )
                {
                    int aux=1+4+8+16+64;
                    if (showConvexDecompositionDlg)
                        aux=1+2+8+16+64;
                    showConvexDecompositionDlg=false;
                    simConvexDecompose(collision.n,aux,convInts,convFloats); // we generate convex shapes!
                }
                simSetObjectIntParameter(collision.n,3003,!inertiaPresent); // we make it non-static if there is an inertia
                simSetObjectIntParameter(collision.n,3004,1); // we make it respondable since it is a collision object
            }

        }
        else if (!isArrayEmpty(collision.sphere_size))
            collision.n = simCreatePureShape( 1,1+2+4+8+16*(!inertiaPresent), collision.sphere_size, mass, NULL);
        else if (!isArrayEmpty(collision.cylinder_size))
            collision.n = simCreatePureShape( 2,1+2+4+8+16*(!inertiaPresent), collision.cylinder_size, mass, NULL);
        else if (!isArrayEmpty(collision.box_size))
            collision.n = simCreatePureShape( 0,1+2+4+8+16*(!inertiaPresent), collision.box_size, mass, NULL);

        // Set the respondable mask:
        simSetObjectIntParameter(collision.n,3019,0xff00); // colliding with everything except with other objects in that tree hierarchy
    }

    if (createVisualIfNone&&(visuals.size()==0))
    {
        if (collisions.size() > 0)
        { // We create a visual from the collision shapes
            for (it=collisions.begin(); it!=collisions.end(); it++) {
                urdfVisualOrCollision &collision = *it;
                simRemoveObjectFromSelection(sim_handle_all,-1);
                simAddObjectToSelection(sim_handle_single,collision.n);
                simCopyPasteSelectedObjects();
                addVisual();
                currentVisual().n = simGetObjectLastSelection();
            }
        } else
        { // This is an empty link (no visual and no collision); create a dummy visual
            float dummySize[3]={0.0005f,0.0005f,0.0005f};
            addVisual();
            currentVisual().n = simCreatePureShape( 0,1+16, dummySize, 0.00001f, NULL);
        }
    }

    if (inertiaPresent && (collisions.size()==0))
    {
        // we do not have a collision object. Let's create a dummy collision object, since inertias can't exist on their own in V-REP:
        float dummySize[3]={0.05f,0.05f,0.05f};
        addCollision();
        currentCollision().n = simCreatePureShape( 1,1+2+4, dummySize, mass, NULL); // we make it non-respondable!
    }

    // Grouping visuals
    const float specular[3]={0.2f,0.2f,0.2f};
    simInt *shapes = new simInt[visuals.size()];
    int validShapes = 0;
    for (unsigned int i=0; i<visuals.size(); i++) {
        urdfVisualOrCollision &visual = visuals[i];
        if (visual.n!=-1) {
            if (visual.hasColor)
            {
                simSetShapeColor(visual.n,NULL,sim_colorcomponent_ambient_diffuse,visual.rgba);
                simSetShapeColor(visual.n,NULL,sim_colorcomponent_specular,specular);
            }

            C7Vector frame;
            frame.X.set(visual.xyz);
            frame.Q=getQuaternionFromRpy(visual.rpy);

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
    std::cout << std::flush;
    if (validShapes > 1) {
        nLinkVisual = simGroupShapes(shapes, validShapes);
    } else if (validShapes == 1) {
        nLinkVisual = shapes[0];
    }

    // Grouping collisions
    shapes = new simInt[collisions.size()];
    validShapes = 0;
    for (unsigned int i=0; i<collisions.size(); i++) {
        urdfVisualOrCollision &collision = collisions[i];
        if (collision.n!=-1) {
            C7Vector frame;
            frame.X.set(collision.xyz);
            frame.Q=getQuaternionFromRpy(collision.rpy);

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
    std::cout << std::flush;
    if (validShapes > 1) {
        nLinkCollision = simGroupShapes(shapes, validShapes);
    } else if (validShapes == 1) {
        nLinkCollision = shapes[0];
    }

    // Inertia
    if (inertiaPresent)
    {
        C7Vector inertiaFrame;
        inertiaFrame.X.set(inertial_xyz);
        inertiaFrame.Q=getQuaternionFromRpy(inertial_rpy);

        C7Vector collisionFrame;
        //collisionFrame.X.set(collision_xyz);
        //collisionFrame.Q=getQuaternionFromRpy(collision_rpy);

        //C4X4Matrix x((collisionFrame.getInverse()*inertiaFrame).getMatrix());
        C4X4Matrix x(inertiaFrame.getMatrix());
        float i[12]={x.M(0,0),x.M(0,1),x.M(0,2),x.X(0),x.M(1,0),x.M(1,1),x.M(1,2),x.X(1),x.M(2,0),x.M(2,1),x.M(2,2),x.X(2)};
        simSetShapeMassAndInertia(nLinkCollision,mass,inertia,C3Vector::zeroVector.data,i);
        //std::cout << "Mass: " << mass << std::endl;
    }
    else
    {
        if (collisions.size() > 0)
        {
            std::string txt("ERROR: found a collision object without inertia data for link '"+ name+"'. Is that link meant to be static?");
            printToConsole(txt.c_str());
        }
    }
    
    // Set the names, visibility, etc.:
    if (nLinkVisual!=-1)
    {
        setVrepObjectName(nLinkVisual,std::string(name+"_visual").c_str());
        //const float specularDiffuse[3]={0.3f,0.3f,0.3f};
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
        setVrepObjectName(nLinkCollision,std::string(name+"_respondable").c_str());
        if (hideCollisionLinks)
            simSetObjectIntParameter(nLinkCollision,10,256); // we "hide" that object in layer 9
    }
}

int urdfLink::scaleShapeIfRequired(int shapeHandle,float scalingFactors[3])
{ // in future there will be a non-iso scaling function for objects in V-REP, but until then...
    if ( (scalingFactors[0]*scalingFactors[1]*scalingFactors[2]>0.99999f)&&(scalingFactors[0]>0.0f)&&(scalingFactors[1]>0.0f) )
        return(shapeHandle); // no scaling required!
    if (fabs(scalingFactors[0])<0.00001f)
        scalingFactors[0]=0.00001f*scalingFactors[0]/fabs(scalingFactors[0]);
    if (fabs(scalingFactors[1])<0.00001f)
        scalingFactors[1]=0.00001f*scalingFactors[1]/fabs(scalingFactors[1]);
    if (fabs(scalingFactors[2])<0.00001f)
        scalingFactors[2]=0.00001f*scalingFactors[2]/fabs(scalingFactors[2]);
    int newShapeHandle=shapeHandle;
    float* vertices;
    int verticesSize;
    int* indices;
    int indicesSize;
    if (simGetShapeMesh(shapeHandle,&vertices,&verticesSize,&indices,&indicesSize,NULL)!=-1)
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
        if (scalingFactors[0]*scalingFactors[1]*scalingFactors[2]<0.0f)
        {
            for (int i=0;i<indicesSize/3;i++)
            {
                int tmp=indices[3*i+0];
                indices[3*i+0]=indices[3*i+1];
                indices[3*i+1]=tmp;
            }
        }
        // Remove the old shape and create a new one with the scaled data:
        simRemoveObject(shapeHandle);
        newShapeHandle=simCreateMeshShape(2,20.0f*piValue/180.0f,vertices,verticesSize,indices,indicesSize,NULL);
        simReleaseBuffer((char*)vertices);
        simReleaseBuffer((char*)indices);
    }
    return(newShapeHandle);
}


