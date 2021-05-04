local simURDF={}

function simURDF.export(modelHandle,fileName,outputMode,exportFuncs)
    assert(sim.isHandle(modelHandle),'not a valid handle')
    assert(fileName,'filename not specified')
    local baseName=fileName
    if fileName:find('%.urdf$') then
        baseName=fileName:sub(1, -6)
    else
        fileName=fileName..'.urdf'
    end
    outputMode=outputMode or 'file'
    exportFuncs=exportFuncs or {}

    exportFuncs.newNode=exportFuncs.newNode or function(t)
        assert(type(t)=='table','bad type')
        local name=table.remove(t,1)
        t[0]=name
        return t
    end

    exportFuncs.toXML=exportFuncs.toXML or function(exportFuncs,node,level)
        level=level or 0
        local indent=''; for i=1,level do indent=indent..'    ' end
        local xml=''
        if level==0 then xml=xml..'<?xml version="1.0"?>\n' end
        xml=xml..indent..'<'..node[0]
        for k,v in pairs(node) do
            if type(k)~='number' then
                for c,r in pairs{['"']='&quot;',['<']='&lt;',['>']='&gt;'} do v=string.gsub(v,c,r) end
                xml=xml..' '..k..'="'..v..'"'
            end
        end
        if #node>0 then
            xml=xml..'>\n'
            for i,child in ipairs(node) do
                xml=xml..exportFuncs.toXML(exportFuncs,child,level+1)
            end
            xml=xml..indent..'</'..node[0]..'>\n'
        else
            xml=xml..' />\n'
        end
        return xml
    end

    exportFuncs.getShapeGeometryNode=exportFuncs.getShapeGeometryNode or function(exportFuncs,shapeHandle,baseName)
        local geometryNode=exportFuncs.newNode{'geometry'}
        local r,pureType,dims=sim.getShapeGeomInfo(shapeHandle)
        local pure=(r&2)>0
        local x,y,z=dims[1]*dims[4],dims[2]*dims[4],dims[3]*dims[4]
        if pure and pureType==sim.pure_primitive_cuboid then
            local boxNode=exportFuncs.newNode{'box',size=string.format('%f %f %f',x,y,z)}
            table.insert(geometryNode,boxNode)
        elseif pure and pureType==sim.pure_primitive_spheroid then
            assert(math.abs(x-y)<1e-3 and math.abs(y-z)<1e-3,'incosistent X/Y/Z dimension in sphere')
            local sphereNode=exportFuncs.newNode{'sphere',radius=x/2}
            table.insert(geometryNode,sphereNode)
        elseif pure and pureType==sim.pure_primitive_cylinder then
            assert(math.abs(x-y)<1e-3,'incosistent X/Y dimension in cylinder')
            local cylinderNode=exportFuncs.newNode{'cylinder',radius=x/2,length=z}
            table.insert(geometryNode,cylinderNode)
        else
            local fn=string.format('%s_%s.dae',baseName,sim.getObjectName(shapeHandle))
            simAssimp.exportShapes({shapeHandle},fn,'collada',1.0,simAssimp.upVector.z,4+512)
            local meshNode=exportFuncs.newNode{'mesh',filename='file://'..fn}
            table.insert(geometryNode,meshNode)
        end
        return geometryNode
    end

    exportFuncs.matrixToRPY=exportFuncs.matrixToRPY or function(exportFuncs,m,alternateSolution)
        -- Convert a 3x3 rotation matrix to roll-pitch-yaw coordinates.
        -- URDF's rpy are the Z1-Y2-X3 Tait-Bryan angles.
        -- See https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
        if getmetatable(m)~=Matrix then
            assert(type(m)=='table','table expected')
            assert(#m==9,'not a 3x3 matrix (9 values expected)')
            m=Matrix(3,3,m)
        end
        assert(m:sameshape{3,3},'not a 3x3 matrix')
        local r,p,y=0,0,0
        if math.abs(m[3][1])>=1-1e-12 then
            y=0
            if m[3][1]<0 then
                p=math.pi/2
                r=math.atan2(m[1][2],m[1][3])
            else
                p=-math.pi/2
                r=math.atan2(-m[1][2],-m[1][3])
            end
        else
            if alternateSolution then
                p=-math.asin(m[3][1])
            else
                p=math.pi+math.asin(m[3][1])
            end
            r=math.atan2(m[3][2]/math.cos(p),m[3][3]/math.cos(p))
            y=math.atan2(m[2][1]/math.cos(p),m[1][1]/math.cos(p))
        end
        return {r,p,y}
    end

    exportFuncs.matrixToXYZRPY=exportFuncs.matrixToXYZRPY or function(exportFuncs,m,alternateSolution)
        if getmetatable(m)~=Matrix then
            assert(type(m)=='table','table expected')
            assert(#m==12 or #m==16,'not a 4x4 matrix (12 or 16 values expected)')
            m=Matrix(#m//4,4,m)
        end
        assert(m:sameshape{3,4} or m:sameshape{4,4},'not a 4x4 matrix')
        local R,t=m:slice(1,1,3,3),m:slice(1,4,3,4)
        local xyz={t[1],t[2],t[3]}
        local rpy=exportFuncs.matrixToRPY(exportFuncs,R,alternateSolution)
        return xyz,rpy
    end

    exportFuncs.getShapeOriginNode=exportFuncs.getShapeOriginNode or function(exportFuncs,shapeHandle,parentHandle)
        local originNode=exportFuncs.newNode{'origin'}
        local m=sim.getObjectMatrix(shapeHandle,parentHandle)
        local xyz,rpy=exportFuncs.matrixToXYZRPY(exportFuncs,m)
        originNode.xyz=string.format('%f %f %f',unpack(xyz))
        originNode.rpy=string.format('%f %f %f',unpack(rpy))
        return originNode
    end

    exportFuncs.getLinkInertialNode=exportFuncs.getLinkInertialNode or function(exportFuncs,linkHandle)
        local inertialNode=exportFuncs.newNode{'inertial'}
        local mi,mt=sim.getShapeInertia(linkHandle)
        local xyz,rpy=exportFuncs.matrixToXYZRPY(exportFuncs,mt)
        table.insert(inertialNode,exportFuncs.newNode{'origin',
            xyz=string.format('%f %f %f',unpack(xyz)),
            rpy=string.format('%f %f %f',unpack(rpy)),
        })
        table.insert(inertialNode,exportFuncs.newNode{'inertia',
            ixx=mi[1],
            ixy=(mi[2]+mi[4])/2,
            ixz=(mi[3]+mi[7])/2,
            iyy=mi[5],
            iyz=(mi[6]+mi[8])/2,
            izz=mi[9],
        })
        table.insert(inertialNode,exportFuncs.newNode{'mass',value=sim.getShapeMass(linkHandle)})
        return inertialNode
    end

    exportFuncs.getLinkCollisionNode=exportFuncs.getLinkCollisionNode or function(exportFuncs,linkHandle,baseName)
        local collisionNode=exportFuncs.newNode{'collision'}
        table.insert(collisionNode,exportFuncs.getShapeOriginNode(exportFuncs,linkHandle,linkHandle))
        table.insert(collisionNode,exportFuncs.getShapeGeometryNode(exportFuncs,linkHandle,baseName))
        return collisionNode
    end

    exportFuncs.getVisuals=exportFuncs.getVisuals or function(exportFuncs,linkHandle)
        local visuals={}
        for i,visual in ipairs(sim.getObjectsInTree(linkHandle,sim.object_shape_type,3)) do
            local resp=sim.getObjectInt32Param(visual,sim.shapeintparam_respondable)>0
            if not resp then table.insert(visuals,visual) end
        end
        return visuals
    end

    exportFuncs.getLinkVisualNode=exportFuncs.getLinkVisualNode or function(exportFuncs,visualHandle,linkHandle,baseName)
        local visualNode=exportFuncs.newNode{'visual'}
        table.insert(visualNode,exportFuncs.getShapeOriginNode(exportFuncs,visualHandle,linkHandle))
        table.insert(visualNode,exportFuncs.getShapeGeometryNode(exportFuncs,visualHandle,baseName))
        local materialNode=exportFuncs.newNode{'material',name=sim.getObjectName(visualHandle)..'_material'}
        local r,col=sim.getShapeColor(visualHandle,nil,sim.colorcomponent_ambient_diffuse)
        local colorNode=exportFuncs.newNode{'color',rgba=string.format('%f %f %f 1.0',unpack(col))}
        table.insert(materialNode,colorNode)
        table.insert(visualNode,materialNode)
        return visualNode
    end

    exportFuncs.getLinkNode=exportFuncs.getLinkNode or function(exportFuncs,linkHandle,baseName)
        local linkNode=exportFuncs.newNode{'link',name=sim.getObjectName(linkHandle)}
        table.insert(linkNode,exportFuncs.getLinkInertialNode(exportFuncs,linkHandle))
        table.insert(linkNode,exportFuncs.getLinkCollisionNode(exportFuncs,linkHandle,baseName))
        local visuals=exportFuncs.getVisuals(exportFuncs,linkHandle)
        if #visuals>1 then
            error('only one visual per link is supported') -- maybe group shapes instead?
        elseif #visuals==1 then
            table.insert(linkNode,exportFuncs.getLinkVisualNode(exportFuncs,visuals[1],linkHandle,baseName))
        end
        return linkNode
    end

    exportFuncs.getJointType=exportFuncs.getJointType or function(exportFuncs,jointHandle)
        local jointType=sim.getJointType(jointHandle)
        local cyclic,interval=sim.getJointInterval(jointHandle)
        if jointType==sim.joint_revolute_subtype then
            return cyclic and 'continuous' or 'revolute'
        elseif jointType==sim.joint_prismatic_subtype then
            return 'prismatic'
        elseif jointType==sim.joint_spherical_subtype then
            error('spherical joints not supported by URDF')
        else
            error('unknown joint type for '..sim.getObjectName(jointHandle))
        end
    end

    exportFuncs.getJointAxisNode=exportFuncs.getJointAxisNode or function(exportFuncs,jointHandle)
        local axisNode=exportFuncs.newNode{'axis',xyz='0 0 1'}
        return axisNode
    end

    exportFuncs.getJointLimitNode=exportFuncs.getJointLimitNode or function(exportFuncs,jointHandle)
        local cyclic,interval=sim.getJointInterval(jointHandle)
        if not cyclic then
            local limitNode=exportFuncs.newNode{'limit'}
            limitNode.lower=interval[1]
            limitNode.upper=interval[1]+interval[2]
            limitNode.effort=sim.getJointMaxForce(jointHandle)
            limitNode.velocity=sim.getObjectFloatParam(jointHandle,sim.jointfloatparam_upper_limit)
            return limitNode
        end
    end

    exportFuncs.getJointOriginNode=exportFuncs.getJointOriginNode or function(exportFuncs,jointHandle,parentHandle,childHandle)
        local originNode=exportFuncs.newNode{'origin'}
        local m=sim.getObjectMatrix(childHandle,parentHandle)
        local xyz,rpy=exportFuncs.matrixToXYZRPY(exportFuncs,m)
        originNode.xyz=string.format('%f %f %f',unpack(xyz))
        originNode.rpy=string.format('%f %f %f',unpack(rpy))
        return originNode
    end

    exportFuncs.getJointNode=exportFuncs.getJointNode or function(exportFuncs,jointHandle,parentHandle,childHandle)
        local jointNode=exportFuncs.newNode{'joint'}
        jointNode.name=sim.getObjectName(jointHandle)
        jointNode.type=exportFuncs.getJointType(exportFuncs,jointHandle)
        table.insert(jointNode,exportFuncs.getJointAxisNode(exportFuncs,jointHandle))
        local limitNode=exportFuncs.getJointLimitNode(exportFuncs,jointHandle)
        if limitNode~=nil then table.insert(jointNode,limitNode) end
        table.insert(jointNode,exportFuncs.newNode{'parent',link=sim.getObjectName(parentHandle)})
        table.insert(jointNode,exportFuncs.newNode{'child',link=sim.getObjectName(childHandle)})
        table.insert(jointNode,exportFuncs.getJointOriginNode(exportFuncs,jointHandle,parentHandle,childHandle))
        return jointNode
    end

    exportFuncs.getRobotNode=exportFuncs.getRobotNode or function(exportFuncs,modelHandle,baseName)
        local robotNode=exportFuncs.newNode{'robot',name=sim.getObjectName(modelHandle)}
        local linkDone={}
        for i,link in ipairs(exportFuncs.getModelHierarchy(exportFuncs,modelHandle)) do
            for j,linkHandle in ipairs{link.parentHandle,link.childHandle} do
                if not linkDone[linkHandle] then
                    linkDone[linkHandle]=true
                    table.insert(robotNode,exportFuncs.getLinkNode(exportFuncs,linkHandle,baseName))
                end
            end
            table.insert(robotNode,exportFuncs.getJointNode(exportFuncs,link.jointHandle,link.parentHandle,link.childHandle))
        end
        return robotNode
    end

    exportFuncs.getModelHierarchy=exportFuncs.getModelHierarchy or function(exportFuncs,modelHandle)
        local links={}
        for i,jointHandle in ipairs(sim.getObjectsInTree(modelHandle,sim.object_joint_type,1)) do
            local parentHandle=sim.getObjectParent(jointHandle)
            if sim.getObjectType(parentHandle)~=sim.object_joint_type then
                for j,childHandle in ipairs(sim.getObjectsInTree(jointHandle,sim.object_shape_type,3)) do
                    if sim.getObjectType(childHandle)~=sim.object_joint_type then
                        local link={
                            parentHandle=parentHandle,
                            jointHandle=jointHandle,
                            childHandle=childHandle,
                        }
                        table.insert(links,link)
                    end
                end
            end
        end
        return links
    end

    if outputMode=='f' then return exportFuncs end
    local tree=exportFuncs.getRobotNode(exportFuncs,modelHandle,baseName)
    if outputMode=='tree' then return tree end
    local xml=exportFuncs.toXML(exportFuncs,tree)
    if outputMode=='string' then return xml end
    if outputMode=='file' then
        local f=io.open(fileName,'w+')
        f:write(xml)
        f:close()
        return tree
    end
    error('unknown outputMode: '..outputMode)
end

function simURDF.sendTF(modelHandle,fileName)
    if not simROS2 then
        error('ROS2 plugin not available')
    end
    simROS2.importInterface('geometry_msgs/msg/TransformStamped')
    local tfs={}
    local ef=simURDF.export(modelHandle,fileName,'f')
    for i,link in ipairs(ef.getModelHierarchy(ef,modelHandle)) do
        local tf=geometry_msgs.msg.TransformStamped.__new()
        tf.header.frame_id=sim.getObjectName(link.parentHandle)
        tf.header.stamp=simROS2.getTime(simROS2.clock_type.system)
        tf.child_frame_id=sim.getObjectName(link.childHandle)
        local p=sim.getObjectPose(link.childHandle,link.parentHandle)
        tf.transform.translation={x=p[1],y=p[2],z=p[3]}
        tf.transform.rotation={x=p[4],y=p[5],z=p[6],w=p[7]}
        table.insert(tfs,tf)
    end
    simROS2.sendTransforms(tfs)
end

return simURDF
