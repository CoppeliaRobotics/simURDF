local simURDF = loadPlugin 'simURDF'
local sim = require 'sim-2'
local simAssimp = require 'simAssimp'
local __ = {}
__.urdf = {}

simURDF.importFile = simURDF.import

function simURDF.export(origModel, fileName, options)
    options = options or 0
    -- options & bit0: transform dummies into red cubes
    -- options & bit1: make visuals and collisions origin frame where parent joint is, if possible
    -- options & bit2: do not reset joint positions to 0
    assert(sim.isHandle(origModel), 'not a valid handle')
    local baseName = fileName
    if fileName:find('%.urdf$') then
        baseName = fileName:sub(1, -6)
    else
        fileName = fileName .. '.urdf'
    end

    local model = sim.copyPasteObjects({origModel}, 1 + 2 + 4 + 8 + 16 + 32)[1] -- work with a copy
    sim.setBoolProperty(model, 'modelBase', sim.getBoolProperty(origModel, 'modelBase'))
    sim.setIntProperty(model, 'model.propertyFlags', sim.getIntProperty(origModel, 'model.propertyFlags'))
    if (options & 1) ~= 0 then __.urdf.replaceDummies(model) end
    if (options & 4) == 0 then
        local l = sim.getObjectsInTree(model, sim.sceneobject_joint)
        for i = 1, #l, 1 do
            if sim.getJointType(l[i]) ~= sim.joint_spherical then
                sim.setJointPosition(l[i], 0)
            end
        end
    end
    __.urdf.insertAuxShapes(model)
    local info = {baseFile = baseName, base = model, options = options}
    local tree = __.urdf.newNode({'robot', name = sim.getObjectAlias(model)})
    local dynamicStage = 0
    if sim.getBoolProperty(model, 'model.notDynamic') then dynamicStage = 2 end
    local tree = __.urdf.parseAndCreateMeshFiles(tree, model, -1, -1, dynamicStage, info)
    sim.removeModel(model)
    local xml = __.urdf.toXML(tree)
    local f = io.open(fileName, 'w+')
    f:write(xml)
    f:close()
end

function __.urdf.newNode(t)
    assert(type(t) == 'table', 'bad type')
    local name = table.remove(t, 1)
    t[0] = name
    return t
end

function __.urdf.toXML(node, level)
    level = level or 0
    local indent = '';
    for i = 1, level do indent = indent .. '    ' end
    local xml = ''
    if level == 0 then xml = xml .. '<?xml version="1.0"?>\n' end
    xml = xml .. indent .. '<' .. node[0]
    for k, v in pairs(node) do
        if type(k) ~= 'number' then
            for c, r in pairs {['"'] = '&quot;', ['<'] = '&lt;', ['>'] = '&gt;'} do
                v = string.gsub(v, c, r)
            end
            xml = xml .. ' ' .. k .. '="' .. v .. '"'
        end
    end
    if #node > 0 then
        xml = xml .. '>\n'
        for i, child in ipairs(node) do xml = xml .. __.urdf.toXML(child, level + 1) end
        xml = xml .. indent .. '</' .. node[0] .. '>\n'
    else
        xml = xml .. ' />\n'
    end
    return xml
end

function __.urdf.appendVisibleChildShapes(array, object)
    local shapes = sim.getObjectsInTree(object, sim.sceneobject_shape, 1 | 2)
    for i = 1, #shapes, 1 do
        if sim.getBoolProperty(shapes[i], 'visible') then
            array[#array + 1] = shapes[i]
        end
        __.urdf.appendVisibleChildShapes(array, shapes[i])
    end
end

function __.urdf.getFirstJoints(object, dynamicStage)
    local retVal = {}
    local objs = {}
    local ind = 0
    while true do
        local child = sim.getObjectChild(object, ind)
        if child == -1 then break end
        objs[#objs + 1] = {object = child, dynamicStage = dynamicStage}
        ind = ind + 1
    end
    for i = 1, #objs, 1 do
        local obj = objs[i].object
        local dynStage = objs[i].dynamicStage
        
        if not sim.getBoolProperty(obj, 'notVisible') then
            if sim.getBoolProperty(obj, 'notDynamic') then
                dynStage = 2 -- rest of the chain cannot be dynamic anymore
            end
            local objType = sim.getObjectType(obj)
            if objType == sim.sceneobject_shape then
                if sim.getBoolProperty(obj, 'dynamic') then
                    if dynStage == 0 then dynStage = 1 end
                else
                    if dynStage == 1 then
                        dynStage = 2 -- rest of the chain cannot be dynamic anymore
                    end
                end
                local ind = 0
                while true do
                    local child = sim.getObjectChild(obj, ind)
                    if child == -1 then break end
                    objs[#objs + 1] = {object = child, dynamicStage = dynStage}
                    ind = ind + 1
                end
            elseif (objType == sim.sceneobject_joint) then
                if sim.getJointMode(obj) == sim.jointmode_dynamic then
                    if dynStage == 0 then dynStage = 1 end
                else
                    if dynStage == 1 then
                        dynStage = 2 -- rest of the chain cannot be dynamic anymore
                    end
                end
                retVal[#retVal + 1] = {joint = obj, dynamicStage = dynStage}
            elseif (objType == sim.sceneobject_forcesensor) then
                retVal[#retVal + 1] = {joint = obj, dynamicStage = dynStage}
            end
        end
    end
    return retVal
end

function __.urdf.matrixToRPY(m)
    -- Convert a 3x3 rotation matrix to roll-pitch-yaw coordinates.
    -- URDF's rpy are the Z1-Y2-X3 Tait-Bryan angles.
    -- See https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
    if getmetatable(m) ~= Matrix then
        assert(type(m) == 'table', 'table expected')
        assert(#m == 9, 'not a 3x3 matrix (9 values expected)')
        m = Matrix(3, 3, m)
    end
    assert(m:sameshape{3, 3}, 'not a 3x3 matrix')
    local r, p, y = 0, 0, 0
    if math.abs(m[3][1]) >= 1 - 1e-12 then
        y = 0
        if m[3][1] < 0 then
            p = math.pi / 2
            r = math.atan2(m[1][2], m[1][3])
        else
            p = -math.pi / 2
            r = math.atan2(-m[1][2], -m[1][3])
        end
    else
        p = math.pi + math.asin(m[3][1])
        r = math.atan2(m[3][2] / math.cos(p), m[3][3] / math.cos(p))
        y = math.atan2(m[2][1] / math.cos(p), m[1][1] / math.cos(p))
    end
    return {r, p, y}
end

function __.urdf.matrixToXYZRPY(m)
    if getmetatable(m) ~= Matrix then
        assert(type(m) == 'table', 'table expected')
        assert(#m == 12 or #m == 16, 'not a 4x4 matrix (12 or 16 values expected)')
        m = Matrix(#m // 4, 4, m)
    end
    assert(m:sameshape{3, 4} or m:sameshape{4, 4}, 'not a 4x4 matrix')
    local R, t = m:slice(1, 1, 3, 3), m:slice(1, 4, 3, 4)
    local xyz = {t[1], t[2], t[3]}
    local rpy = __.urdf.matrixToRPY(R)
    return xyz, rpy
end

function __.urdf.getShapeOriginNode(shape, parent)
    local originNode = __.urdf.newNode {'origin'}
    local xyz, rpy = __.urdf.matrixToXYZRPY(
                         sim.getObjectMatrix(
                             shape | sim.handleflag_reljointbaseframe, parent
                         )
                     )
    originNode.xyz = string.format('%f %f %f', unpack(xyz))
    originNode.rpy = string.format('%f %f %f', unpack(rpy))
    return originNode
end

function __.urdf.getLinkInertialNode(shape, parent)
    local inertialNode = __.urdf.newNode {'inertial'}
    local mi, mt = sim.getShapeInertia(shape)
    local m = sim.getObjectMatrix(shape | sim.handleflag_reljointbaseframe, parent)
    mt = sim.multiplyMatrices(m, mt)
    local xyz, rpy = __.urdf.matrixToXYZRPY(mt)
    table.insert(
        inertialNode, __.urdf.newNode {
            'origin',
            xyz = string.format('%f %f %f', unpack(xyz)),
            rpy = string.format('%f %f %f', unpack(rpy)),
        }
    )
    table.insert(
        inertialNode, __.urdf.newNode {
            'inertia',
            ixx = mi[1],
            ixy = (mi[2] + mi[4]) / 2,
            ixz = (mi[3] + mi[7]) / 2,
            iyy = mi[5],
            iyz = (mi[6] + mi[8]) / 2,
            izz = mi[9],
        }
    )
    table.insert(inertialNode, __.urdf.newNode {'mass', value = sim.getFloatProperty(shape, 'mass')})
    return inertialNode
end

function __.urdf.getCollisionOrVisualNodes(shape, prevJoint, info, isCollisionNode)
    local retVal = {}
    local originalFramePose = sim.getObjectPose(shape)
    local shapes = sim.copyPasteObjects({shape}, 2 + 4 + 8 + 16 + 32)
    for i = 1, #shapes, 1 do
        local r, pureType, dims = sim.getShapeGeomInfo(shapes[i])
        if (r & 1) ~= 0 then
            local subShapes = sim.ungroupShape(shapes[i])
            table.remove(shapes, i)
            for j = 1, #subShapes, 1 do shapes[#shapes + 1] = subShapes[j] end
        end
    end
    for i = 1, #shapes, 1 do
        local aShape = shapes[i]
        if (info.options & 2) ~= 0 then
            sim.relocateShapeFrame(aShape, {0, 0, 0, 0, 0, 0, 1}) -- We ungrouped a shape. We try to keep the same frame
        else
            if #shapes > 1 then
                sim.relocateShapeFrame(aShape, originalFramePose) -- We ungrouped a shape. We try to keep the same frame
            end
        end
        local node, nm
        if isCollisionNode then
            node = __.urdf.newNode {'collision'}
            nm = __.urdf.getObjectName(shape, info) .. '_coll_' .. i
        else
            node = __.urdf.newNode {'visual'}
            nm = __.urdf.getObjectName(shape, info) .. '_vis_' .. i
        end
        table.insert(node, __.urdf.getShapeOriginNode(aShape, prevJoint))
        table.insert(node, __.urdf.getShapeGeometryNode(aShape, nm, info))

        if not isCollisionNode then
            local materialNode = __.urdf.newNode {
                'material',
                name = __.urdf.getObjectName(shape, info) .. '_material',
            }
            local r, col = sim.getShapeColor(aShape, nil, sim.colorcomponent_ambient_diffuse)
            local colorNode = __.urdf.newNode {
                'color',
                rgba = string.format('%f %f %f 1.0', unpack(col)),
            }
            table.insert(materialNode, colorNode)
            table.insert(node, materialNode)
        end

        node.name = sim.getObjectAlias(shape)
        retVal[#retVal + 1] = node
    end
    sim.removeObjects(shapes)
    return retVal
end

function __.urdf.getObjectName(objectHandle, info)
    local nm = 'robot_base'
    if objectHandle ~= info.base then
        nm = sim.getObjectAlias(objectHandle)
        local obj = -1
        local i = -1
        while obj ~= objectHandle do
            i = i + 1
            obj = sim.getObject('./' .. nm, {index = i, proxy = info.base})
        end
        if i ~= 0 then
            nm = nm .. '_' .. (i + 1) -- there are several objects with the same alias!
            if info.sameAliasWarning == nil then
                info.sameAliasWarning = true
                addLog(sim.verbosity_warnings, "found one or several objects with identical alias.")
            end
        end
    end
    return nm
    --[[
    local n=sim.getObjectAliasRelative(objectHandle,info.base,7)
    n=n:gsub('%W','')
    if n=='' then return 'robot_base' end
    return n
    --]]
end

function __.urdf.getShapeGeometryNode(shape, name, info)
    local geometryNode = __.urdf.newNode {'geometry'}
    local r, pureType, dims = sim.getShapeGeomInfo(shape)
    local pure = (r & 2) > 0
    local x, y, z = dims[1], dims[2], dims[3]
    if pure and pureType == sim.primitiveshape_cuboid then
        local boxNode = __.urdf.newNode {'box', size = string.format('%f %f %f', x, y, z)}
        table.insert(geometryNode, boxNode)
    elseif pure and pureType == sim.primitiveshape_spheroid then
        assert(
            math.abs(x - y) < 1e-3 and math.abs(y - z) < 1e-3,
            'incosistent X/Y/Z dimension in sphere'
        )
        local sphereNode = __.urdf.newNode {'sphere', radius = x / 2}
        table.insert(geometryNode, sphereNode)
    elseif pure and pureType == sim.primitiveshape_cylinder then
        assert(math.abs(x - y) < 1e-3, 'incosistent X/Y dimension in cylinder')
        local cylinderNode = __.urdf.newNode {'cylinder', radius = x / 2, length = z}
        table.insert(geometryNode, cylinderNode)
    else
        --        local fn=string.format('%s_%s.dae',info.baseFile,name)
        --        simAssimp.exportShapes({shape},fn,'collada',1.0,simAssimp.upVector.z,4+512)
        local fn = string.format('%s_%s.dae', info.baseFile, name)
        simAssimp.exportShapes({shape}, fn, 'collada', 1.0, simAssimp.upVector.z, 4 + 512)
        local meshNode = __.urdf.newNode {'mesh', filename = 'file://' .. fn}
        table.insert(geometryNode, meshNode)
    end
    return geometryNode
end

function __.urdf.getJointType(jointHandle)
    local retVal = 'fixed'
    if sim.getObjectType(jointHandle) == sim.sceneobject_joint then
        local jointType = sim.getJointType(jointHandle)
        local cyclic, interval = sim.getJointInterval(jointHandle)
        if jointType == sim.joint_revolute then
            retVal = cyclic and 'continuous' or 'revolute'
        elseif jointType == sim.joint_prismatic then
            retVal = 'prismatic'
        end
    end
    return retVal
end

function __.urdf.getJointLimitNode(jointHandle)
    if sim.getObjectType(jointHandle) == sim.sceneobject_joint then
        local cyclic, interval = sim.getJointInterval(jointHandle)
        if not cyclic then
            local p = sim.getJointPosition(jointHandle)
            local limitNode = __.urdf.newNode {'limit'}
            limitNode.lower = interval[1] - p
            limitNode.upper = interval[1] - p + interval[2]
            limitNode.effort = sim.getJointTargetForce(jointHandle)
            limitNode.velocity = sim.getFloatArrayProperty(jointHandle, 'maxVelAccelJerk')[1]
            return limitNode
        end
    end
end

function __.urdf.getJointOriginNode(jointHandle, prevJoint)
    local originNode = __.urdf.newNode {'origin'}
    local m = sim.getObjectMatrix(jointHandle | sim.handleflag_reljointbaseframe, prevJoint)
    local xyz, rpy = __.urdf.matrixToXYZRPY(m)
    originNode.xyz = string.format('%f %f %f', unpack(xyz))
    originNode.rpy = string.format('%f %f %f', unpack(rpy))
    return originNode
end

function __.urdf.getJointNode(jointHandle, parentHandle, childHandle, prevJoint, info)
    local jointNode = __.urdf.newNode {'joint'}
    jointNode.name = __.urdf.getObjectName(jointHandle, info)
    jointNode.type = __.urdf.getJointType(jointHandle)
    table.insert(jointNode, __.urdf.newNode {'axis', xyz = '0 0 1'})
    local limitNode = __.urdf.getJointLimitNode(jointHandle)
    if limitNode ~= nil then table.insert(jointNode, limitNode) end
    table.insert(
        jointNode, __.urdf.newNode {'parent', link = __.urdf.getObjectName(parentHandle, info)}
    )
    table.insert(
        jointNode, __.urdf.newNode {'child', link = __.urdf.getObjectName(childHandle, info)}
    )
    table.insert(jointNode, __.urdf.getJointOriginNode(jointHandle, prevJoint))
    return jointNode
end

function __.urdf.replaceDummies(object)
    local ot = sim.getObjectType(object)
    if ot == sim.sceneobject_dummy and sim.getBoolProperty(object, 'visible') then
        local dummyShape = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {0.01, 0.01, 0.01})
        sim.setObjectAlias(dummyShape, sim.getObjectAlias(object))
        sim.setObjectColor(dummyShape, 0, sim.colorcomponent_ambient_diffuse, {1, 0, 0})
        sim.setObjectPose(dummyShape, sim.getObjectPose(object))
        sim.setObjectParent(dummyShape, sim.getObjectParent(object), true)
        sim.setObjectParent(object, dummyShape, true)
        sim.removeObjects({object})
        object = dummyShape
    end
    local l = sim.getObjectsInTree(object, sim.handle_all, 1 | 2)
    for i = 1, #l, 1 do __.urdf.replaceDummies(l[i]) end
end

function __.urdf.insertAuxShapes(object)
    local l = sim.getObjectsInTree(object, sim.handle_all, 1 | 2)
    for i = 1, #l, 1 do
        local ot = sim.getObjectType(object)
        if ot == sim.sceneobject_joint or ot == sim.sceneobject_forcesensor then
            local ct = sim.getObjectType(l[i])
            if ct == sim.sceneobject_joint or ct == sim.sceneobject_forcesensor then
                local auxShape = sim.createPrimitiveShape(sim.primitiveshape_spheroid, {0.05, 0.05, 0.05})
                sim.setObjectAlias(auxShape, 'auxShape')
                sim.setObjectColor(auxShape, 0, sim.colorcomponent_ambient_diffuse, {1, 1, 1})
                sim.setObjectParent(auxShape, object, false)
                sim.setObjectParent(l[i], auxShape, true)
                l[#l + 1] = auxShape
            else
                __.urdf.insertAuxShapes(l[i])
            end
        else
            __.urdf.insertAuxShapes(l[i])
        end
    end
end

function __.urdf.parseAndCreateMeshFiles(tree, object, parent, prevJoint, dynamicStage, info)
    local objectType = sim.getObjectType(object)
    if not sim.getBoolProperty(object, 'model.notVisible') then -- ignore invisible models
        if sim.getBoolProperty(object, 'model.notDynamic') then
            if dynamicStage == 1 then
                dynamicStage = 2 -- rest of the chain cannot be dynamic anymore
            end
        end
        if objectType == sim.sceneobject_shape then
            local linkNode = __.urdf.newNode {'link', name = __.urdf.getObjectName(object, info)}
            local hidden = ((sim.getIntProperty(object, 'layer') & 255) == 0)
            local dyn = sim.getBoolProperty(object, 'dynamic')
            if dyn then
                if dynamicStage == 0 then dynamicStage = 1 end
            else
                if dynamicStage == 1 then
                    dynamicStage = 2 -- rest of the chain cannot be dynamic anymore
                end
            end
            local visuals = {}
            if dynamicStage == 1 then
                table.insert(linkNode, __.urdf.getLinkInertialNode(object, prevJoint))
            end
            if dynamicStage == 1 or hidden then
                -- we create 1-n 'collision' nodes from that object
                local cn = __.urdf.getCollisionOrVisualNodes(object, prevJoint, info, true)
                for i = 1, #cn, 1 do table.insert(linkNode, cn[i]) end
            end
            if not hidden then visuals[#visuals + 1] = object end
            __.urdf.appendVisibleChildShapes(visuals, object, info)
            -- we create 1-n 'visual' nodes from those objects
            for j = 1, #visuals, 1 do
                local cn = __.urdf.getCollisionOrVisualNodes(visuals[j], prevJoint, info, false)
                for i = 1, #cn, 1 do table.insert(linkNode, cn[i]) end
            end
            table.insert(tree, linkNode)
            local firstJoints = __.urdf.getFirstJoints(object, dynamicStage)
            for i = 1, #firstJoints, 1 do
                tree = __.urdf.parseAndCreateMeshFiles(
                           tree, firstJoints[i].joint, object, prevJoint,
                           firstJoints[i].dynamicStage, info
                       )
            end
        elseif objectType == sim.sceneobject_joint or objectType == sim.sceneobject_forcesensor then
            local child = sim.getObjectChild(object, 0)
            if child ~= -1 then
                if sim.getBoolProperty(child, 'model.notVisible') then -- ignore invisible models
                    child = -1
                else
                    local childT = sim.getObjectType(child)
                    if childT ~= sim.sceneobject_shape then
                        child = -1 -- we only take into account joints that have at least one shape in-between
                    end
                end
            end
            if parent ~= -1 and child ~= -1 then
                table.insert(tree, __.urdf.getJointNode(object, parent, child, prevJoint, info))
                if objectType == sim.sceneobject_joint and sim.getJointMode(object) ~=
                    sim.jointmode_dynamic and dynamicStage == 1 then dynamicStage = 2 end
                prevJoint = object
                tree = __.urdf.parseAndCreateMeshFiles(tree, child, object, prevJoint, dynamicStage, info)
            end
        end
    end
    return tree
end

function simURDF.sendTF(modelHandle, fileName)
    local simROS2
    pcall(
        function()
            simROS2 = require 'simROS2'
        end
    )
    simROS2.importInterface('geometry_msgs/msg/TransformStamped')
    local tfs = {}
    local ef = simURDF.export(modelHandle, fileName, 'f')
    for i, link in ipairs(ef.getModelHierarchy(ef, modelHandle)) do
        local tf = geometry_msgs.msg.TransformStamped.__new()
        tf.header.frame_id = ef.getObjectName(ef, link.parentHandle)
        tf.header.stamp = simROS2.getTime(simROS2.clock_type.system)
        tf.child_frame_id = ef.getObjectName(ef, link.childHandle)
        local p = sim.getObjectPose(link.childHandle, link.parentHandle)
        tf.transform.translation = {x = p[1], y = p[2], z = p[3]}
        tf.transform.rotation = {x = p[4], y = p[5], z = p[6], w = p[7]}
        table.insert(tfs, tf)
    end
    simROS2.sendTransforms(tfs)
end

return simURDF
