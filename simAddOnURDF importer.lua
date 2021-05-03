function importURDF()
    local fn=options.fileName
    local opts=0
    if not options.hideCollisionLinks then opts=opts+1 end
    if not options.hideJoints then opts=opts+2 end
    if not options.convexDecompose then opts=opts+4 end
    if not options.createVisualIfNone then opts=opts+8 end
    if options.showConvexDecompositionDlg then opts=opts+16 end
    if not options.centerModel then opts=opts+32 end
    if not options.prepareModel then opts=opts+64 end
    if not options.alternateLocalRespondableMasks then opts=opts+128 end
    if not options.positionCtrl then opts=opts+256 end
    local success,err=pcall(function() simURDF.import(fn,opts) end)
    if err then
        simUI.msgBox(simUI.msgbox_type.info,simUI.msgbox_buttons.ok,'Error','Error: '..err)
    end
    closeDialog()
end

function closeDialog()
    if ui then
        simUI.destroy(ui)
        ui=nil
    end
    done=true
end

function updateOptions(ui,id,val)
    local function val2bool(v)
        if v==0 then return false else return true end
    end
    if optionsInfo[id] then
        options[optionsInfo[id].key]=val2bool(val)
    end
end

function sysCall_info()
    return {autoStart=false}
end

function sysCall_init()
    closeDialog()

    optionsInfo={
        [20]={name='Assign collision links to layer 9',key='hideCollisionLinks'},
        [30]={name='Assign joints to layer 10',key='hideJoints'},
        [40]={name='Convex decompose non-convex collision links',key='convexDecompose'},
        [50]={name='Show convex decomposition dialog',key='showConvexDecompositionDlg'},
        [60]={name='Create visual links if none',key='createVisualIfNone'},
        [70]={name='Center model above ground',key='centerModel'},
        [80]={name='Prepare model definition if feasible',key='prepareModel'},
        [90]={name='Alternate local respondable masks',key='alternateLocalRespondableMasks'},
        [100]={name='Enable position ctrl of joints',key='positionCtrl'},
    }

    options={
        hideCollisionLinks=true,
        hideJoints=true,
        convexDecompose=true,
        showConvexDecompositionDlg=false,
        createVisualIfNone=true,
        centerModel=true,
        prepareModel=true,
        alternateLocalRespondableMasks=false,
        positionCtrl=true,
    }

    local scenePath=sim.getStringParameter(sim.stringparam_scene_path)
    local fileName=sim.fileDialog(sim.filedlg_type_load,'Import URDF...',scenePath,'','URDF file','urdf')

    if fileName then
        done=false
        options.fileName=fileName
        local function checkbox(id,text,varname)
        end
        local xml='<ui modal="true" layout="vbox" title="Importing '..fileName..'..." closeable="true" on-close="closeDialog">\n'
        for id,o in pairs(optionsInfo) do
            xml=xml..'<checkbox id="'..id..'" checked="'..(options[o.key] and 'true' or 'false')..'" text="'..o.name..'" on-change="updateOptions" />\n'
        end
        xml=xml..'<button text="Import" on-click="importURDF" />\n'
        xml=xml..[[<label wordwrap="true" text="If you experience a sudden crash during the import operation, make sure to enable 'Show convex decomposition dialog' and adjust the parameters there. Another option would be to simply disable 'Convex decompose non-convex collision links'." />]]..'\n'
        xml=xml..'</ui>'
        ui=simUI.create(xml)
    end
end

function sysCall_nonSimulation()
    if done then
        return {cmd='cleanup'}
    end
end

function sysCall_cleanup()
    closeDialog()
end
