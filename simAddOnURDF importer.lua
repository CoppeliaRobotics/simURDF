function sysCall_info()
    return {autoStart=false,menu='Importers\nURDF importer'}
end

function sysCall_init()
    closeDialog()

    optionsInfo={
        [1]={name='Assign collision links to layer 9',key='hideCollisionLinks'},
        [2]={name='Assign joints to layer 10',key='hideJoints'},
        [3]={name='Convex hull of non-convex collision links',key='convexHull'},
        [4]={name='Convex decompose non-convex collision links',key='convexDecompose'},
        [5]={name='Create visual links if none',key='createVisualIfNone'},
        [6]={name='Center model above ground',key='centerModel'},
        [7]={name='Prepare model definition if feasible',key='prepareModel'},
        [8]={name='Alternate local respondable masks',key='alternateLocalRespondableMasks'},
        [9]={name='Enable position ctrl of joints',key='positionCtrl'},
    }

    options={
        hideCollisionLinks=true,
        hideJoints=true,
        convexDecompose=false,
        convexHull=false,
        createVisualIfNone=true,
        centerModel=true,
        prepareModel=true,
        alternateLocalRespondableMasks=false,
        positionCtrl=true,
    }

    local importExportDir=sim.getStringParam(sim.stringparam_importexportdir)
    local fileName=simUI.fileDialog(simUI.filedialog_type.load,"Import URDF...",importExportDir,"","URDF file","urdf",true)

    if fileName and #fileName==1 and #fileName[1]>0 then
        sim.setStringParam(sim.stringparam_importexportdir,fileName[1])
        done=false
        options.fileName=fileName[1]
        local function checkbox(id,text,varname)
        end
        local xml='<ui modal="true" layout="vbox" title="Importing '..fileName[1]..'..." closeable="true" on-close="closeDialog">\n'
        for i=1,#optionsInfo,1 do
            local o=optionsInfo[i]
            xml=xml..'<checkbox id="'..i..'" checked="'..(options[o.key] and 'true' or 'false')..'" text="'..o.name..'" on-change="updateOptions" />\n'
        end
        xml=xml..[[<label text="replace occurences of 'package://' with:"/>]]
        xml=xml..'<edit on-editing-finished="editFinished_callback" id="999" />'
        xml=xml..'<button text="Import" on-click="importURDF" />\n'
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

function importURDF()
    local fn=options.fileName
    local opts=0
    if not options.hideCollisionLinks then opts=opts+1 end
    if not options.hideJoints then opts=opts+2 end
    if options.convexDecompose then opts=opts+4 end
    if options.convexHull then opts=opts+512 end
    if options.createVisualIfNone then opts=opts+8 end
    if not options.centerModel then opts=opts+32 end
    if not options.prepareModel then opts=opts+64 end
    if not options.alternateLocalRespondableMasks then opts=opts+128 end
    if not options.positionCtrl then opts=opts+256 end
    closeDialog()
    pcall(function() simURDF.import(fn,opts,packageStr) end)
end

function closeDialog()
    if ui then
        simUI.destroy(ui)
        ui=nil
    end
    done=true
end

function editFinished_callback(ui,id,val)
    packageStr=val
end

function updateOptions(ui,id,val)
    local function val2bool(v)
        if v==0 then return false else return true end
    end
    if optionsInfo[id] then
        options[optionsInfo[id].key]=val2bool(val)
    end
end


