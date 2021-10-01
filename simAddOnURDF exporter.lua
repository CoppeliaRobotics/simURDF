function sysCall_info()
    return {autoStart=false,menu='Exporters\nURDF exporter'}
end

function sysCall_addOnScriptSuspend()
    return {cmd='cleanup'}
end

function close_callback()
    leaveNow=true
end

function action()
    local model=selectedModel
    hideDlg()
    local importExportDir=sim.getStringParam(sim.stringparam_importexportdir)
    local file=simUI.fileDialog(simUI.filedialog_type.save,"Export URDF...",importExportDir,"","URDF file","urdf",true)
    if file and #file==1 and #file[1]>0 then
        sim.setStringParam(sim.stringparam_importexportdir,file[1])
        simURDF.export(model,file[1])
        sim.addLog(sim.verbosity_scriptinfos,"Model successfully exported to "..file[1])
        sim.setObjectSelection({})
    end
end

function sysCall_init()
    sim.addLog(sim.verbosity_scriptinfos,"This add-on allows to export a model to URDF. Just select a model, and make sure its base is a shape object.")
    selectedObject=-1
end

function showDlg()
    if not ui then
        local pos='position="-50,50" placement="relative"'
        if uiPos then
            pos='position="'..uiPos[1]..','..uiPos[2]..'" placement="absolute"'
        end
        local xml ='<ui title="URDF Exporter" activate="false" closeable="true" on-close="close_callback" layout="vbox" '..pos..[[>
                <button text="Export model ]]..sim.getObjectAlias(selectedModel,1)..[[" on-click="action" style="* {min-width: 300px; min-height: 50px;}"/>
        </ui>]]
        ui=simUI.create(xml)
    end
end

function hideDlg()
    if ui then
        uiPos={}
        uiPos[1],uiPos[2]=simUI.getPosition(ui)
        simUI.destroy(ui)
        ui=nil
    end
    selectedModel=-1
end

function sysCall_nonSimulation()
    if leaveNow then
        hideDlg()
        return {cmd='cleanup'}
    end
    local s=sim.getObjectSelection()
    if s and #s==1 then
        if selectedModel~=s[1] then
            hideDlg()
        end
        selectedModel=-1
        if sim.getModelProperty(s[1])&sim.modelproperty_not_model==0 and sim.getObjectType(s[1])==sim.object_shape_type then
            selectedModel=s[1]
            showDlg()
        end
    else
        hideDlg()
    end
end

function sysCall_cleanup()
    hideDlg()
end

function sysCall_beforeInstanceSwitch()
    hideDlg()
    selectedModel=-1
end

function close_callback()
    leaveNow=true
end

function sysCall_beforeSimulation()
    hideDlg()
end
