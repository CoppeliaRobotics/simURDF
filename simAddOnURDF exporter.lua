function sysCall_info()
    return {autoStart=false,menu='Exporters\nURDF exporter...'}
end

function sysCall_init()
    local s=sim.getObjectSelection()
    local model=nil
    if s and #s==1 then
        if sim.getModelProperty(s[1])&sim.modelproperty_not_model==0 and sim.getObjectType(s[1])==sim.object_shape_type then
            model=s[1]
        end
    end
    if model then
        local importExportDir=sim.getStringParam(sim.stringparam_importexportdir)
        local file=simUI.fileDialog(simUI.filedialog_type.save,"Export URDF...",importExportDir,"","URDF file","urdf",true)
        if file and #file==1 and #file[1]>0 then
            sim.setStringParam(sim.stringparam_importexportdir,file[1])
            simURDF.export(model,file[1])
            sim.addLog(sim.verbosity_scriptinfos,"Model successfully exported to "..file[1])
        end
    else
        simUI.msgBox(simUI.msgbox_type.critical,simUI.msgbox_buttons.ok,'URDF exporter','This tool requires exactly one model to be selected.')
    end
    return {cmd='cleanup'}
end
