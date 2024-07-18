sim = require 'sim'

function sysCall_info()
    return {autoStart = false, menu = 'Exporters\nURDF exporter...'}
end

function sysCall_init()
    simUI = require 'simUI'
    simURDF = require 'simURDF'

    closeDialog()
    local s = sim.getObjectSel()
    local model = nil
    if #s == 1 then
        if sim.getModelProperty(s[1]) & sim.modelproperty_not_model == 0 and sim.getObjectType(s[1]) ==
            sim.sceneobject_shape then model = s[1] end
    end
    if model then
        local importExportDir = sim.getStringParam(sim.stringparam_importexportdir)
        local fileNames = simUI.fileDialog(
                         simUI.filedialog_type.save, "Export URDF...", importExportDir, "",
                         "URDF file", "urdf", true
                     )
        if #fileNames > 0 then
            optionsInfo = {
                [1] = {name = 'Reset joints', key = 'resetJoints'},
                [2] = {
                    name = 'Set shape origin at joint location where possible',
                    key = 'shapeAtJointloc',
                },
                [3] = {name = 'Make red cubes from dummies', key = 'redCubes'},
            }
            options = {
                model = model,
                fileName = fileNames[1],
                resetJoints = true,
                shapeAtJointloc = false,
                redCubes = false,
            }
            sim.setStringParam(sim.stringparam_importexportdir, options.fileName)
            done = false
            local xml = '<ui modal="true" layout="vbox" title="Exporting ' .. options.fileName ..
                            '..." closeable="true" on-close="closeDialog">\n'
            for i = 1, #optionsInfo, 1 do
                local o = optionsInfo[i]
                xml = xml .. '<checkbox id="' .. i .. '" checked="' ..
                          (options[o.key] and 'true' or 'false') .. '" text="' .. o.name ..
                          '" on-change="updateOptions" />\n'
            end
            xml = xml .. '<button text="Export" on-click="exportURDF" />\n'
            xml = xml .. '</ui>'
            ui = simUI.create(xml)
        end
    else
        simUI.msgBox(
            simUI.msgbox_type.critical, simUI.msgbox_buttons.ok, 'URDF exporter',
            'This tool requires exactly one model to be selected.'
        )
        return {cmd = 'cleanup'}
    end
end

function sysCall_nonSimulation()
    if done then return {cmd = 'cleanup'} end
end

function sysCall_cleanup()
    closeDialog()
end

function exportURDF()
    local opts = 0
    if not options.resetJoints then opts = opts | 4 end
    if options.shapeAtJointloc then opts = opts | 1 end
    if options.redCubes then opts = opts | 2 end
    closeDialog()
    simURDF.export(options.model, options.fileName, opts)
    sim.addLog(sim.verbosity_scriptinfos, "Model successfully exported to " .. options.fileName)
end

function closeDialog()
    if ui then
        simUI.destroy(ui)
        ui = nil
    end
    done = true
end

function updateOptions(ui, id, val)
    local function val2bool(v)
        if v == 0 then
            return false
        else
            return true
        end
    end
    if optionsInfo[id] then options[optionsInfo[id].key] = val2bool(val) end
end
