local codeEditorInfos=[[
string robot_name=simURDF.import(string urdf,int options=0,string packageStrReplace=nil)
simURDF.export(origModel,fileName,options)
simURDF.sendTF(modelHandle,fileName)
]]

registerCodeEditorInfos("simURDF",codeEditorInfos)