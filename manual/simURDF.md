# URDF Plugin API reference

API functions for URDF input.

## Alphabetical list

- [simURDF.export](#export)
- [simURDF.import](#import)

## Commands

### <a name="export" id="export"></a> <a name="simURDF.export" id="simURDF.export"></a> simURDF.export

Exports a URDF file

#### Synopsis:

```lua
simURDF.export(int modelHandle,string fileName,int options=0)
```

#### Parameters:

- **modelHandle** (int): the handle of a model
- **fileName** (string): the URDF file path
- **options** (int, default: 0): bit coded: bit0=transform dummies into red cubes, bit1=make visuals and collisions origin frame where parent joint is, if possible, bit2=do not reset joint positions to 0

#### Return values:

No return values

#### See also:

[import](#import)

### <a name="import" id="import"></a> <a name="simURDF.import" id="simURDF.import"></a> simURDF.import

Imports a URDF file

#### Synopsis:

```lua
string robotName=simURDF.import(string fileName,int options=0,string replacementString=nil)
```

#### Parameters:

- **fileName** (string): the URDF file path
- **options** (int, default: 0): bit coded: bit0=show collision links, bit1=showJoints, bit2=perform convex decompose on non convex collidables, bit3=create visual if none, bit5=do not center above ground, bit6=do not make model, bit7=do not alternate local respondable masks, bit8=no position control of dynamic joints, bit9=perform convex hull on non convex collidables, bit10=set shape origin at joint location where possible
- **replacementString** (string, default: nil): an optional string that replaces all occurences of "package://" in the XML file

#### Return values:

- **robotName** (string): name of the robot

#### See also:

[export](#export)
