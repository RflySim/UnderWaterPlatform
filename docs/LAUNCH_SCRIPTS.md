# Launch Script Notes

This repository includes four one-click launch scripts in `UUV/`:

| Script | Mode | Renderer |
|---|---|---|
| `UUVModel_SITL.bat` | Software-in-the-loop | RflySim3D / UE4 workflow |
| `UUVModel_HITL.bat` | Hardware-in-the-loop | RflySim3D / UE4 workflow |
| `UUVModel_SITLUE5.bat` | Software-in-the-loop | RflySimUE5 workflow |
| `UUVModel_HITLUE5.bat` | Hardware-in-the-loop | RflySimUE5 workflow |

## RflySim v4.12 Compatibility

The scripts use the RflySim v4.12 CopterSim argument order. For the released UUV model, the third CopterSim argument after the vehicle index is the 3D vehicle class ID, not the former UDP start port.

The UUV model uses:

```bat
SET /a CLASS_3D_ID=600
set DLLModel=UUVModel
SET UE4_MAP=FluxIslandMap
SET UDPSIMMODE=Mavlink_Full
SET /a IsSysID=0
```

The CopterSim launch command follows the structure:

```bat
CopterSim.exe 1 %cntr% %CLASS_3D_ID% %DLLModel% %SimMode% %UE4_MAP% %IS_BROADCAST% %PosXX% %PosYY% %ORIGIN_YAW% ... %UDPSIMMODE% %IsSysID% %ORIGIN_POS_Z%,%ORIGIN_ROLL%,%ORIGIN_PITCH%
```

Do not replace `%CLASS_3D_ID%` with `%portNum%` in current RflySim releases. Passing a UDP port such as `20100` in the class-ID position can make CopterSim and the renderer initialize an incompatible vehicle type, which may lead to invalid states or `NaN` values.

## Initial Pose

The scripts expose the shared initial pose parameters:

```bat
SET /a ORIGIN_POS_X=10
SET /a ORIGIN_POS_Y=4
SET /a ORIGIN_POS_Z=5
SET /a ORIGIN_YAW=0
SET /a ORIGIN_ROLL=0
SET /a ORIGIN_PITCH=0
```

For multi-vehicle runs, the scripts arrange vehicles in an XY grid using `VEHICLE_INTERVAL`; the same initial `Z,roll,pitch` tuple is applied to each vehicle.

## HIL Notes

HIL scripts read the available COM ports using `GetComList.exe` and use the selected COM number in the CopterSim command. The class-ID argument remains `CLASS_3D_ID=600`; the COM port is passed later in the command, where the current CopterSim interface expects the Pixhawk serial identifier.

## UE4 and UE5 Notes

The UE5 scripts start:

```bat
%PSP_PATH%\RflySimUE5\RflySim3D.exe -key=I1
```

The UE4 scripts retain the classic:

```bat
%PSP_PATH%\RflySim3D\RflySim3D.exe
```

If a local installation uses different map assets, update only `UE4_MAP` and the renderer path. Keep the CopterSim argument order unchanged.
