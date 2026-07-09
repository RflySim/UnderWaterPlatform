# UUV Runtime Artifact

This directory contains the UUV runtime files used by the RflySim-compatible SIL/HIL workflow.

| File | Description |
|---|---|
| `UUVModel.dll` | Compiled UUV dynamics model loaded by CopterSim |
| `UUV.params` | PX4/Pixhawk parameter file used in the released HIL workflow |
| `px4_fmu-v6c_default.px4` | Firmware artifact for PX4 FMU v6c hardware |
| `Config.json` | Camera and visual-sensor stream configuration |
| `UUVModel_SITL.bat` | UE4 software-in-the-loop launch script |
| `UUVModel_HITL.bat` | UE4 hardware-in-the-loop launch script |
| `UUVModel_SITLUE5.bat` | UE5 software-in-the-loop launch script |
| `UUVModel_HITLUE5.bat` | UE5 hardware-in-the-loop launch script |

Keep `UUVModel.dll` in this directory when running the launch scripts.

Run batch scripts with administrator privileges. HIL scripts require the flight controller hardware to be connected by USB and the correct COM port to be selected.

The launch scripts have been updated for the RflySim v4.12 CopterSim argument order. In particular, the third argument after the vehicle index is `CLASS_3D_ID=600` for the UUV model, not the UDP start port. See `../docs/LAUNCH_SCRIPTS.md` for details.
