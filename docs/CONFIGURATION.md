# Configuration Reference

This document summarizes the main configuration files and runtime switches included in the artifact.

## Launch Scripts

| Script | Mode | Renderer workflow | Notes |
|---|---|---|---|
| `UUV/UUVModel_SITL.bat` | Software-in-the-loop | UE4 | Starts the software-only simulation workflow |
| `UUV/UUVModel_HITL.bat` | Hardware-in-the-loop | UE4 | Requires compatible autopilot hardware and COM port selection |
| `UUV/UUVModel_SITLUE5.bat` | Software-in-the-loop | UE5 | Starts the UE5-based workflow |
| `UUV/UUVModel_HITLUE5.bat` | Hardware-in-the-loop | UE5 | Requires compatible autopilot hardware and COM port selection |

Run the Windows batch scripts with administrator privileges.

The scripts follow the RflySim v4.12 CopterSim argument order. The UUV class ID is set by `CLASS_3D_ID=600`; the old UDP-port position must not be used as the class-ID argument. See `LAUNCH_SCRIPTS.md` for the compatibility note.

## UUV Runtime Files

| File | Description |
|---|---|
| `UUV/UUVModel.dll` | Compiled UUV dynamics model used by CopterSim |
| `UUV/UUV.params` | PX4/Pixhawk control and platform parameters for HIL experiments |
| `UUV/px4_fmu-v6c_default.px4` | Released firmware image for PX4 FMU v6c hardware |
| `UUV/Config.json` | Visual sensor configuration used by the simulation side |

## Visual Sensor Configuration

Both `UUV/Config.json` and `FusionLocation/Config.json` use the `VisionSensors` list. Each item describes one camera-like output stream.

| Field | Meaning |
|---|---|
| `SeqID` | Sensor sequence index |
| `TypeID` | Output type: `1` RGB, `2` grayscale, `3` depth |
| `TargetCopter` | Vehicle instance ID |
| `TargetMountType` | Mounting mode used by the RflySim vision API |
| `DataWidth`, `DataHeight` | Image resolution |
| `DataCheckFreq` | Capture/check frequency in Hz |
| `SendProtocol` | Transport settings, including IP and UDP port fields |
| `CameraFOV` | Camera field of view in degrees |
| `SensorPosXYZ` | Sensor position relative to the vehicle body |
| `SensorAngEular` | Sensor attitude in Euler-angle form |
| `otherParams` | Reserved or platform-specific parameters |

The released stereo configuration uses two horizontally separated sensors with ports `9999` and `9998`.

## VINS-Fusion Configuration

The `euroc_uuv/` directory contains camera and IMU configuration files adapted to the simulated UUV data stream:

| File | Purpose |
|---|---|
| `cam0_pinhole.yaml`, `cam1_pinhole.yaml` | Pinhole stereo camera models |
| `cam0_mei.yaml`, `cam1_mei.yaml` | Mei camera models |
| `euroc_mono_imu_config.yaml` | Mono + IMU VINS-Fusion configuration |
| `euroc_stereo_config.yaml` | Stereo-only VINS-Fusion configuration |
| `euroc_stereo_imu_config.yaml` | Stereo + IMU VINS-Fusion configuration |
| `rflysim_stereo_imu_config.yaml` | RflySim UUV stereo + IMU configuration used by `oneKeyScript.sh` |

## Fusion-Localization Script Settings

`FusionLocation/oneKeyScript.sh` uses:

```bash
dir1="/home/rflysim/uuv20240309"
```

Update this path to the directory where `FusionLocation/` and the RflySim SDK are installed on the ROS-side environment.

The script launches:

- `roscore`
- RViz through `vins_rviz.launch`
- VINS-Fusion node with `rflysim_stereo_imu_config.yaml`
- `ropeInfo_generator.py`
- `relocate.py`
- `path_fusion.py`
- `UUVAtt_server.py`

## Python Modules

The scripts use both standard Python packages and platform-specific modules.

Standard packages include:

- `numpy`
- `opencv-python`
- `matplotlib`
- `numpy-quaternion`

Platform or ROS packages include:

- `PX4MavCtrlV4`
- `UE4CtrlAPI`
- `VisionCaptureApi`
- `ReqCopterSim`
- `rospy`
- `message_filters`
- `nav_msgs`
- `geometry_msgs`
- `std_msgs`
- `tf`

The RflySim modules are provided by the RflySim SDK. ROS modules are provided by the ROS workspace and are not installed through `pip`.
