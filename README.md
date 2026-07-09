# UnderWaterPlatform

RflySim-compatible underwater vehicle simulation artifact for the paper:

**A Physics-Based Simulation Framework for Underwater Vehicle Dynamics and Multi-Sensor Perception**

This repository packages the released UUV dynamics artifact, SIL/HIL launch scripts, sensor configuration files, VINS-Fusion interface settings, fusion-localization code, control demos, and selected experiment logs used to support the manuscript revision.

- Project video: [Bilibili demo](https://www.bilibili.com/video/BV1h8q5BSEo7/)
- Public repository URL: [https://github.com/RflySim/UnderWaterPlatform.git](https://github.com/RflySim/UnderWaterPlatform.git)
- Platform documentation: [https://www.rflysim.com](https://www.rflysim.com)

## Preview

The figures below summarize the released SIL/HIL workflow, distributed simulation interface, experimental setup, and representative motion-behavior validation results.

<p align="center">
  <img src="media/sil_hil_architecture.png" alt="SIL and HIL simulation architecture" width="900">
</p>

<p align="center">
  <img src="media/distributed_simulation_architecture.png" alt="Distributed simulation communication architecture" width="360">
  <img src="media/sil_hil_environment.png" alt="SIL and HIL experimental environment" width="520">
</p>

<p align="center">
  <img src="media/motion_behavior_validation.png" alt="Motion-behavior validation trajectories" width="560">
</p>

## What Is Included

| Path | Contents | Purpose |
|---|---|---|
| `UUV/` | Compiled UUV dynamics DLL, PX4/Pixhawk parameter and firmware files, camera configuration, and SIL/HIL launch scripts | Start the RflySim-compatible UUV simulation in software-in-the-loop or hardware-in-the-loop mode |
| `Demo/` | Python examples for attitude/path control and camera capture | Provide minimal runnable examples for control and perception data access |
| `FusionLocation/` | ROS-side fusion-localization scripts, rope/relative-position helper, relocalization bridge, and Kalman-filter fusion code | Reproduce the visual-inertial and cable-based localization workflow |
| `euroc_uuv/` | Camera and VINS-Fusion YAML configuration files | Configure VINS-Fusion for the simulated UUV stereo/mono image and IMU streams |
| `Data/` | CSV logs for SIL/HIL and localization experiments | Support offline inspection of trajectory, attitude, velocity, IMU, GPS-compatible, and fusion outputs |
| `docs/` | Reproducibility, configuration, and data documentation | Explain how the artifact maps to the experiments and reviewer-requested release information |
| `media/` | README preview figures exported from the manuscript figures | Provide a quick visual overview of the platform workflow and validation artifacts |

## Main Capabilities

- RflySim-compatible UUV dynamics execution through `UUVModel.dll`.
- Software-in-the-loop (SIL) and hardware-in-the-loop (HIL) launch paths for UE4 and UE5 workflows.
- PX4/Pixhawk-oriented HITL setup using the released parameter and firmware files.
- Configurable visual sensor output through `Config.json`, including RGB, grayscale, and depth streams.
- Stereo camera and IMU interface configuration for VINS-Fusion.
- Fusion-localization workflow combining VINS relocalization, cable/rope relative-position information, and Kalman-filter path fusion.
- Selected CSV logs for SIL/HIL comparison and localization evaluation.

## Tested Environment

The manuscript experiments report the following environment:

- RflySim v4.12 with CopterSim, QGroundControl, PX4/Pixhawk support, and RflySimUE5.
- Unreal Engine 5.2 cooked assets.
- MATLAB/Simulink R2024b for model development and code generation.
- Python 3.12 for the released Python-side workflow.
- ROS and VINS-Fusion for the fusion-localization experiment.
- Windows 11 workstation with Intel Core i7-11700F CPU, 16 GB RAM, and NVIDIA GeForce RTX 3070 GPU with 8 GB VRAM.

The repository does not mirror third-party platforms such as RflySim, PX4, QGroundControl, ROS, or VINS-Fusion. Install those dependencies from their official sources and keep their license terms.

## Quick Start: SIL

1. Install and configure RflySim on the Windows host.
2. Open a terminal with administrator privileges.
3. Run one of the SIL launch scripts:

```bat
UUV\UUVModel_SITL.bat
UUV\UUVModel_SITLUE5.bat
```

4. When prompted, input `1` to create the vehicle instance.
5. Run a demo script after CopterSim, QGroundControl, and the 3D renderer are ready:

```bat
python Demo\UUVAttCtrlPath.py
python Demo\UUVAttCtrlCamera.py
```

## Quick Start: HIL

1. Flash the compatible firmware in `UUV/px4_fmu-v6c_default.px4` to the target PX4/Pixhawk hardware, or build firmware for the target hardware following the PX4/RflySim workflow.
2. Connect the flight controller to the Windows simulation host by USB.
3. Open a terminal with administrator privileges.
4. Run one of the HIL launch scripts:

```bat
UUV\UUVModel_HITL.bat
UUV\UUVModel_HITLUE5.bat
```

5. Input the COM port number shown by the script.
6. Run the demo or fusion-localization workflow after the simulation environment is ready.

## Fusion Localization Workflow

The fusion-localization workflow uses RflySim image/IMU streams, VINS-Fusion, rope/cable relative-position information, and Kalman-filter path fusion.

1. Prepare the ROS and VINS-Fusion environment.
2. Copy `FusionLocation/` to the ROS-side workspace or virtual machine.
3. Copy the RflySim SDK Python interface to the same environment and refresh the SDK path after platform updates.
4. Copy the YAML files in `euroc_uuv/` into the VINS-Fusion configuration directory.
5. Start the simulation through the UE5 SIL or HIL script.
6. Run:

```bash
cd FusionLocation
bash oneKeyScript.sh
```

See [docs/REPRODUCIBILITY.md](docs/REPRODUCIBILITY.md) for the staged reproduction workflow.

## Sensor Configuration

The main camera configuration files are:

- `UUV/Config.json` for the simulation-side visual sensors.
- `FusionLocation/Config.json` for the image-capture side of the fusion workflow.

The `TypeID` field selects the visual stream type:

| `TypeID` | Output |
|---:|---|
| `1` | RGB image |
| `2` | Grayscale image |
| `3` | Depth image |

Other important fields include image resolution, capture frequency, camera field of view, UDP port, sensor position, and sensor attitude. See [docs/CONFIGURATION.md](docs/CONFIGURATION.md) for details.

## Data Logs

The `Data/` directory contains selected CSV logs:

- `Data/sil_hil/sil/` and `Data/sil_hil/hil/`: paired SIL/HIL system logs.
- `Data/location1/`, `Data/location2/`, and `Data/location3/`: localization experiment outputs.
- `Data/Data/`: legacy copy of SIL/HIL data kept for compatibility with earlier scripts.

See [docs/DATASETS.md](docs/DATASETS.md) for file naming conventions and column-level guidance.

## Reproducibility Checklist

The released artifact includes:

- UUV dynamics binary artifact: `UUV/UUVModel.dll`.
- PX4/Pixhawk parameter and firmware files: `UUV/UUV.params`, `UUV/px4_fmu-v6c_default.px4`.
- SIL/HIL launch scripts for UE4 and UE5.
- Camera and sensor stream configuration files.
- VINS-Fusion camera/IMU YAML configurations.
- Control, image-capture, relocalization, rope-position, and fusion scripts.
- Selected CSV logs for SIL/HIL and localization experiments.
- Documentation that maps repository files to the manuscript experiments.

The public SOLAQUA real-UUV dataset used for the short-horizon replay validation is an external public dataset cited in the manuscript and is not mirrored in this repository.

## Citation

If this artifact is useful for your research, please cite the associated manuscript:

```text
X. Dai, Y. Yang, and Y. Chen, "A Physics-Based Simulation Framework for Underwater Vehicle Dynamics and Multi-Sensor Perception," submitted.
```

A machine-readable citation stub is provided in `CITATION.cff` and should be updated after publication.

## License

The released source files, configuration files, documentation, and examples in this repository are provided under the MIT License. Third-party platforms and tools, including RflySim, PX4, QGroundControl, ROS, VINS-Fusion, MATLAB/Simulink, and Unreal Engine assets, remain subject to their own license terms.

## Notes

- Run the Windows launch scripts with administrator privileges.
- Keep `UUVModel.dll` in the same directory as the launch scripts.
- The Python scripts require the RflySim Python API modules such as `PX4MavCtrlV4`, `UE4CtrlAPI`, `VisionCaptureApi`, and `ReqCopterSim`.
- ROS message packages and VINS-Fusion are installed separately and are not Python `pip` packages.
