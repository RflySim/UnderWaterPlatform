# Reproducibility Guide

This document describes how the released artifact maps to the experiments and release information in the manuscript.

## Artifact Scope

The repository provides:

- UUV dynamics binary artifact generated from the Simulink-based model.
- RflySim-compatible SIL/HIL launch scripts.
- PX4/Pixhawk parameter and firmware files for the released hardware target.
- RGB/grayscale/depth camera configuration files.
- VINS-Fusion camera/IMU configuration files.
- Python scripts for control, image capture, relocalization, rope/cable relative localization, and path fusion.
- Selected CSV logs for SIL/HIL and localization experiments.

The repository does not include the full RflySim platform, Unreal Engine, PX4 source tree, QGroundControl, ROS, VINS-Fusion source code, or the external public SOLAQUA dataset.

## Reported Experiment Environment

The manuscript reports the following environment:

| Item | Version or specification |
|---|---|
| Simulation platform | RflySim v4.12 |
| Simulation components | CopterSim, QGroundControl, PX4/Pixhawk support, RflySimUE5 |
| Rendering asset base | Unreal Engine 5.2 cooked assets |
| Model development | MATLAB/Simulink R2024b |
| Python environment | Python 3.12 |
| Fusion-localization stack | ROS and VINS-Fusion |
| CPU | Intel Core i7-11700F |
| RAM | 16 GB |
| GPU | NVIDIA GeForce RTX 3070, 8 GB VRAM |
| Operating system | Windows 11 for the simulation host |

## Reproduction Levels

Different users may reproduce different levels depending on available hardware and platform licenses.

| Level | Goal | Required assets |
|---|---|---|
| L0 | Inspect released logs and configuration files | This repository only |
| L1 | Run Python-side demo scripts against a running simulator | RflySim, RflySimSDK, Python 3.12 |
| L2 | Run software-in-the-loop simulation | RflySim, CopterSim, renderer, `UUVModel.dll`, SIL batch scripts |
| L3 | Run hardware-in-the-loop simulation | L2 assets plus PX4/Pixhawk hardware, firmware, USB serial connection |
| L4 | Run fusion localization | L2 or L3, ROS, VINS-Fusion, `euroc_uuv/` configs, `FusionLocation/` scripts |

## SIL Procedure

1. Install RflySim on the Windows host.
2. Place `UUVModel.dll` in `UUV/` together with the launch scripts.
3. Run one of:

```bat
UUV\UUVModel_SITL.bat
UUV\UUVModel_SITLUE5.bat
```

4. Start a demo after the simulator and renderer are ready:

```bat
python Demo\UUVAttCtrlPath.py
python Demo\UUVAttCtrlCamera.py
```

5. Inspect generated CSV logs or image outputs as needed.

## HIL Procedure

1. Flash `UUV/px4_fmu-v6c_default.px4` to compatible PX4 FMU v6c hardware, or build the matching firmware for another target.
2. Connect the autopilot hardware by USB.
3. Run one of:

```bat
UUV\UUVModel_HITL.bat
UUV\UUVModel_HITLUE5.bat
```

4. Input the COM port shown in the script prompt.
5. Run the control or fusion-localization scripts after the HIL environment is ready.

## Fusion-Localization Procedure

1. Install ROS and VINS-Fusion in the ROS-side environment.
2. Copy `euroc_uuv/` YAML files to the VINS-Fusion configuration path.
3. Copy `FusionLocation/` and the RflySimSDK Python interface into the ROS-side environment.
4. Update the `dir1` variable in `FusionLocation/oneKeyScript.sh` if the working directory differs from the released path.
5. Start the RflySim UUV simulation.
6. Run:

```bash
cd FusionLocation
bash oneKeyScript.sh
```

The script starts ROS, RViz, VINS-Fusion, rope/cable relative localization, relocalization, path fusion, and the control server in separate terminal tabs.

## Expected Outputs

Typical outputs include:

- Vehicle state logs: `uav*.csv` and `true*.csv`.
- SIL/HIL comparison logs under `Data/sil_hil/`.
- Localization outputs: `vins.csv`, `rope.csv`, `relocate.csv`, and `fusion.csv`.
- Camera images if `Demo/UUVAttCtrlCamera.py` is used.
- ROS topics under `/rflysim/...` during the fusion-localization workflow.

## External Public Dataset

The public-data replay experiment in the manuscript uses the external SOLAQUA real-UUV dataset. The dataset is cited in the manuscript and is not mirrored in this repository. This repository instead provides the platform artifact, configuration files, selected simulation logs, and fusion-localization scripts needed to reproduce the released simulation workflow.

## Verification Checklist

Before reporting results, check:

- The selected launch script matches SIL or HIL mode.
- The selected renderer version matches the intended UE4 or UE5 workflow.
- `Config.json` uses the intended `TypeID`, resolution, frequency, and UDP ports.
- VINS-Fusion is using the YAML files in `euroc_uuv/`.
- The RflySimSDK Python path is refreshed after platform updates.
- The generated CSV logs contain nonzero timestamps and expected state channels.
- The final LaTeX manuscript or report is compiled after bibliography updates if citation screenshots are needed.
