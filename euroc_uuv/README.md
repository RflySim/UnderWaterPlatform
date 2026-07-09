# VINS-Fusion Configuration Files

This directory contains camera and IMU configuration files adapted from the EuRoC-style VINS-Fusion format for the simulated UUV data stream.

| File | Purpose |
|---|---|
| `cam0_pinhole.yaml`, `cam1_pinhole.yaml` | Stereo pinhole camera model files |
| `cam0_mei.yaml`, `cam1_mei.yaml` | Stereo Mei camera model files |
| `euroc_mono_imu_config.yaml` | Mono + IMU VINS-Fusion configuration |
| `euroc_stereo_config.yaml` | Stereo-only VINS-Fusion configuration |
| `euroc_stereo_imu_config.yaml` | Stereo + IMU VINS-Fusion configuration |
| `rflysim_stereo_imu_config.yaml` | Configuration used by the released RflySim UUV stereo + IMU workflow |

Copy the required YAML files into the VINS-Fusion configuration directory before running `FusionLocation/oneKeyScript.sh`.
