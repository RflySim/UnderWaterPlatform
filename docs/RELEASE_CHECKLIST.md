# Release Checklist

This checklist records how the repository addresses the artifact and reproducibility items described in the manuscript revision.

| Item | Repository location |
|---|---|
| Public source-code entry point | `readme.md`, `readme_zh.md` |
| Video link | `readme.md`, `readme_zh.md` |
| Repository license | `LICENSE` |
| README preview media | `media/` |
| UUV dynamics runtime artifact | `UUV/UUVModel.dll` |
| SIL launch scripts | `UUV/UUVModel_SITL.bat`, `UUV/UUVModel_SITLUE5.bat` |
| HIL launch scripts | `UUV/UUVModel_HITL.bat`, `UUV/UUVModel_HITLUE5.bat` |
| PX4/Pixhawk parameter file | `UUV/UUV.params` |
| PX4/Pixhawk firmware artifact | `UUV/px4_fmu-v6c_default.px4` |
| Camera and visual-sensor configuration | `UUV/Config.json`, `FusionLocation/Config.json` |
| VINS-Fusion camera/IMU configuration | `euroc_uuv/` |
| Control demos | `Demo/` |
| Fusion-localization scripts | `FusionLocation/` |
| SIL/HIL experiment logs | `Data/sil_hil/` |
| Localization experiment logs | `Data/location1/`, `Data/location2/`, `Data/location3/` |
| Reproducibility instructions | `docs/REPRODUCIBILITY.md` |
| Configuration reference | `docs/CONFIGURATION.md` |
| Launch-script compatibility notes | `docs/LAUNCH_SCRIPTS.md` |
| Data description | `docs/DATASETS.md` |
| Python dependency hints | `requirements.txt` |
| Citation stub | `CITATION.cff` |

Before public release, confirm that the GitHub repository URL and the video URL remain accessible.
