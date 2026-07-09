# Fusion Localization

This directory contains the ROS-side localization and fusion scripts used with the simulated UUV sensor streams.

| File | Purpose |
|---|---|
| `oneKeyScript.sh` | Starts ROS, RViz, VINS-Fusion, rope localization, relocalization, path fusion, and control tabs |
| `UUVAtt_server.py` | Remote-control and sensor-interface server for the UUV workflow |
| `ropeInfo_generator.py` | Generates rope/cable relative-position information |
| `relocate.py` | Bridges VINS-Fusion output and relocalization logic |
| `path_fusion.py` | Fuses VINS and rope/cable pose information |
| `kf.py` | Kalman filter used by the path-fusion script |
| `Config.json` | Camera configuration used by the image-capture side |

## Setup Notes

1. Install ROS and VINS-Fusion in the ROS-side environment.
2. Copy the RflySim SDK Python interface into the same environment.
3. Update the `dir1` path in `oneKeyScript.sh` if the working directory differs from the released path.
4. Start the RflySim UUV simulation before running the fusion workflow.

Run:

```bash
bash oneKeyScript.sh
```

The script assumes that the VINS-Fusion workspace has already been built and sourced.
