# Data Description

The `Data/` directory contains selected CSV logs from SIL/HIL and localization experiments. Files are stored without headers to keep compatibility with the original scripts.

## Directory Layout

| Path | Description |
|---|---|
| `Data/sil_hil/sil/` | Software-in-the-loop logs |
| `Data/sil_hil/hil/` | Hardware-in-the-loop logs |
| `Data/location1/` | Localization experiment run 1 |
| `Data/location2/` | Localization experiment run 2 |
| `Data/location3/` | Localization experiment run 3 |
| `Data/Data/sil/`, `Data/Data/hil/` | Legacy copy of SIL/HIL logs kept for compatibility |

## Common State Logs

| Pattern | Typical content |
|---|---|
| `uavTimeStmp.csv`, `trueTimeStmp.csv` | Estimated and simulated timestamps |
| `uavAngEular.csv`, `trueAngEular.csv` | Estimated and simulated Euler angles |
| `uavAngRate.csv`, `trueAngRate.csv` | Estimated and simulated angular rates |
| `uavAngQuatern.csv`, `trueAngQuatern.csv` | Estimated and simulated quaternion components in exported order |
| `uavPosNED.csv`, `truePosNED.csv` | Estimated and simulated local position in NED-related coordinates |
| `uavVelNED.csv`, `trueVelNED.csv` | Estimated and simulated velocity in NED-related coordinates |
| `uavAccB.csv`, `trueAccB.csv` | Estimated and simulated body-frame acceleration |
| `uavMotorRPMS.csv`, `trueMotorRPMS.csv` | Estimated and simulated actuator/motor output channels |
| `uavGyro.csv` | Gyroscope output |
| `uavMag.csv` | Magnetometer output |
| `uavVibr.csv` | Vibration indicators |
| `uavPosGPS.csv` | GPS-compatible/global-position output vector exported by the RflySim API |
| `uavPosGPSHome.csv` | Home/reference GPS-compatible position |
| `uavGlobalPos.csv` | Global-position vector transformed by the RflySim API |

## Localization Logs

| File | Typical content |
|---|---|
| `vins.csv` | VINS-Fusion pose output |
| `rope.csv` | Rope/cable relative-position pose output |
| `relocate.csv` | Relocalized pose output |
| `fusion.csv` | Kalman-filter fusion pose output |

The localization pose files use seven columns in the released logs: three position components followed by four quaternion components in the exported order.

## Reading Logs

The files can be loaded with NumPy:

```python
import numpy as np

path = "Data/location1/fusion.csv"
fusion = np.loadtxt(path, delimiter=",")
print(fusion.shape)
```

When comparing SIL and HIL logs, align samples by timestamp before computing trajectory, attitude, or velocity differences.

## Data Use Notes

- The first row may contain zeros from script initialization.
- Some scripts append values at the RflySim API update frequency; check timestamps before assuming a fixed sample rate.
- Coordinate-frame interpretation should follow the corresponding RflySim API field names and the paper's SIL/HIL data-flow description.
- The external SOLAQUA real-UUV dataset used in the manuscript replay experiment is not mirrored here.
