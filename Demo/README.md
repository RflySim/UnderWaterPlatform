# Demo Scripts

This directory contains minimal Python examples for interacting with the running UUV simulation.

| File | Purpose |
|---|---|
| `UUVAttCtrlPath.py` | Sends attitude/thrust commands and records selected vehicle states to CSV arrays |
| `UUVAttCtrlCamera.py` | Requests configured image streams from the renderer and displays/saves captured images |

## Requirements

- RflySim must be running in SIL or HIL mode.
- The RflySim Python SDK must be on the Python path.
- Python 3.12 and the packages listed in `../requirements.txt` are recommended for the released workflow.

## Typical Usage

Start the simulation first, then run:

```bat
python Demo\UUVAttCtrlPath.py
python Demo\UUVAttCtrlCamera.py
```

`UUVAttCtrlCamera.py` reads the active camera configuration from `Config.json` through the RflySim vision API.
