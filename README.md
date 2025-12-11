# Unmanned Underwater Vehicle (UUV) Control and Simulation Platform

This is a control and simulation platform for Unmanned Underwater Vehicles (UUV) used to implement various control algorithms and simulation experiments for underwater robots.

## Project Structure

```
├── Config.json         # Sensor configuration file
├── code/               # Code directory
├── data/               # Experimental data storage directory
├── image/              # Project-related image storage directory
└── model/              # UUV model file directory
```

## Directory Description

### 1. Config.json

Configuration file for defining visual sensor parameters in the system. Contains the following information:
- Sensor sequence number and type ID
- Data width, height, and sampling frequency
- Transmission protocol and IP address
- Camera Field of View (FOV)
- Sensor position and angle information

### 2. code/ Directory

Contains Python implementation files for various UUV control algorithms:

#### UUVAttCtrlUnderWaterDepth.py
Underwater depth sensor simulator and control interface. Main functions:
- Implements depth sensor simulation model including noise, bias, and wave effects
- Provides QGC-style GUI interface for real-time depth data display
- Supports data recording and saving functionality

#### UUVAttCtrlPathCircle.py
UUV circular path tracking control program. Main functions:
- Implements UUV attitude control and circular path planning
- Supports PX4 flight controller Offboard mode control
- Records UUV attitude, position, and velocity data

#### UUVAttCtrlPositionCtrl.py
UUV position controller implementation. Main functions:
- Implements position control algorithm based on NED coordinate system
- Supports target position tracking and path planning
- Achieves precise position control by combining with attitude control

#### UUVAttCtrlSonarFast.py
Sonar image processing and visualization program. Main functions:
- Implements depth map to sonar image conversion algorithm
- Provides real-time sonar image visualization interface
- Supports sonar simulation with different parameter configurations

### 3. data/ Directory

Stores data from various control experiments:

#### attitude_ctrl_data/
Attitude control experiment data, including:
- circle/: Circular path tracking data
- complex_path/: Complex path tracking data

#### depth_data/
Depth control experiment data, containing 9 groups of experimental results with different parameters (parameter1~parameter9), each group includes:
- cont_data.csv: Continuous depth data
- out_data.csv: Sampled output depth data
- parameter.txt: Experimental parameter configuration file
- time.csv: Timestamp data
- true_data.csv: True depth data

#### position_ctrl_data/
Position control experiment data, including:
- circle/: Circular trajectory control data
- square/: Square trajectory control data

#### sonar_data/
Sonar image data, containing configurations with different field of views and maximum detection distances:
- fov60max20/: 60° field of view, maximum distance 20 meters, including PNG images of original sonar, noisy sonar, blurred sonar, and blurred noisy sonar
- fov60max40/: 60° field of view, maximum distance 40 meters, including PNG images of original sonar, noisy sonar, blurred sonar, and blurred noisy sonar
- fov90max40/: 90° field of view, maximum distance 40 meters, including PNG images of original sonar, noisy sonar, blurred sonar, and blurred noisy sonar
- fov90max40_time_series_data/: Time series sonar data, containing 19 sonar images at different times (Figure_1.png to Figure_19.png)

### 4. image/ Directory

Stores project-related image files for documentation and visualization purposes.

### 5. model/ Directory

Contains UUV model files:

#### UUVModel.slx
Simulink model file defining the UUV's physical model and dynamic characteristics

#### UUVModel.dll
Compiled UUV model dynamic link library used for simulation calculations

#### UUVModel_init.m
MATLAB initialization script for loading and configuring the UUV model

#### GenerateModelDLLFile.p
Script for generating model DLL files
