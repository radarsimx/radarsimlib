# RadarSimLib

<img src="./assets/radarsimlib.svg" alt="logo" width="200"/>

A **Radar** **Sim**ulator for **Py**thon

RadarSimPy is a powerful and versatile Python-based Radar Simulator that models radar transceivers and simulates baseband data from point targets and 3D models. Its signal processing tools offer range/Doppler processing, direction of arrival estimation, and beamforming using various cutting-edge techniques, and you can even characterize radar detection using Swerling’s models. Whether you’re a beginner or an advanced user, RadarSimPy is the perfect tool for anyone looking to develop new radar technologies or expand their knowledge of radar systems.

## Key Features

- ### Radar Modeling

  - Radar transceiver modeling
  - Arbitrary waveform
  - Phase noise
  - Phase/amplitude modulation
  - Fast-time/slow-time modulation

- ### Simulation

  - Simulation of radar baseband data from point targets
  - Simulation of interference
  - Simulation of radar baseband data from 3D modeled objects/environment (**`#raytracing`**)
  - Simulation of target's RCS (**`#raytracing`**)
  - Simulation of LiDAR point cloud from 3D modeled objects/environment (**`#raytracing`**)

## Dependence

- [Visual C++ Runtime](https://aka.ms/vs/16/release/vc_redist.x64.exe/) (`Windows`)

## Installation

To use the module, please put the radarsimpy folder within your project folder as shown below.

## Coordinate Systems

- ### Scene Coordinate

  - axis (m): `[x, y, z]`
  - phi (deg): angle on the x-y plane. 0 deg is the positive x-axis, 90 deg is the positive y-axis
  - theta (deg): angle on the z-x plane. 0 deg is the positive z-axis, 90 deg is the x-y plane
  - azimuth (deg): azimuth -90 ~ 90 deg equal to phi -90 ~ 90 deg
  - elevation (deg): elevation -90 ~ 90 deg equal to theta 180 ~ 0 deg

- ### Object's Local Coordinate

  - axis (m): `[x, y, z]`
  - yaw (deg): rotation along the z-axis. Positive yaw rotates the object from the positive x-axis to the positive y-axis
  - pitch (deg): rotation along the y-axis. Positive pitch rotates the object from the positive x-axis to the positive z-axis
  - roll (deg): rotation along the x-axis. Positive roll rotates the object from the positive z-axis to the negative y-axis
  - origin (m): `[x, y, z]`
  - rotation (deg): `[yaw, pitch, roll]`
  - rotation (deg/s): rate `[yaw rate, pitch rate, roll rate]`

## Usage Examples

The source files of these Jupyter notebooks are available [here](https://github.com/radarsimx/radarsimnb).

- ### Radar modeling and point target simulation

  - [Doppler radar](https://radarsimx.com/2019/05/16/doppler-radar/)
  - [FMCW radar](https://radarsimx.com/2018/10/11/fmcw-radar/)
  - [TDM MIMO FMCW radar](https://radarsimx.com/2019/04/07/tdm-mimo-fmcw-radar/)
  - [PMCW radar](https://radarsimx.com/2019/05/24/pmcw-radar/)
  - [Arbitrary waveform](https://radarsimx.com/2021/05/10/arbitrary-waveform/)
  - [Phase noise](https://radarsimx.com/2021/01/13/phase-noise/)
  - [CFAR](https://radarsimx.com/2021/01/10/cfar/)
  - [DoA estimation](https://radarsimx.com/2022/12/12/doa-estimation/)
  - [Interference](https://radarsimx.com/2023/01/13/interference/)

- ### Radar modeling and 3D scene simulation with raytracing

  - [Imaging radar](https://radarsimx.com/2022/12/02/imaging-radar/)
  - [FMCW radar with a corner reflector](https://radarsimx.com/2021/05/10/fmcw-radar-with-a-corner-reflector/)
  - [FMCW radar with a plate](https://radarsimx.com/2021/05/10/fmcw-radar-with-a-plate/)
  - [FMCW radar with a car](https://radarsimx.com/2021/05/10/fmcw-radar-with-a-car/)
  - [Doppler of a turbine](https://radarsimx.com/2021/05/10/doppler-of-a-turbine/)
  - [Micro-Doppler](https://radarsimx.com/2021/05/10/micro-doppler/)
  - [Multi-path effect](https://radarsimx.com/2021/05/10/multi-path-effect/)

- ### 3D modeled target's RCS simulation

  - [Corner reflector RCS](https://radarsimx.com/2021/05/10/corner-reflector-rcs/)
  - [Plate RCS](https://radarsimx.com/2021/05/10/plate-rcs/)
  - [Car RCS](https://radarsimx.com/2021/05/10/car-rcs/)

- ### LiDAR point cloud

  - [LIDAR point cloud](https://radarsimx.com/2020/02/05/lidar-point-cloud/)

- ### Receiver characterization

  - [Receiver operating characteristic (ROC)](https://radarsimx.com/2019/10/06/receiver-operating-characteristic/)

## Build

**Building `radarsimlib` requires to access the source code of `radarsimcpp`. If you don't have access to `radarsimcpp`, please download the pre-built module instead**

- Windows CPU

```batch
build_win.bat
```

- Windows CUDA

```batch
build_win_cuda.bat
```
