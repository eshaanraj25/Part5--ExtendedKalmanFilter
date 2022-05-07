# Project: Extended Kalman Filter

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
In this project we utilize a Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. A linear kalman filter has been used for working with Lidar measurements and an extended kalman filter has been used to deal with the radar measurments. The estimations of the two filters are combined to yield a complete state estimation system.

Getting Started
---

The project has been developed on a Linux machine with Python 3.6. The system was provided by Udacity for this particular project.

## Prerequisites
Following are the dependencies:

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's Term 2 Simulator. [Link](https://github.com/udacity/self-driving-car-sim/releases)

To install the dependencies, use the script [install-linux.sh](install-linux.sh)

Dataset
---
Synthetic data provided by Udacity is used for the project. The data is present in the [data](data) directory. It consists of measurements in a txt file format.

The simulator provides us with two datasets. The difference is in the starting measurment: The first starts with a Lidar measurement and the second starts with a Radar measurement.

Using the application
---

## Build
Use the commands to build the project:

```bash
mkdir build && cd build
cmake .. && make
```

## Run
After building the project, run the project:

```bash
cd build
./ExtendedKF
```

## Results

- [Video](video.mp4) for the same

- [Youtube](https://youtu.be/QzgAMg0PCDs) video for the same

- The final error for Dataset One are:

```
RMSE
X: 0.0974
Y: 0.0855
VX: 0.4517
VY: 0.4404
```

- The final error for Dataset Two are:

```
RMSE
X: 0.0726
Y: 0.0965
VX: 0.4219
VY: 0.4937
```