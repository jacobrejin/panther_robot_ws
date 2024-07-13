## Panther Fusion Package

This package is used to help fuse various sensor data.
The pacakge has multiple parameter yaml file, each file can be used for a different sensor fusion task.
Below is a list of the parameter files and their descriptions:

### IMP: In order to use the panther fusion package, specifically for the UWB sensor data fusion, Please make sure you have the correct transforms and pose data available. You can read the readme file for the `panther_transforms` package to get the correct transforms up and running.


## 1. ekf_panther_uwb_1

This is used to fuse the UWB data with the IMU and the Odometer readings.
The yaml file is a copy of the official hussarian pather package (ekf node for panther which produces the odom->baselink tf)
Additional Pose entry was created to add the UWB data to the fusion.