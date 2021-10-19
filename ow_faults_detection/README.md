The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

ow_faults_detection
=========
This package contains infrastructure for fault detection. Users can use this package to detect different lander faults, The detection is relayed through a series of /fault rostopics. 

## `faults detection` node
This node is the source of fault detection in OCEANWATERS. It compares nomimal state and commands to telemetry data to determine whether or not a fault has occured. This package is also what other nodes use to create more informative detections, such as Autonomy. 

## Detecting Faults
Currently we detect five faults: system, antenna, arm, power, and camera. To select an array of faults to inject, refer to Faults Injection. These topics are high level and mirror the fault descriptions from JPL's lander system. To see the details of each fualt message, see the /msg folder and each respective msg type. 

`System Faults:`
observe the topic /faults/system_faults_status

`Antenna Faults:`
observe the topic /faults/pt_faults_status

`Arm Faults:`
observe the topic /faults/arm_faults_status

`Power Faults:`
observe the topic /faults/power_faults_status

`Camera Faults:`
observe the topic /faults/camera_faults_status