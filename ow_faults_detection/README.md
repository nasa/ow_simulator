The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
Exploration Research and Simulation can be found in README.md in the
root directory of this repository.

ow_faults_detection
===================

This package contains infrastructure for fault detection. The
detection is relayed through a series of rostopics.

## `faults_detection` node

This node is the source of fault detection in OCEANWATERS. It checks
telemetry from different systems for off-nominal values to determine
whether or not a fault has occured. This package is also what other
systems, such as autonomy, use to create more informative detection.

## Detecting Faults

Faults are detected in five lander systems: antenna, arm, power, and camera.  
The fault state of these systems is reflected in the telemetry messages 
listed below, which include a system fault message that aggregates the states.

Antenna Faults: `/pan_tilt_faults_status`  
Arm Faults:     `/arm_faults_status`  
Camera Faults:  `/camera_faults_status`  
Power Faults:   `/power_faults_status`  
System Faults:  `/system_faults_status`

These messages are housed in the `owl_msgs` node.
