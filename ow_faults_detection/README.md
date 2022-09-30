The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
Exploration Research and Simulation can be found in README.md in the
root directory of this repository.

ow_faults_detection
===================

This package contains infrastructure for fault detection. The
detection is relayed through a series of /fault rostopics.

## `faults_detection` node

This node is the source of fault detection in OCEANWATERS. It checks
telemetry from different systems for off-nominal values to determine
whether or not a fault has occured. This package is also what other
systems, such as autonomy, use to create more informative detection.

## Detecting Faults

Faults are detected in five lander systems: antenna, arm, the
force/torque sensor, power, and camera.  The fault state of these
systems is reflected in the telemetry messages listed below, which
include a system fault message that aggregates the states.

Antenna Faults: `/faults/pt_faults_status`
Arm Faults: `/faults/arm_faults_status`
Force-Torque Sensor Faults: `/faults/pt_faults_status`
Power Faults: `/faults/power_faults_status`
Camera Faults: `/faults/camera_faults_status`
System Faults: `/faults/system_faults_status`

