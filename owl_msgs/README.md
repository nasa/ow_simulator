The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
Exploration Research and Simulation can be found in README.md in the
root directory of this repository.

owl_msgs
===================

This package contains OceanWATERS and OWLAT telemetry. The messages
defined here are currently used for fault detection. The
detection is relayed through a series of rostopics.


## Detecting Faults

Faults are detected in five lander systems: antenna, arm, the
force/torque sensor, power, and camera.  The fault state of these
systems is reflected in the telemetry messages listed below, which
include a system fault message that aggregates the states.



