The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

ow_faults_injection
===================

This package contains infrastructure for fault injection, i.e. the
introduction of simulated faults into the simulation.

Injected faults are implemented using ROS parameters.

## Fault injection node

The `faults` node in this package is the source of all the fault state
in OceanWATERS.  It uses a `dynamic_reconfigure::Server` so that
clients can modify fault state at runtime.

## Injecting faults

One can set faults in rqt, on the command line, or from a python
script -- see
https://github.com/nasa/ow_simulator/wiki/Fault-Injection-and-Modeling
for instructions.

Setting a fault causes it to be updated on the
`dynamic_reconfigure::Server` in the faults node such that a message
is emitted on the `/faults/update_parameters` topic and
`rqt_reconfigure` is updated with the new value.
