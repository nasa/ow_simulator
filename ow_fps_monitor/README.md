The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

# FPS Monitor

A plugin that monitors average frame rate as obsered from the
 active user camera and publishs the value to a rostopic "/monitor\_fps/avg\_fps".  

The plugin also comes with a configurable rostest that lets the user set the
 minimum acceptable average frame rate over a given period of time.

## usage

To launch the preconfigured rostest use:  
```bash
$ rostest ow_fps_monitor default.test
```