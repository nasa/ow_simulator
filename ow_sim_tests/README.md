# ow_sim_tests
A package **ow_sim_tests** that hosts OceanWATERS integration tests.

- [Usage](#usage)
- [Plugins](#plugins)
## Usage

* To test arm operations run the following command:
```bash
rostest ow_sim_tests arm_check.test
```

* To test simulation fps run the following command:
```bash
rostest ow_sim_tests fps_monitor.test
```

## Plugins

### FPS Monitor

A plugin that monitors average frame rate as obsered from the
 active user camera and publishs the value to a rostopic "/monitor\_fps/avg\_fps".  

The plugin also comes with a configurable rostest that lets the user set the
 minimum acceptable average frame rate over a given period of time.
