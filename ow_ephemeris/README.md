The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.

ow_ephemeris
============
The `irg_open` repository contains tools for modeling an ephemeris:
 - `irg_planetary_ephemeris`
 - `IRGCelestialBodyPlugin`

While this package contains details necessary to customize the ephemeris for
OceanWATERS:
 - It downloads SPICE kernels and stores them in the `data/` directory.
 - A config file for `irg_planetary_ephemeris` that sets up a mission site on
   Europa.
 - A script that converts sunlight distance from `irg_planetary_ephemeris` into 
   lux values for our shaders.

