#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

## GLOBAL VARS ##
J_SCOOP_YAW = 5
J_HAND_YAW = 4
J_DIST_PITCH = 3
J_PROX_PITCH = 2
J_SHOU_PITCH = 1
J_SHOU_YAW = 0
J_GRINDER = 5

X_SHOU = 0.79
Y_SHOU = 0.175
HAND_Y_OFFSET = 0.0249979319838

SCOOP_OFFSET = 0.215
GRINDER_OFFSET = 0.16

# Distance between scoop center of mass and lower blade
SCOOP_HEIGHT = 0.076

DEFAULT_GROUND_HEIGHT = -0.155

X_DELIV = 0.2
Y_DELIV = 0.2
Z_DELIV = 1.2
SHOU_YAW_DELIV = 0.4439

GUARD_FILTER_AV_WIDTH = 10
# Multiply the slope on the first 10 ticks of the guarded move by this coeff to obtain threshold
GUARD_MAX_SLOPE_BEFORE_CONTACT_COEFF = 5
TRAJ_PUB_RATE = 10
NB_ARM_LINKS = 7

# Distance between center or mass of the scoop and center of rotation in l_wrist
ROT_RADIUS = 0.36

# Distance between wrist center of mass and scoop center of mass
# Component parallel to ground
WRIST_SCOOP_PARAL = 0.2
# Component perperdicular to ground
WRIST_SCOOP_PERP = 0.3

# Radii in dig_circular
R_PARALLEL_TRUE = 0.46
R_PARALLEL_FALSE = 0.25

# Radii in dig_circular for actions
R_PARALLEL_TRUE_A = 0.46
R_PARALLEL_FALSE_A = 0.10
