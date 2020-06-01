#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

## GLOBAL VARS ##
J_SCOOP_YAW = 5
J_HAND_YAW = 4
J_DIST_PITCH = 3
J_PROX_PITCH = 2
J_SHOU_PITCH = 1
J_SHOU_YAW = 0

X_SHOU = 0.79
Y_SHOU = 0.175
HAND_Y_OFFSET = 0.0249979319838
GROUND_POSITION = -0.175
SCOOP_OFFSET = 0.215

SAW_OFFSET = 0.8

X_DELIV = 0.2
Y_DELIV = 0.2
Z_DELIV = 1.2
SHOU_YAW_DELIV = 0.4439

GUARD_FILTER_AV_WIDTH = 10
# Multiply the slope on the first 10 ticks of the guarded move by this coeff to obtain threshold
GUARD_MAX_SLOPE_BEFORE_CONTACT_COEFF = 5
TRAJ_PUB_RATE = 10
NB_ARM_LINKS = 6
