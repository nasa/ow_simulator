#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Move an arm joint to either an absolute or relative position.")
parser.add_argument('joint', type=int, nargs='?', default=0, choices=range(6),
  help="Index of joint that will be moved be moved" \
       "\n0 : shoulder yaw" \
       "\n1 : shoulder pitch" \
       "\n2 : proximal pitch" \
       "\n3 : distal pitch" \
       "\n4 : hand yaw" \
       "\n5 : scoop yaw")
parser.add_argument('angle', type=float, nargs='?', default=-0.5,
  help="Position/rotation in position in radians")
parser.add_argument('--relative', '-r', action='store_true', default=False,
  help="Joints will move to the provided positions relative to their current "
       "positions.")
args = parser.parse_args()

node_helper.call_single_use_action_client(actions.ArmMoveJointServer,
  **vars(args))
