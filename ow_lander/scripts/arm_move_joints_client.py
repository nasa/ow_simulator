#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper

import argparse

from distutils.util import strtobool

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Move all arm joints to either absolute or relative positions.")
parser.add_argument('relative', type=strtobool, nargs='?', default='False',
  help="Joints will move to the provided positions relative to their current "
       "positions.")
parser.add_argument('j_shou_yaw', type=float, nargs='?', default=0.1,
  help="Shoulder yaw joint angle")
parser.add_argument('j_shou_pitch', type=float, nargs='?', default=0.1,
  help="Shoulder pitch joint angle")
parser.add_argument('j_prox_pitch', type=float, nargs='?', default=0.1,
  help="Proximal pitch joint angle")
parser.add_argument('j_dist_pitch', type=float, nargs='?', default=0.1,
  help="Distal pitch joint angle")
parser.add_argument('j_hand_yaw', type=float, nargs='?', default=0.1,
  help="Hand yaw joint angle")
parser.add_argument('j_scoop_yaw', type=float, nargs='?', default=0.1,
  help="Scoop yaw joint angle")
args = parser.parse_args()

angles_arg = [args.j_shou_yaw, args.j_shou_pitch, args.j_prox_pitch,
             args.j_dist_pitch, args.j_hand_yaw, args.j_scoop_yaw]

node_helper.call_single_use_action_client(actions.ArmMoveJointsServer,
  relative=args.relative, angles=angles_arg)
