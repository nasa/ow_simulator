#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander.actions import arm
from ow_lander.constants import DEFAULT_GROUND_HEIGHT
from ow_lander import node_helper

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="")
parser.add_argument('x_start', type=float, nargs='?', default=1.46,
  help='x-coordinate of trenching start point')
parser.add_argument('y_start', type=float, nargs='?', default=0.0,
  help='y-coordinate of trenching start point')
parser.add_argument('depth', type=float, nargs='?', default=0.01,
  help='Desired scooping depth')
parser.add_argument('length', type=float, nargs='?', default=0.1,
  help='Desired length of trench')
parser.add_argument('ground_position', type=float, nargs='?',
  default=DEFAULT_GROUND_HEIGHT,
  help='z-coordinate of ground level in base_link frame')
args = parser.parse_args()

node_helper.call_single_use_action_client(arm.DigLinearServer, **vars(args))
