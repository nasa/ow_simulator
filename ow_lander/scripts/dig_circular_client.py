#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper
from ow_lander.constants import DEFAULT_GROUND_HEIGHT

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Removes tailing following a grind with a circular scooping " \
              "motion.")
parser.add_argument('x_start', type=float, nargs='?', default=1.65,
  help='x-coordinate of trenching start point')
parser.add_argument('y_start', type=float, nargs='?', default=0.0,
  help='y-coordinate of trenching start point')
parser.add_argument('depth', type=float, nargs='?', default=0.01,
  help='Desired scooping depth')
parser.add_argument('parallel', type=lambda x: eval(x[0].upper() + x[1:].lower()), nargs='?', default= True,
  help='If True, resulting trench is parallel to arm. If False, perpendicular to arm')
parser.add_argument('ground_position', type=float, nargs='?',
    default=DEFAULT_GROUND_HEIGHT,
    help='z-coordinate of ground level in base_link frame')
args = parser.parse_args()

node_helper.call_single_use_action_client(actions.DigCircularServer,
  **vars(args))
