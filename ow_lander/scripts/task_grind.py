#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper
from ow_lander import constants

import argparse

from distutils.util import strtobool

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="A grinder is used excavate the terrain.")
parser.add_argument('x_start', type=float, nargs='?', default=1.65,
  help="X-coordinate of grinding starting point")
parser.add_argument('y_start', type=float, nargs='?', default=0.0,
  help="Y-coordinate of grinding starting point")
parser.add_argument('depth', type=float, nargs='?', default=0.05,
  help="Desired excavation depth")
parser.add_argument('length', type=float, nargs='?', default=0.6,
  help="Desired trench length")
parser.add_argument('parallel', type=strtobool, nargs='?', default = True,
  help="If True, resulting trench is parallel to arm. If False, " \
  "perpendicular to arm")
parser.add_argument('ground_position', type=float, nargs='?',
  default=constants.DEFAULT_GROUND_HEIGHT,
  help="Z-coordinate of ground level in base_link frame")
args = parser.parse_args()

node_helper.call_single_use_action_client(actions.TaskGrindServer, **vars(args))
