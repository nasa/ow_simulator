#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import argparse

from ow_lander import actions
from ow_lander import node_helper
from ow_lander import constants

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="The grinder is used process the terrain for excavation.")
parser.add_argument('-x', type=float, default=1.45,
  help="X-coordinate of grinding starting point in base_link frame")
parser.add_argument('-y', type=float, default=0.0,
  help="Y-coordinate of grinding starting point in base_link frame")
parser.add_argument('-z', type=float, default=constants.DEFAULT_GROUND_HEIGHT,
  help="Estimate of ground position at point (x, y) in base_link frame")
parser.add_argument('--depth', '-d', type=float, default=0.05,
  help="Depth of grinder tip")
parser.add_argument('--length', '-l', type=float, default=0.6,
  help="Length of trench")
parser.add_argument('--perpendicular', '-p', action='store_true', default=False,
  help="The trench will be perpendicular to the vector that extends from "
       "the lander's base to the end-effector. By default the trench will be "
       "aligned parallel with this vector. NOTE: A perpendicular grind will "
       "interpret (x, y) as the center of its trench, where as a parallel "
       "trench will interpret (x, y) as the end of the trench nearest to the "
       "lander.")
args = parser.parse_args()

node_helper.call_single_use_action_client(actions.TaskGrindServer,
  x_start=args.x, y_start=args.y, ground_position=args.z,
  depth=args.depth, length=args.length, parallel=not args.perpendicular)
