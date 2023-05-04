#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper
from ow_lander import constants

from geometry_msgs.msg import Point

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Removes tailings following a grind with a circular scooping "
              "motion.")
parser.add_argument('--frame', '-f', type=int, default=0,
  choices=constants.FRAME_ID_MAP.keys(),
  help="The index of the frame surface position will be interpreted in "
       + str(constants.FRAME_ID_MAP))
parser.add_argument('--relative', '-r', action='store_true', default=False,
  help="Position will be interpreted as relative to the current position")
parser.add_argument('-x', type=float, default=1.65,
  help="X-coordinate on surface where trench starts")
parser.add_argument('-y', type=float, default=0,
  help="Y-coordinate on surface where trench starts")
parser.add_argument('-z', type=float, default=constants.DEFAULT_GROUND_HEIGHT,
  help="Z-coordinate on surface where trench starts")
parser.add_argument('--depth', '-d', type=float, default=0.01,
  help="Desired scooping depth")
parser.add_argument('--perpendicular', '-p', action='store_true', default=False,
  help="Dig a trench that is perpendicular to the vector that extends from "
       "the arm's base to the end-effector. By default the trench will be "
       "aligned parallel with this vector.")
args = parser.parse_args()

point_arg = Point(args.x, args.y, args.z)

node_helper.call_single_use_action_client(actions.TaskScoopCircularServer,
  frame=args.frame, relative=args.relative, point=point_arg,
  depth=args.depth, parallel=not args.perpendicular)
