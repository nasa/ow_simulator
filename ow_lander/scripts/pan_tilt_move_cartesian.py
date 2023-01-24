#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper
from ow_lander import constants

import argparse

from geometry_msgs.msg import Point

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Moves the antenna mast to look at a position in space.")
parser.add_argument('--frame', '-f', type=int, default=0,
  choices=constants.FRAME_ID_MAP.keys(),
  help="The frame index of the position. " + str(constants.FRAME_ID_MAP))
parser.add_argument('-x', type=float, default=0,
  help="X-coordinate of the position looked at")
parser.add_argument('-y', type=float, default=0,
  help="Y-coordinate of the position looked at")
parser.add_argument('-z', type=float, default=0,
  help="Z-coordinate of the position looked at")
args = parser.parse_args()

point_arg = Point(args.x, args.y, args.z)

node_helper.call_single_use_action_client(actions.PanTiltMoveCartesianServer,
  frame=args.frame, point=point_arg)
