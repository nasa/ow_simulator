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
  help="Discard sample")
parser.add_argument('--frame', '-f', type=int, default=0,
  choices=constants.FRAME_ID_MAP.keys(),
  help="The index of the frame position will be interpreted in "
       + str(constants.FRAME_ID_MAP))
parser.add_argument('--relative', '-r', action='store_true', default=False,
  help="If True, position will be interpreted in the tool's frame")
parser.add_argument('-x', type=float, default=1.5,
  help="X-coordinate on surface where sample will be discarded")
parser.add_argument('-y', type=float, default=0.8,
  help="Y-coordinate on surface where sample will be discarded")
parser.add_argument('-z', type=float, default=constants.DEFAULT_GROUND_HEIGHT,
  help="Z-coordinate on surface where sample will be discarded")
parser.add_argument('--height', '-h', type=float, default=0.4,
  help="Height above position where sample will be dumped")
args = parser.parse_args()

point_arg = Point(args.x_start, args.y_start, args.z_start)

node_helper.call_single_use_action_client(actions.TaskDiscardSampleServer,
  frame=args.frame, relative=args.relative, point=point_arg, height=args.height)
