#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper

from geometry_msgs.msg import Point

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('x_start', type=float, nargs='?', default=1.5,
  help='x-coordinates of discard location')
parser.add_argument('y_start', type=float, nargs='?', default=0.8,
  help='y-coordinates of discard location')
parser.add_argument('z_start', type=float, nargs='?', default=0.65,
  help='z-coordinates of discard location')
args = parser.parse_args()

point_arg = Point(args.x_start, args.y_start, args.z_start)

node_helper.call_single_use_action_client(actions.DiscardServer,
  discard=point_arg)
