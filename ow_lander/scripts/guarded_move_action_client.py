#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper

from geometry_msgs.msg import Point

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Move arm slowly in a direction until a surface is detected. " \
              "DEPRECATED: ArmFindSurface should be used in place of " \
              "GuardedMove")
parser.add_argument('x_start', type=float, nargs='?', default=2.0,
  help='x-coordinates of guarded move starting point')
parser.add_argument('y_start', type=float, nargs='?', default=0.0,
  help='y-coordinates of guarded move starting point')
parser.add_argument('z_start', type=float, nargs='?', default=0.3,
  help='z-coordinates of guarded move starting point')
parser.add_argument('direction_x', type=float, nargs='?', default=0.0,
  help='x-coordinates of the vector normal to the surface')
parser.add_argument('direction_y', type=float, nargs='?', default=0.0,
  help='y-coordinates of the vector normal to the surface')
parser.add_argument('direction_z', type=float, nargs='?', default=1.0,
  help='z-coordinates of the vector normal to the surface')
parser.add_argument('search_distance', type=float, nargs='?', default=0.5,
  help='Total distance end-effector will traverse along the direction.')
args = parser.parse_args()

start_arg = Point(args.x_start, args.y_start, args.z_start)
normal_arg = Point(args.direction_x, args.direction_y, args.direction_z)

node_helper.call_single_use_action_client(actions.GuardedMoveServer,
  start=start_arg, normal=normal_arg, search_distance=args.search_distance)
