#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper
from ow_lander import constants

import argparse

from distutils.util import strtobool
from geometry_msgs.msg import Pose, Point, Quaternion

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Move end-effector by Cartesian coordinates/translation.")
parser.add_argument('frame', type=int, nargs='?', default=0,
  choices=constants.FRAME_ID_MAP.keys(),
  help="The frame in which coordinates/translation will be interpreted")
parser.add_argument('relative', type=strtobool, nargs='?', default=False,
  help="If True, pose will be interpreted as relative to the current pose.")
parser.add_argument('x', type=float, nargs='?', default=0,
  help="X-coordinate or the change in the X-coordinate")
parser.add_argument('y', type=float, nargs='?', default=0,
  help="Y-coordinate or the change in the Y-coordinate")
parser.add_argument('z', type=float, nargs='?', default=0,
  help="Z-coordinate or the change in the Z-coordinate")
parser.add_argument('qx', type=float, nargs='?', default=0,
  help="First scalar of the quaternion")
parser.add_argument('qy', type=float, nargs='?', default=0,
  help="Second scalar of the quaternion")
parser.add_argument('qz', type=float, nargs='?', default=0,
  help="Third scalar of the quaternion")
parser.add_argument('qw', type=float, nargs='?', default=0,
  help="Fourth scalar of the quaternion")
args = parser.parse_args()

pose_arg = Pose(
  position=Point(args.x, args.y, args.z),
  orientation=Quaternion(args.qx, args.qy, args.qz, args.qw)
)

node_helper.call_single_use_action_client(actions.ArmMoveCartesianServer,
  frame=args.frame, relative=args.relative, pose=pose_arg)
