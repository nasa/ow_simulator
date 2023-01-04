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
parser.add_argument('--frame', '-f', type=int, default=0,
  choices=constants.FRAME_ID_MAP.keys(),
  help="The frame index corresponding to the frame that orientation and "\
       " position relative to.")
parser.add_argument('--relative', '-r', action='store_true', default=False,
  help="If True, pose will be interpreted as relative to the current pose.")
parser.add_argument('-x', type=float, default=0,
  help="X-coordinate or the change in the X-coordinate")
parser.add_argument('-y', type=float, default=0,
  help="Y-coordinate or the change in the Y-coordinate")
parser.add_argument('-z', type=float, default=0,
  help="Z-coordinate or the change in the Z-coordinate")
parser.add_argument('--orientation', '-o', type=float, nargs=4,
  default=[0, 0, 0, 1], metavar=('X', 'Y', 'Z', 'W'),
  help="Orientation in quaternion form. Takes 4 scalars, the first 3 are the" \
       " imaginary parts, and the 4th is the real part.")
args = parser.parse_args()

pose_arg = Pose(
  position=Point(args.x, args.y, args.z),
  orientation=Quaternion(*args.orientation)
)

node_helper.call_single_use_action_client(actions.ArmMoveCartesianServer,
  frame=args.frame, relative=args.relative, pose=pose_arg)
