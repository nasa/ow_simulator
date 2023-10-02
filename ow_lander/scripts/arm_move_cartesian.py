#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper
from ow_lander import constants

import argparse

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Move end-effector by Cartesian coordinates/translation.")
parser.add_argument('--frame', '-f', type=int, default=0,
  choices=constants.FRAME_ID_MAP.keys(),
  help="The frame index corresponding to the frame that orientation and "\
       " position are relative to. " + str(constants.FRAME_ID_MAP))
parser.add_argument('--relative', '-r', action='store_true', default=False,
  help="Pose will be interpreted as relative to the current pose")
parser.add_argument('-x', type=float, default=0,
  help="X-coordinate or the change in the X-coordinate")
parser.add_argument('-y', type=float, default=0,
  help="Y-coordinate or the change in the Y-coordinate")
parser.add_argument('-z', type=float, default=0,
  help="Z-coordinate or the change in the Z-coordinate")
# accept either Euler angles or quaternions as input for orientation
rot_parse = parser.add_mutually_exclusive_group()
rot_parse.add_argument('--euler', '-e', type=float, nargs=3,
  default=[0, 0, 0], metavar=('X', 'Y', 'Z'),
  help="Orientation in Euler angle form. Takes 3 radian values.")
rot_parse.add_argument('--quaternion', '-q', type=float, nargs=4,
  default=[0, 0, 0, 1], metavar=('X', 'Y', 'Z', 'W'),
  help="Orientation in quaternion form. Takes 4 scalars, the first 3 are the" \
       " imaginary parts, and the 4th is the real part.")

args = parser.parse_args()

quat = quaternion_from_euler(*args.euler) if args.euler != [0, 0, 0] \
       else args.quaternion

pose_arg = Pose(
  position=Point(args.x, args.y, args.z),
  orientation=Quaternion(*quat)
)

node_helper.call_single_use_action_client(actions.ArmMoveCartesianServer,
  frame=args.frame, relative=args.relative, pose=pose_arg)
